#include "gps.h"
#include "modem.h"
#include "network.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

static const char *TAG = "gps";

/* ── Time to first fix, and the one-shot antenna check ──────────────────
 * TTFF is measured from boot, or from the last engine re-arm, and logged
 * once: the one number that says whether AGPS and apflash are doing their
 * job on this unit — on every boot, in the field, without a bench. */
static int64_t s_ttff_epoch_us;        /* 0 = boot; else the last re-arm */
static bool    s_ttff_logged;
static bool    s_ant_checked;

/* ── Modem-side settings ────────────────────────────────────────────────
 * Read a QGPSCFG setting and write it only if it differs. The write is
 * saved in the modem's NVRAM (per the EC200U GNSS Application Note), so in
 * practice this issues the write once in a unit's life and a 300 ms query
 * on every later boot. Any error is logged and ignored: none of these is
 * worth failing a boot over, and an older modem firmware may not know the
 * key at all. */
static void gps_cfg_ensure(const char *key, int want)
{
    char cmd[48], resp[96] = {0};
    snprintf(cmd, sizeof(cmd), "AT+QGPSCFG=\"%s\"", key);
    if (!modem_at(cmd, "+QGPSCFG:", 2000, resp, sizeof(resp))) {
        ESP_LOGW(TAG, "QGPSCFG \"%s\": query not supported — left as is", key);
        return;
    }
    /* +QGPSCFG: "apflash",0 */
    const char *p     = strstr(resp, "+QGPSCFG:");
    const char *comma = p ? strchr(p, ',') : NULL;
    int cur = comma ? atoi(comma + 1) : -1;
    if (cur == want) {
        ESP_LOGI(TAG, "QGPSCFG \"%s\" = %d", key, cur);
        return;
    }
    snprintf(cmd, sizeof(cmd), "AT+QGPSCFG=\"%s\",%d", key, want);
    if (modem_at(cmd, "OK", 2000, resp, sizeof(resp)))
        ESP_LOGI(TAG, "QGPSCFG \"%s\" %d -> %d (saved on the modem; effective from its next restart)",
                 key, cur, want);
    else
        ESP_LOGW(TAG, "QGPSCFG \"%s\" set %d rejected — stays %d", key, want, cur);
}

/* ── AGPS ───────────────────────────────────────────────────────────────
 * See GPS_AGPS_* in config.h for what it buys and why it gets its own PDP
 * context. Three modem-side settings, all saved on the modem: the AGPS
 * context's APN and credentials (QICSGP), the assistance server on that
 * context (QAGPSCFG) and the AGPS switch (QAGPS). The first two are rewritten
 * every boot — idempotent, and the APN can change via setconfig; the switch
 * is query-then-set like the QGPSCFG options. Every failure is logged and
 * non-fatal: AGPS is an accelerator, the engine works without it. With
 * GPS_AGPS_ENABLE 0 this turns the saved switch OFF, so the build flag is
 * the truth on the modem too. */
static int agps_switch(void)            /* -1 unknown, else 0 / 1 */
{
    char resp[64] = {0};
    if (!modem_at("AT+QAGPS?", "+QAGPS:", 2000, resp, sizeof(resp))) return -1;
    const char *p = strstr(resp, "+QAGPS:");
    return p ? atoi(p + 7) : -1;
}

static void gps_agps_apply(void)
{
    int cur = agps_switch();
#if GPS_AGPS_ENABLE
    char cmd[240], resp[96] = {0};
    network_build_qicsgp_ctx(GPS_AGPS_PDP_CTX, cmd, sizeof(cmd));
    if (!modem_at(cmd, "OK", 3000, resp, sizeof(resp))) {
        ESP_LOGW(TAG, "AGPS: context %d refused (QICSGP) — left as it was (switch %d)",
                 GPS_AGPS_PDP_CTX, cur);
        return;
    }
    int n = snprintf(cmd, sizeof(cmd), "AT+QAGPSCFG=%d,\"%s\",\"%s\",\"%s\",\"%s\",1,\"%s\"",
                     GPS_AGPS_PDP_CTX, GPS_AGPS_URL, GPS_AGPS_VENDOR_ID, GPS_AGPS_MODEL_ID,
                     GPS_AGPS_PASSWORD, network_apn());
    if (n < 0 || n >= (int)sizeof(cmd) || !modem_at(cmd, "OK", 3000, resp, sizeof(resp))) {
        ESP_LOGW(TAG, "AGPS: server config refused (QAGPSCFG) — not enabled (switch %d)", cur);
        return;
    }
    if (cur == 1) {
        ESP_LOGI(TAG, "AGPS: on — ephemeris fetched over context %d (APN %s) when the engine starts",
                 GPS_AGPS_PDP_CTX, network_apn());
        return;
    }
    if (modem_at("AT+QAGPS=1", "OK", 3000, resp, sizeof(resp)))
        ESP_LOGI(TAG, "AGPS: enabled (saved on the modem) — context %d, APN %s",
                 GPS_AGPS_PDP_CTX, network_apn());
    else
        ESP_LOGW(TAG, "AGPS: AT+QAGPS=1 refused — engine runs unassisted");
#else
    if (cur == 1) {
        if (modem_at("AT+QAGPS=0", "OK", 3000, NULL, 0))
            ESP_LOGW(TAG, "AGPS: switched off on the modem (GPS_AGPS_ENABLE 0)");
        else
            ESP_LOGW(TAG, "AGPS: still on — AT+QAGPS=0 refused");
    } else {
        ESP_LOGI(TAG, "AGPS: off (GPS_AGPS_ENABLE 0)");
    }
#endif
}

/* ── GNSS off, on the modem too ──────────────────────────────────────────
 * gpsEnable false must mean no GPS activity at all, not just no GPS publish.
 * gps_init(true) leaves two options saved in the modem's NVRAM that outlive
 * this firmware's own switch: autogps (the engine starts with the modem) and
 * the AGPS switch (the modem then fetches assistance data over the SIM). Left
 * on, a unit whose GPS was switched off — or one FOTA'd here from a product
 * that had it on — would still run the engine and pull AGPS data after every
 * modem power-up. So: the session is stopped now, and both options are off
 * from the modem's next restart. Query-then-set like the options above: a
 * modem that is already off gets three queries and no write. */
static void gps_modem_off(void)
{
    char resp[64] = {0};
    if (modem_at("AT+QGPS?", "+QGPS:", 2000, resp, sizeof(resp)) && strstr(resp, "+QGPS: 1")) {
        if (modem_at("AT+QGPSEND", "OK", 5000, NULL, 0))
            ESP_LOGI(TAG, "GNSS engine stopped (GPS disabled)");
        else
            ESP_LOGW(TAG, "AT+QGPSEND refused — GNSS engine may still be running");
    }
    gps_cfg_ensure("autogps", 0);
    if (agps_switch() == 1) {
        if (modem_at("AT+QAGPS=0", "OK", 3000, NULL, 0))
            ESP_LOGI(TAG, "AGPS: switched off on the modem (GPS disabled)");
        else
            ESP_LOGW(TAG, "AGPS: still on — AT+QAGPS=0 refused");
    }
}

/* ── GSV summary ─────────────────────────────────────────────────────────
 * "$xxGSV,<msgs>,<msg>,<in view>,<prn>,<elev>,<az>,<cn0>,...*cs": groups of
 * four from the fourth field on; the C/N0 is empty for a satellite known
 * from the almanac but not tracked, and the last group runs into the
 * "*checksum". A satellite is counted only when its group is COMPLETE (the
 * C/N0 field reached, empty or not): the EC200U emits NMEA 4.10, which ends
 * every GSV sentence with a lone signal-ID field — "$GPGSV,1,1,00,0*65" is
 * "no satellites", not one satellite called 0 (office bench, 2026-09-18).
 * Any talker. Pure — see gps.h. */
bool gps_gsv_summary(const char *resp, int strong_db, int *in_view, int *strong, int *best)
{
    int seen = 0, good = 0, top = 0;
    bool any = false;
    if (!resp) return false;
    const char *s = resp;
    while ((s = strstr(s, "GSV,")) != NULL) {
        any = true;
        const char *p = s + 4, *e;
        int  field = 0;
        bool have_prn = false;                      /* this group names a satellite */
        for (;;) {
            e = p;
            while (*e && *e != ',' && *e != '*' && *e != '\r' && *e != '\n') e++;
            if (field >= 3) {                       /* past msgs, msg, in-view */
                int slot = (field - 3) % 4;         /* prn, elev, az, cn0 */
                if (slot == 0) have_prn = (e > p);
                if (slot == 3 && have_prn) {        /* group complete: one satellite */
                    seen++;
                    if (e > p) {
                        int cn0 = atoi(p);
                        if (cn0 > top) top = cn0;
                        if (cn0 >= strong_db) good++;
                    }
                }
            }
            field++;
            if (*e != ',') break;                   /* '*', end of line or of text */
            p = e + 1;                              /* a ',' always opens a field, even an empty last one */
        }
        s = e;
    }
    if (!any) return false;
    if (in_view) *in_view = seen;
    if (strong)  *strong  = good;
    if (best)    *best    = top;
    return true;
}

/* ── Antenna check ───────────────────────────────────────────────────────
 * Once per boot: the satellites in view and their C/N0, from the engine's
 * own GSV sentences. Runs at the first fix — or, more usefully, after
 * GPS_ANT_CHECK_NOFIX_S without one, when "why no fix?" is the question and
 * GSV still answers it (satellites are seen long before they are fixed on).
 * nmeasrc is switched on only around the query and back off after it, as
 * the app note's own example does; gps_init() also forces it off at boot, so
 * a failure here can never leave NMEA output enabled on the AT port. */
static void gps_antenna_check_maybe(bool have_fix)
{
    if (s_ant_checked) return;
    int64_t since = esp_timer_get_time() - s_ttff_epoch_us;
    if (!have_fix && since < (int64_t)GPS_ANT_CHECK_NOFIX_S * 1000000LL) return;
    s_ant_checked = true;

    static char resp[1024];                 /* up to ~8 GSV sentences */
    memset(resp, 0, sizeof(resp));
    if (!modem_at("AT+QGPSCFG=\"nmeasrc\",1", "OK", 2000, resp, sizeof(resp))) {
        ESP_LOGW(TAG, "antenna check skipped — nmeasrc not accepted");
        return;
    }
    bool got = modem_at("AT+QGPSGNMEA=\"GSV\"", "+QGPSGNMEA:", 3000, resp, sizeof(resp));
    (void)modem_at("AT+QGPSCFG=\"nmeasrc\",0", "OK", 2000, NULL, 0);

    int in_view = 0, strong = 0, best = 0;
    if (!got || !gps_gsv_summary(resp, GPS_ANT_CHECK_STRONG_DB, &in_view, &strong, &best)) {
        ESP_LOGW(TAG, "antenna check: no GSV data from the engine (%s)",
                 have_fix ? "fix held" : "no fix yet");
        return;
    }
    if (strong == 0)
        ESP_LOGW(TAG, "antenna check: %d satellites in view, NONE tracked at >= %d dB-Hz (best %d) — "
                      "check the antenna, its cable and placement (roof, clear sky; expect 35-45 dB-Hz)",
                 in_view, GPS_ANT_CHECK_STRONG_DB, best);
    else
        ESP_LOGI(TAG, "antenna check: %d satellites in view, %d tracked at >= %d dB-Hz, best %d dB-Hz (%s)",
                 in_view, strong, GPS_ANT_CHECK_STRONG_DB, best, have_fix ? "fix held" : "no fix yet");
}

static void ttff_note(const GpsFix_t *fix)
{
    if (s_ttff_logged) return;
    s_ttff_logged = true;
    int64_t now = esp_timer_get_time();
    ESP_LOGI(TAG, "first fix %lld s after %s: hdop %.1f, %d satellites",
             (long long)((now - s_ttff_epoch_us) / 1000000),
             s_ttff_epoch_us ? "engine re-arm" : "boot", (double)fix->hdop, fix->nsat);
}

void gps_init(bool enable) {
    if (!enable) {         /* no fix is ever read or published while gps_enable
                            * is false; this only makes sure the MODEM is not
                            * running GNSS or AGPS on its own either */
        gps_modem_off();
        return;
    }

    /* Two saved-on-the-modem options, checked before the engine is started so
     * a fresh modem gets them written on its first boot:
     *
     *   autogps  the GNSS engine starts when the MODEM powers up, ~10 s
     *            before this code runs (it waits for network bring-up, see
     *            main.c). Every boot's time-to-first-fix gets those seconds
     *            back, and the AT+QGPS=1 below is then a formality. Confirmed
     *            on the office unit 2026-09-17: a 30 s power cut, 504 at
     *            AT+QGPS=1 yet no fix until 30 s later — the engine was
     *            already running before we asked.
     *   apflash  hot start after a power cycle. A bowser is powered down
     *            between deliveries; with apflash the engine keeps its
     *            ephemeris in the modem's flash and a start within ~2 h of
     *            the last fix takes seconds instead of a cold start's 30-60 s
     *            under open sky, minutes in a yard. Needs no network, unlike
     *            AGPS. Measured the same day: ~10 min cold, ~30 s hot.
     *
     * Both take effect from the modem's next restart. */
    gps_cfg_ensure("autogps", 1);
    gps_cfg_ensure("apflash", 1);
    /* No NMEA on the AT port. The antenna check enables it around one query
     * and disables it again; this repairs the saved setting if that ever
     * failed half-way. */
    gps_cfg_ensure("nmeasrc", 0);

    gps_agps_apply();

    char resp[64] = {0};
    if (!modem_at("AT+QGPS=1", "OK", 10000, resp, sizeof(resp))) {
        /* 504 = "session already on": with autogps that is the NORMAL path,
         * and it is also what an ESP-only reset produces (the modem stays
         * powered, so the session survives). Anything else is logged and is
         * still non-fatal — a missing antenna is accepted here and only shows
         * as "never gets a fix" in gps_poll(). */
        if (strstr(resp, "504"))
            ESP_LOGI(TAG, "GNSS engine already running (autogps / modem not reset) — acquisition started before this boot reached here");
        else
            ESP_LOGW(TAG, "AT+QGPS=1 not accepted (no OK) — GNSS may not be running");
    } else {
        ESP_LOGI(TAG, "GNSS engine powered on now — autogps will start it earlier from the next modem restart");
    }

#if GPS_TEST_COLD_START
    /* The engine is on here (OK or 504 above): QGPSDEL needs it on, and the
     * wipe applies to the NEXT engine start, hence the stop/start. */
    ESP_LOGE(TAG, "BENCH FLAG GPS_TEST_COLD_START=1: wiping assistance data and restarting the engine — "
                  "this boot is a forced COLD start. Never ship this build.");
    bool del = modem_at("AT+QGPSDEL=0", "OK", 3000, resp, sizeof(resp));
    bool end = modem_at("AT+QGPSEND",   "OK", 5000, resp, sizeof(resp));
    vTaskDelay(pdMS_TO_TICKS(1000));
    bool on  = modem_at("AT+QGPS=1",    "OK", 10000, resp, sizeof(resp));
    ESP_LOGE(TAG, "BENCH: QGPSDEL=0 %s, QGPSEND %s, QGPS=1 %s — TTFF clock starts now",
             del ? "OK" : "FAILED", end ? "OK" : "FAILED", on ? "OK" : "FAILED");
    s_ttff_epoch_us = esp_timer_get_time();
    s_ttff_logged   = false;
    s_ant_checked   = false;
#endif

    /* Constellations are only reported, never changed here: a change takes
     * effect on the modem's next restart and its default is variant-specific
     * (5 = GPS+BDS on EC200U-CN, which is what India wants). Visible in the
     * boot log so a GPS-only unit can be spotted and decided on. */
    if (modem_at("AT+QGPSCFG=\"gnssconfig\"", "+QGPSCFG:", 2000, resp, sizeof(resp))) {
        const char *c = strchr(resp, ',');
        ESP_LOGI(TAG, "GNSS constellations (gnssconfig) = %d  [0 GPS, 3 GPS+GLO+GAL, 4 GPS+GLO, 5 GPS+BDS, 6 GPS+GAL]",
                 c ? atoi(c + 1) : -1);
    }
}

void gps_end(void) {
    gps_modem_off();
}

/* Copies the field starting at *p up to (not including) the next ',' or end
 * of line into out, then advances *p past the comma. Returns false only when
 * *p is already exhausted — the final field (no trailing comma) is still a
 * successful extraction. Stops at CR/LF so the "OK" that follows the reply
 * can never be read as the last field. */
static bool next_field(const char **p, char *out, size_t out_size) {
    if (!*p || !**p || **p == '\r' || **p == '\n') { if (out_size) out[0] = '\0'; return false; }
    const char *s = *p;
    const char *e = s;
    while (*e && *e != ',' && *e != '\r' && *e != '\n') e++;
    size_t len = (size_t)(e - s);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, s, len);
    out[len] = '\0';
    *p = (*e == ',') ? e + 1 : e;
    return true;
}

/* Field order (EC200U GNSS Application Note, AT+QGPSLOC=2):
 *   1 UTC hhmmss.sss   2 lat (-)dd.ddddd   3 lon (-)ddd.ddddd   4 HDOP
 *   5 altitude m       6 fix 2/3           7 COG ddd.mm         8 speed km/h
 *   9 speed knots     10 date ddmmyy      11 nsat
 * Confirmed against a real capture 2026-07-17 (Bangalore, ~847 m, 7 sats). */
bool gps_parse_loc(const char *resp, GpsFix_t *out) {
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!resp) return false;

    const char *start = strstr(resp, "+QGPSLOC:");
    if (!start) return false;
    start += strlen("+QGPSLOC:");
    while (*start == ' ') start++;

    const char *cur = start;
    char field[24];
    char *endp = NULL;

    if (!next_field(&cur, field, sizeof(field))) return false;   /* 1 UTC, discarded */

    if (!next_field(&cur, field, sizeof(field))) return false;   /* 2 lat */
    double lat = strtod(field, &endp);
    if (endp == field) return false;

    if (!next_field(&cur, field, sizeof(field))) return false;   /* 3 lon */
    double lon = strtod(field, &endp);
    if (endp == field) return false;

    /* Positive-logic bounds so NaN/Inf are REJECTED as well as out-of-range:
     * `NaN < -90` and `NaN > 90` are both false, so a negative form would let
     * a garbled field strtod reads as "nan"/"inf" through as a valid fix and
     * publish invalid JSON. This also catches a mode-0 reply (ddmm.mmmm) or a
     * field-order change: 1258.29 is not a latitude. */
    if (!(lat >= -90.0 && lat <= 90.0) || !(lon >= -180.0 && lon <= 180.0)) {
        ESP_LOGW(TAG, "AT+QGPSLOC out-of-range/non-finite coordinates (%.6f,%.6f) — discarded", lat, lon);
        return false;
    }
    /* Reject the exact (0,0) origin ("Null Island"): never a real position,
     * and exactly what empty/garbled coordinate fields collapse to. */
    if (lat == 0.0 && lon == 0.0) {
        ESP_LOGW(TAG, "AT+QGPSLOC returned (0,0) — treating as no fix");
        return false;
    }

    /* Optional quality/motion fields. Absent or garbled -> left at 0; the
     * fix itself stays valid. Clamped so a corrupt field cannot publish a
     * nonsense number. */
    float hdop = 0, spd = 0; int nsat = 0; bool spd_ok = false;
    if (next_field(&cur, field, sizeof(field))) {                /* 4 HDOP */
        double v = strtod(field, &endp);
        if (endp != field && v >= 0.0 && v <= 99.9) hdop = (float)v;
    }
    next_field(&cur, field, sizeof(field));                      /* 5 altitude */
    next_field(&cur, field, sizeof(field));                      /* 6 fix     */
    next_field(&cur, field, sizeof(field));                      /* 7 COG     */
    if (next_field(&cur, field, sizeof(field))) {                /* 8 km/h    */
        double v = strtod(field, &endp);
        if (endp != field && v >= 0.0 && v <= 300.0) { spd = (float)v; spd_ok = true; }
    }
    next_field(&cur, field, sizeof(field));                      /* 9 knots   */
    next_field(&cur, field, sizeof(field));                      /* 10 date   */
    if (next_field(&cur, field, sizeof(field))) {                /* 11 nsat   */
        long v = strtol(field, &endp, 10);
        if (endp != field && v >= 0 && v <= 99) nsat = (int)v;
    }

    out->latitude  = lat;
    out->longitude = lon;
    out->hdop      = hdop;
    out->speed_kmh   = spd;
    out->speed_valid = spd_ok;
    out->nsat        = nsat;
    out->valid     = true;
    return true;
}

bool gps_poll(GpsFix_t *out) {
    memset(out, 0, sizeof(*out));

    char resp[256] = {0};
    if (!modem_at("AT+QGPSLOC=2", "+QGPSLOC:", 5000, resp, sizeof(resp))) {
        /* Covers "no fix yet" (+CME ERROR: 516) and any other error/timeout
         * — modem_at() already treats +CME ERROR as failure. 516 is the
         * normal state for a while after a cold start, not a fault.
         *
         * One error IS worth acting on: 505 ("session not active") means the
         * GNSS engine is OFF. That happens whenever a recovery path power-
         * cycles the modem via PWRKEY (mqtt_reconnect_if_needed's L3,
         * network_init's SIM rescue, FOTA's final-attempt escalation) —
         * gps_init() only runs at boot, so without this re-arm a GPS-enabled
         * unit would publish 404 forever after any such recovery, until the
         * next reboot. Re-arming here restores fixes within a poll cycle or
         * two. (Numeric code matched because this modem reports GNSS errors
         * numerically even with AT+CMEE=2 — see the 2026-07-17 field logs.) */
        if (strstr(resp, "ERROR: 505") || strstr(resp, "not active")) {
            ESP_LOGW(TAG, "GNSS session not active (modem power-cycled?) — re-arming engine");
            gps_init(true);   /* only reachable when gps_enable is true */
            s_ttff_epoch_us = esp_timer_get_time();   /* TTFF now counts from the re-arm */
            s_ttff_logged   = false;
        }
        gps_antenna_check_maybe(false);
        return false;
    }
    bool ok = gps_parse_loc(resp, out);
    if (ok) ttff_note(out);
    gps_antenna_check_maybe(ok);
    return ok;
}

/* Append one formatted optional field to `dst` at offset `n`; returns the new
 * offset, never past the buffer. Worst case for all three fields is 34
 * characters (",\"hdop\":99.9,\"nsat\":99,\"spd\":300.0") against 64. */
static size_t add_field(char *dst, size_t size, size_t n, const char *fmt, ...)
{
    if (n >= size - 1) return n;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(dst + n, size - n, fmt, ap);
    va_end(ap);
    if (w <= 0) return n;
    return (n + (size_t)w < size) ? n + (size_t)w : size - 1;
}

void gps_build_payload(const GpsFix_t *fix, const char *hw_id, const char *ts,
                        char *buf, size_t buf_size) {
    if (!fix->valid) {
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"gps\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
        return;
    }

    /* Optional fields only when reported, so the shape a lat/lon-only
     * consumer expects is unchanged when the modem sends nothing extra. spd
     * goes by the flag, not the value: 0.0 is "stopped" and is published. */
    char extra[64] = "";
    size_t n = 0;
    if (fix->hdop > 0)     n = add_field(extra, sizeof(extra), n, ",\"hdop\":%.1f", (double)fix->hdop);
    if (fix->nsat > 0)     n = add_field(extra, sizeof(extra), n, ",\"nsat\":%d",   fix->nsat);
    if (fix->speed_valid)  (void)add_field(extra, sizeof(extra), n, ",\"spd\":%.1f", (double)fix->speed_kmh);

    snprintf(buf, buf_size,
        "{\"HW_ID\":\"%s\",\"gps\":{\"lat\":%.6f,\"lon\":%.6f%s},\"TS\":\"%s\"}",
        hw_id, fix->latitude, fix->longitude, extra, ts);
}
