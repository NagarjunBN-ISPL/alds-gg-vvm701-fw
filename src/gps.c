#include "gps.h"
#include "modem.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "gps";

void gps_init(bool enable) {
    if (!enable) return;   /* zero AT traffic — this is what keeps GPS inert
                             * while gps_enable is false, the default state. */

    /* GNSS and cellular (WWAN) share RF hardware on Quectel modules and don't
     * run at full strength simultaneously unless told which one to favor —
     * documented in the EC200U series GNSS Application Note, and confirmed
     * by field reports of EC200U-CN GNSS commands struggling specifically
     * while a PDP/internet session is active (which it always is here, since
     * gps_init() now runs after network_init() — see the app_main comment on
     * why that ordering was chosen). 0 = GNSS priority, for this initial
     * acquisition window; this trades a small amount of cellular throughput
     * for a real chance at a first fix, not a permanent WWAN downgrade.
     * Non-fatal if rejected (e.g. not supported on this exact firmware) —
     * AT+QGPS=1 below is still attempted either way. */
    char resp[64] = {0};
    if (!modem_at("AT+QGPSCFG=\"priority\",0", "OK", 5000, resp, sizeof(resp)))
        ESP_LOGW(TAG, "AT+QGPSCFG priority=GNSS rejected/unsupported — continuing anyway");

    if (!modem_at("AT+QGPS=1", "OK", 10000, resp, sizeof(resp))) {
        /* Non-fatal, known-benign causes: (a) GNSS already running from a
         * prior boot that didn't power-cycle the modem — an ESP-only reset
         * leaves the modem powered (see modem_hard_reset()'s comment), so
         * re-issuing AT+QGPS=1 can find a session already active (CME 504);
         * (b) no antenna physically connected, which some firmware still
         * accepts here and only ever surfaces later as "never gets a fix" in
         * gps_poll(). Either way, a failure here must never block boot —
         * same philosophy as a genset/radar unit simply not answering. */
        ESP_LOGW(TAG, "AT+QGPS=1 failed/rejected — GNSS may already be running, or no antenna connected");
    } else {
        ESP_LOGI(TAG, "GNSS engine powered on — fix acquisition running in background");
    }
}

void gps_end(void) {
    char resp[64] = {0};
    modem_at("AT+QGPSEND", "OK", 5000, resp, sizeof(resp));
}

/* Copies the field starting at *p up to (not including) the next ',' or end
 * of string into out, then advances *p past the comma. Returns false only
 * when *p is already exhausted (called past the last field) — the final
 * field in a response (no trailing comma) is still a successful extraction. */
static bool next_field(const char **p, char *out, size_t out_size) {
    if (!*p || !**p) { if (out_size) out[0] = '\0'; return false; }
    const char *comma = strchr(*p, ',');
    const char *end    = comma ? comma : *p + strlen(*p);
    size_t len = (size_t)(end - *p);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, *p, len);
    out[len] = '\0';
    *p = comma ? comma + 1 : *p + strlen(*p);
    return true;
}

bool gps_poll(GpsFix_t *out) {
    memset(out, 0, sizeof(*out));

    char resp[256] = {0};
    if (!modem_at("AT+QGPSLOC=2", "+QGPSLOC:", 5000, resp, sizeof(resp))) {
        /* Covers both "no fix yet" (+CME ERROR) and any other error/timeout —
         * modem_at() already treats +CME ERROR as failure, so no separate
         * fix-vs-error distinction is needed. This is the normal state for a
         * long time after boot (cold GNSS acquisition can take minutes), not
         * a fault. */
        return false;
    }

    char *start = strstr(resp, "+QGPSLOC:");
    if (!start) return false;
    start += strlen("+QGPSLOC:");
    while (*start == ' ') start++;

    /* Field order per Quectel's general GNSS AT command set with mode 2
     * (decimal-degree format): UTC,lat,lon,hdop,alt,fix,cog,spkm,spkn,date,
     * nsat. NOT yet confirmed against the EC200U-CN's own AT command manual
     * or a live capture with the antenna connected — see the TODOs on
     * GpsFix_t in gps.h. The lat/lon range check below catches a wrong field
     * order landing an out-of-range number in latitude/longitude, but does
     * NOT itself confirm the other fields (hdop/altitude/fixType/speed/
     * courseDeg) are in the right slots — treat every gps_poll() result as
     * provisional until checked against a real capture. */
    const char *cur = start;
    char field[24];

    if (!next_field(&cur, out->fix_time_utc, sizeof(out->fix_time_utc))) return false;

    if (!next_field(&cur, field, sizeof(field))) return false;
    double lat = atof(field);

    if (!next_field(&cur, field, sizeof(field))) return false;
    double lon = atof(field);

    if (!next_field(&cur, field, sizeof(field))) return false;
    out->hdop = (float)atof(field);

    if (!next_field(&cur, field, sizeof(field))) return false;
    out->altitude_m = (float)atof(field);

    if (!next_field(&cur, field, sizeof(field))) return false;
    out->fix_type = (uint8_t)atoi(field);

    if (!next_field(&cur, field, sizeof(field))) return false;
    out->course_deg = (float)atof(field);

    if (!next_field(&cur, field, sizeof(field))) return false;
    out->speed = (float)atof(field);   /* spkm — speed over ground */

    if (!next_field(&cur, field, sizeof(field))) return false;
    /* spkn (speed in knots) intentionally not stored — spkm above is kept as
     * the one reported speed value. */

    if (!next_field(&cur, out->fix_date, sizeof(out->fix_date))) return false;

    if (!next_field(&cur, field, sizeof(field))) return false;
    out->num_sats = (uint8_t)atoi(field);

    /* Sanity check, not a format re-verification: a real fix's coordinates
     * must land in valid Earth-coordinate range. This catches "the field
     * order is wrong and some other field got parsed as latitude" more
     * usefully than silently publishing a bogus number. */
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        ESP_LOGW(TAG, "AT+QGPSLOC parsed out-of-range coordinates (%.6f,%.6f) — "
                      "field order likely wrong, discarding", lat, lon);
        return false;
    }

    out->latitude  = lat;
    out->longitude = lon;
    out->valid     = true;
    return true;
}

void gps_build_payload(const GpsFix_t *fix, const char *hw_id, const char *ts,
                        char *buf, size_t buf_size) {
    if (!fix->valid) {
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"gps\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
        return;
    }

    snprintf(buf, buf_size,
        "{\"HW_ID\":\"%s\",\"gps\":{"
        "\"lat\":%.6f,\"lon\":%.6f,\"alt_m\":%.1f,\"hdop\":%.1f,"
        "\"numSat\":%u,\"fixType\":%u,\"speed\":%.1f,\"courseDeg\":%.1f,"
        "\"fixTime\":\"%s\",\"fixDate\":\"%s\""
        "},\"TS\":\"%s\"}",
        hw_id,
        fix->latitude, fix->longitude, fix->altitude_m, fix->hdop,
        (unsigned)fix->num_sats, (unsigned)fix->fix_type, fix->speed, fix->course_deg,
        fix->fix_time_utc, fix->fix_date,
        ts);
}
