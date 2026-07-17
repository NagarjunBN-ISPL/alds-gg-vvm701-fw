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

    char resp[64] = {0};
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
        /* Covers both "no fix yet" (+CME ERROR 516) and any other error/
         * timeout — modem_at() already treats +CME ERROR as failure, so no
         * separate fix-vs-error distinction is needed. "No fix yet" is the
         * normal state for a long time after boot (cold GNSS acquisition can
         * take minutes), not a fault.
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
        }
        return false;
    }

    char *start = strstr(resp, "+QGPSLOC:");
    if (!start) return false;
    start += strlen("+QGPSLOC:");
    while (*start == ' ') start++;

    /* Field order: UTC,lat,lon,hdop,alt,fix,cog,spkm,spkn,date,nsat. Confirmed
     * against a real capture on 2026-07-17 (Bangalore, ~847m elevation,
     * 7 satellites, 3D fix). Only lat/lon (fields 2 and 3) are kept — see the
     * comment on GpsFix_t in gps.h — so parsing stops right after longitude;
     * the remaining 8 fields on the wire are never read. */
    const char *cur = start;
    char field[24];

    if (!next_field(&cur, field, sizeof(field))) return false;   /* UTC time, discarded */

    if (!next_field(&cur, field, sizeof(field))) return false;
    double lat = atof(field);

    if (!next_field(&cur, field, sizeof(field))) return false;
    double lon = atof(field);

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
        "{\"HW_ID\":\"%s\",\"gps\":{\"lat\":%.6f,\"lon\":%.6f},\"TS\":\"%s\"}",
        hw_id, fix->latitude, fix->longitude, ts);
}
