/* The AT+QGPSLOC=2 parser and the GPS payload builder, EXTRACTED from
 * src/gps.c and run under ASan + UBSan. The parser reads a modem reply that
 * is copied verbatim into a JSON message, so a wrong field, a non-finite
 * number or a one-byte overread here is a bad publish, not a near miss.
 * Field order per the EC200U GNSS Application Note (AT+QGPSLOC=2):
 * UTC,lat,lon,HDOP,alt,fix,COG,spkm,spkn,date,nsat. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>

#include "gps.h"
#include "esp_log.h"

int g_log_quiet = 1;
static const char *TAG __attribute__((unused)) = "gps";

#include "extracted_gps.inc"

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        if (fails < 30) printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static uint32_t rng_s = 0x6A75CE01;
static uint32_t rnd(void) {
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 17; rng_s ^= rng_s << 5;
    return rng_s;
}

/* Parse from a heap buffer sized exactly to the text, so an overread is a
 * hard ASan failure. */
static bool parse_exact(const char *text, GpsFix_t *fix) {
    size_t n = strlen(text);
    char *b = malloc(n + 1);
    memcpy(b, text, n + 1);
    bool ok = gps_parse_loc(b, fix);
    free(b);
    return ok;
}

static bool near(double a, double b) { return fabs(a - b) < 1e-9; }

int main(void) {
    GpsFix_t f;

    /* ── 1. Real-shaped reply (Bangalore, 3D fix, 7 sats), with the trailing
     *       OK the modem appends after the URC line. */
    {
        const char *r = "\r\n+QGPSLOC: 061951.000,12.971599,77.594566,1.2,847.5,3,0.00,0.0,0.0,170926,07\r\n\r\nOK\r\n";
        CHECK(parse_exact(r, &f), "real reply parses");
        CHECK(f.valid && near(f.latitude, 12.971599) && near(f.longitude, 77.594566), "lat/lon exact (%.6f,%.6f)", f.latitude, f.longitude);
        CHECK(fabsf(f.hdop - 1.2f) < 1e-6f, "hdop 1.2 (got %.2f)", f.hdop);
        CHECK(f.nsat == 7, "nsat 7 (got %d)", f.nsat);
        CHECK(f.speed_kmh == 0.0f && f.speed_valid, "speed 0 when stationary, and it IS a reading");
    }

    /* ── 2. Moving: speed and course populated, southern/western hemisphere. */
    {
        const char *r = "+QGPSLOC: 120000.000,-33.868820,-151.209290,0.8,12.0,3,182.30,58.4,31.5,170926,11\r\nOK";
        CHECK(parse_exact(r, &f), "moving reply parses");
        CHECK(near(f.latitude, -33.868820) && near(f.longitude, -151.209290), "negative coordinates kept");
        CHECK(fabsf(f.speed_kmh - 58.4f) < 1e-5f && f.speed_valid, "speed 58.4 km/h (got %.1f)", f.speed_kmh);
        CHECK(f.nsat == 11, "nsat 11");
    }

    /* ── 3. lat/lon only — older firmware / truncated line — still a fix. */
    {
        CHECK(parse_exact("+QGPSLOC: 061951.000,12.971599,77.594566", &f) && f.valid, "lat/lon-only reply is a fix");
        CHECK(f.hdop == 0 && f.nsat == 0 && f.speed_kmh == 0 && !f.speed_valid, "optional fields default to 0, speed not a reading");
        CHECK(parse_exact("+QGPSLOC: 061951.000,12.971599,77.594566\r\nOK\r\n", &f) && f.valid, "lat/lon-only with OK");
    }

    /* ── 4. Rejections. */
    {
        CHECK(!parse_exact("+QGPSLOC: 061951.000,,77.594566,1.2", &f) && !f.valid, "empty latitude rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,12.971599,,1.2", &f), "empty longitude rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,nan,77.594566", &f), "nan latitude rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,12.9,inf", &f), "inf longitude rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,0.000000,0.000000,1.0,0.0,2,0,0,0,170926,03", &f), "(0,0) rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,1258.2960N,07735.6740E,1.2", &f), "mode-0 ddmm format rejected (out of range)");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,91.0,77.5", &f), "lat > 90 rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,12.9,-180.5", &f), "lon < -180 rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000,abc,77.5", &f), "non-numeric latitude rejected");
        CHECK(!parse_exact("+CME ERROR: 516", &f), "516 (no fix) rejected");
        CHECK(!parse_exact("+CME ERROR: 505", &f), "505 (session off) rejected");
        CHECK(!parse_exact("", &f), "empty rejected");
        CHECK(!parse_exact("OK", &f), "bare OK rejected");
        CHECK(!parse_exact("+QGPSLOC:", &f), "URC with no fields rejected");
        CHECK(!parse_exact("+QGPSLOC: 061951.000", &f), "URC with only UTC rejected");
        CHECK(!gps_parse_loc(NULL, &f), "NULL reply rejected");
        CHECK(!gps_parse_loc("+QGPSLOC: 1,2,3", NULL), "NULL out rejected");
    }

    /* ── 5. Garbled optional fields never poison a good fix. */
    {
        CHECK(parse_exact("+QGPSLOC: 061951.000,12.971599,77.594566,xx,847.5,3,0.00,yy,0.0,170926,zz", &f) && f.valid,
              "garbled optional fields: fix still valid");
        CHECK(f.hdop == 0 && f.speed_kmh == 0 && !f.speed_valid && f.nsat == 0, "garbled optional fields -> 0, speed not a reading");
        CHECK(parse_exact("+QGPSLOC: 061951.000,12.971599,77.594566,999,847.5,3,0.00,9999,0.0,170926,999", &f) && f.valid,
              "out-of-range optional fields: fix still valid");
        CHECK(f.hdop == 0 && f.speed_kmh == 0 && !f.speed_valid && f.nsat == 0, "out-of-range optional fields -> 0, speed not a reading");
        /* A field longer than the 24-byte field buffer is truncated, not overrun. */
        CHECK(parse_exact("+QGPSLOC: 061951.000,12.971599000000000000000000000001,77.594566", &f) && f.valid,
              "over-long field truncated safely");
    }

    /* ── 6. The URC preceded by another URC or noise (modem_at drains). */
    {
        const char *r = "+QMTSTAT: 0,1\r\n+QGPSLOC: 061951.000,12.971599,77.594566,1.2,847.5,3,0.00,0.0,0.0,170926,07\r\nOK";
        CHECK(parse_exact(r, &f) && f.valid && f.nsat == 7, "URC after another URC");
    }

    /* ── 7. Payload builder: shape, optional fields, and the 404 fallback. */
    {
        char buf[192];
        GpsFix_t nofix = {0};
        gps_build_payload(&nofix, "865510083149163", "2026-09-17 06:10:20", buf, sizeof(buf));
        CHECK(strcmp(buf, "{\"HW_ID\":\"865510083149163\",\"gps\":{\"data\":\"404\"},\"TS\":\"2026-09-17 06:10:20\"}") == 0,
              "404 payload exact: %s", buf);

        GpsFix_t full = { .valid = true, .latitude = 12.971599, .longitude = 77.594566, .hdop = 1.2f, .speed_kmh = 58.4f, .speed_valid = true, .nsat = 7 };
        gps_build_payload(&full, "865510083149163", "2026-09-17 06:10:20", buf, sizeof(buf));
        CHECK(strcmp(buf, "{\"HW_ID\":\"865510083149163\",\"gps\":{\"lat\":12.971599,\"lon\":77.594566,\"hdop\":1.2,\"nsat\":7,\"spd\":58.4},\"TS\":\"2026-09-17 06:10:20\"}") == 0,
              "full payload exact: %s", buf);

        GpsFix_t bare = { .valid = true, .latitude = 12.971599, .longitude = 77.594566 };
        gps_build_payload(&bare, "865510083149163", "2026-09-17 06:10:20", buf, sizeof(buf));
        CHECK(strcmp(buf, "{\"HW_ID\":\"865510083149163\",\"gps\":{\"lat\":12.971599,\"lon\":77.594566},\"TS\":\"2026-09-17 06:10:20\"}") == 0,
              "lat/lon-only payload keeps the legacy shape: %s", buf);

        /* The field-log case of 2026-09-17: a stationary bowser must publish
         * spd 0.0, not drop the key — an absent spd is indistinguishable from
         * "not reported". Straight from a real reply so the whole path is
         * covered, not just the builder. */
        const char *r = "+QGPSLOC: 110952.000,12.887360,77.641840,5.9,815.1,3,000.00,0.0,0.0,170926,08\r\nOK";
        CHECK(parse_exact(r, &f) && f.valid && f.speed_valid && f.speed_kmh == 0.0f, "stationary reply parses with a valid 0 speed");
        gps_build_payload(&f, "860710081332507", "2026-09-17 11:09:52", buf, sizeof(buf));
        CHECK(strcmp(buf, "{\"HW_ID\":\"860710081332507\",\"gps\":{\"lat\":12.887360,\"lon\":77.641840,\"hdop\":5.9,\"nsat\":8,\"spd\":0.0},\"TS\":\"2026-09-17 11:09:52\"}") == 0,
              "stationary payload carries spd 0.0: %s", buf);
        CHECK(parse_exact("+QGPSLOC: 110952.000,12.887360,77.641840,5.9,815.1,3,000.00,zz,0.0,170926,08", &f) && !f.speed_valid, "garbled speed -> not a reading");
        gps_build_payload(&f, "860710081332507", "2026-09-17 11:09:52", buf, sizeof(buf));
        CHECK(strstr(buf, "spd") == NULL && strstr(buf, "\"nsat\":8}") != NULL, "garbled speed -> no spd key: %s", buf);

        /* Worst case must fit the main.c buffer with margin. */
        GpsFix_t worst = { .valid = true, .latitude = -89.999999, .longitude = -179.999999, .hdop = 99.9f, .speed_kmh = 300.0f, .speed_valid = true, .nsat = 99 };
        gps_build_payload(&worst, "865510083149163", "2026-09-17 06:10:20", buf, sizeof(buf));
        CHECK(strlen(buf) < 160, "worst-case payload %zu bytes < 160", strlen(buf));
        CHECK(buf[strlen(buf) - 1] == '}', "worst-case payload not truncated");
    }

    /* ── 8. Fuzz: random bytes and mutated real replies. No crash, no
     *       overread, and never a "valid" fix with an out-of-range value. */
    {
        const char base[] = "+QGPSLOC: 061951.000,12.971599,77.594566,1.2,847.5,3,0.00,58.4,31.5,170926,07\r\nOK\r\n";
        static const char alphabet[] = "0123456789.,-+eEnaifNAIF \r\nOK+QGPSLOC:xyz";
        for (int i = 0; i < 40000; i++) {
            char m[160];
            size_t len;
            if (i & 1) {
                len = sizeof(base) - 1;
                memcpy(m, base, len);
                int nmut = 1 + (int)(rnd() % 4);
                for (int k = 0; k < nmut; k++) {
                    size_t pos = rnd() % len;
                    m[pos] = alphabet[rnd() % (sizeof(alphabet) - 1)];
                }
            } else {
                len = rnd() % 120;
                for (size_t k = 0; k < len; k++) m[k] = alphabet[rnd() % (sizeof(alphabet) - 1)];
            }
            m[len] = '\0';
            bool ok = parse_exact(m, &f);
            if (ok) {
                CHECK(f.latitude >= -90 && f.latitude <= 90 && f.longitude >= -180 && f.longitude <= 180,
                      "fuzz: valid fix out of range [%s]", m);
                CHECK(!(f.latitude == 0 && f.longitude == 0), "fuzz: (0,0) accepted [%s]", m);
                CHECK(f.hdop >= 0 && f.hdop <= 99.9f && f.speed_kmh >= 0 && f.speed_kmh <= 300 && f.nsat >= 0 && f.nsat <= 99,
                      "fuzz: optional field out of clamp [%s]", m);
                char buf[192];
                gps_build_payload(&f, "865510083149163", "2026-09-17 06:10:20", buf, sizeof(buf));
                CHECK(strlen(buf) < sizeof(buf) - 1 && buf[strlen(buf) - 1] == '}', "fuzz: payload truncated [%s]", m);
            } else {
                CHECK(!f.valid, "fuzz: false but valid set");
            }
        }
    }

    /* ── 9. GSV summary (the antenna check). Two talkers, empty C/N0 on
     *       untracked satellites, a group that ends in the checksum. */
    {
        static const char GSV[] =
            "\r\n+QGPSGNMEA: $GPGSV,3,1,11,01,45,120,38,02,20,050,,05,60,200,42,12,10,330,29*7C\r\n"
            "+QGPSGNMEA: $GPGSV,3,2,11,13,30,090,33,15,55,270,40,17,05,010,,19,75,180,44*70\r\n"
            "+QGPSGNMEA: $GPGSV,3,3,11,24,15,300,31,25,40,140,36,28,25,220,*55\r\n"
            "+QGPSGNMEA: $BDGSV,1,1,03,201,50,160,41,203,30,100,35,206,70,090,*6B\r\n"
            "\r\nOK\r\n";
        int v = -1, s = -1, b = -1;
        CHECK(gps_gsv_summary(GSV, 30, &v, &s, &b), "gsv: parses");
        CHECK(v == 14 && s == 9 && b == 44, "gsv: 14 in view / 9 strong / best 44, got %d/%d/%d", v, s, b);
        CHECK(gps_gsv_summary(GSV, 45, &v, &s, &b) && s == 0 && b == 44, "gsv: threshold above best -> 0 strong");
        CHECK(gps_gsv_summary(GSV, 30, NULL, NULL, NULL), "gsv: NULL outputs tolerated");
        CHECK(!gps_gsv_summary("\r\nOK\r\n", 30, &v, &s, &b), "gsv: no sentence -> false");
        CHECK(!gps_gsv_summary("", 30, &v, &s, &b), "gsv: empty -> false");
        CHECK(!gps_gsv_summary("+CME ERROR: 507\r\n", 30, &v, &s, &b), "gsv: 507 -> false");
        { const char *one = "$GNGSV,1,1,01,07,,,*41\r\n";
          CHECK(gps_gsv_summary(one, 30, &v, &s, &b) && v == 1 && s == 0 && b == 0, "gsv: seen but untracked"); }
        /* NMEA 4.10, as the EC200U emits it: a lone signal-ID field closes
         * every sentence. Verbatim from the office unit with no antenna
         * (2026-09-18) — this was counted as "2 satellites in view". */
        { const char *none = "\r\n+QGPSGNMEA: $GPGSV,1,1,00,0*65\r\n+QGPSGNMEA: $GAGSV,1,1,00,0*74\r\n\r\nOK\r\n";
          CHECK(gps_gsv_summary(none, 30, &v, &s, &b) && v == 0 && s == 0 && b == 0,
                "gsv 4.10: empty sentences -> 0 in view, got %d/%d/%d", v, s, b); }
        { const char *sig = "$GPGSV,2,1,05,01,45,120,38,02,20,050,,05,60,200,42,12,10,330,29,1*7C\r\n"
                            "$GPGSV,2,2,05,13,30,090,33,1*70\r\n";
          CHECK(gps_gsv_summary(sig, 30, &v, &s, &b) && v == 5 && s == 3 && b == 42,
                "gsv 4.10: signal ID after 4 and after 1 satellites not counted, got %d/%d/%d", v, s, b); }
        /* Mutations: never a crash, never more strong than seen. */
        size_t n = strlen(GSV);
        for (int it = 0; it < 20000; it++) {
            char *m = malloc(n + 1);
            memcpy(m, GSV, n + 1);
            int hits = 1 + (int)(rnd() % 8);
            for (int h = 0; h < hits; h++) m[rnd() % n] = (char)(rnd() % 256);
            size_t cut = rnd() % (n + 1);
            if (rnd() & 1) m[cut] = '\0';
            v = s = b = -1;
            bool ok = gps_gsv_summary(m, 30, &v, &s, &b);
            if (ok) CHECK(v >= 0 && s >= 0 && s <= v && b >= 0, "gsv fuzz: counts %d/%d/%d", v, s, b);
            free(m);
        }
    }

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
