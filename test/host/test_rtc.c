/* Host harness for the clock-setting path in src/modem.c — qntp_time(),
 * parse_and_set_rtc() and days_from_civil() are pulled out of the real file by
 * extract.py, with settimeofday/gettimeofday redirected to a simulated clock.
 *
 * What it pins: an AT+QNTP result sets the clock to the right UTC second
 * (offset applied, 2- or 4-digit year), anything but a clean "+QNTP: 0,"..."
 * sets nothing, and NTP goes through the same range and jump checks as
 * network time. Expected values come from glibc's timegm(), not from the code
 * under test. */
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

int g_log_quiet = 1;
static const char *TAG = "modem";

/* ── simulated clock ───────────────────────────────────────────────────── */
static struct timeval s_clock;
static int sim_settimeofday(const struct timeval *tv, const void *tz) { (void)tz; s_clock = *tv; return 0; }
static int sim_gettimeofday(struct timeval *tv, void *tz) { (void)tz; *tv = s_clock; return 0; }
#define settimeofday sim_settimeofday
#define gettimeofday sim_gettimeofday

#include "esp_log.h"

static bool s_time_synced = false;
#define RTC_RESYNC_REANCHOR_AFTER 3

#include "extracted_rtc.inc"

/* ── test scaffolding ──────────────────────────────────────────────────── */
static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do {                                        \
        checks++;                                                          \
        if (!(cond)) { fails++;                                            \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__); }                  \
    } while (0)

static time_t utc(int y, int mo, int d, int h, int mi, int s) {
    struct tm t = {0};
    t.tm_year = y - 1900; t.tm_mon = mo - 1; t.tm_mday = d;
    t.tm_hour = h; t.tm_min = mi; t.tm_sec = s;
    return timegm(&t);
}

/* Feed one +QNTP line to the path ntp_sync() runs; true if the clock was set. */
static bool ntp_line(const char *resp) {
    const char *t = qntp_time(resp);
    return t && parse_and_set_rtc(t, false);
}

int main(void) {
    const time_t T = utc(2026, 9, 28, 7, 12, 33);

    /* [1] UTC reply, OK first then the result — the way modem_at() hands it over. */
    s_time_synced = false; s_clock.tv_sec = 0;
    CHECK(ntp_line("\r\nOK\r\n\r\n+QNTP: 0,\"2026/09/28,07:12:33+00\"\r\n") && s_clock.tv_sec == T,
          "[1] UTC: got %lld want %lld", (long long)s_clock.tv_sec, (long long)T);

    /* [2] Local time with the IST offset (+22 quarter-hours) lands on the same UTC. */
    s_clock.tv_sec = 0;
    CHECK(ntp_line("+QNTP: 0,\"2026/09/28,12:42:33+22\"") && s_clock.tv_sec == T,
          "[2] IST: got %lld want %lld", (long long)s_clock.tv_sec, (long long)T);

    /* [3] A two-digit year, as some firmware prints it. */
    s_clock.tv_sec = 0;
    CHECK(ntp_line("+QNTP: 0,\"26/09/28,07:12:33+00\"") && s_clock.tv_sec == T,
          "[3] 2-digit year: got %lld want %lld", (long long)s_clock.tv_sec, (long long)T);

    /* [4] Anything but a clean success sets nothing. */
    static const char *BAD[] = {
        "+QNTP: 561",                                   /* modem error code          */
        "+QNTP: 1,\"2026/09/28,07:12:33+00\"",          /* non-zero err with a time  */
        "+QNTP: 0,2026/09/28,07:12:33+00",              /* no quotes                 */
        "+QNTP: 0,\"1980/01/06,00:00:03+00\"",          /* modem placeholder date    */
        "+QNTP: 0,\"2080/01/06,00:00:03+00\"",          /* past the 2050 cap         */
        "+QNTP: 0,\"2026/13/28,07:12:33+00\"",          /* month 13                  */
        "OK",                                           /* no result line at all     */
        "",
    };
    for (size_t i = 0; i < sizeof(BAD) / sizeof(BAD[0]); i++) {
        s_time_synced = false; s_clock.tv_sec = 12345;
        CHECK(!ntp_line(BAD[i]) && s_clock.tv_sec == 12345, "[4] accepted: %s", BAD[i]);
    }
    CHECK(qntp_time(NULL) == NULL, "[4] NULL must not crash");

    /* [5] Once synced, NTP gets the same jump guard as network time: a result
     *     3 days off is refused twice, then accepted on the third (re-anchor). */
    s_time_synced = true; s_clock.tv_sec = T;
    const char *off = "+QNTP: 0,\"2026/10/01,07:12:33+00\"";
    CHECK(!ntp_line(off) && s_clock.tv_sec == T, "[5] 1st >24 h jump must be refused");
    CHECK(!ntp_line(off) && s_clock.tv_sec == T, "[5] 2nd >24 h jump must be refused");
    CHECK(ntp_line(off) && s_clock.tv_sec == utc(2026, 10, 1, 7, 12, 33), "[5] 3rd must re-anchor");

    /* [6] A normal re-sync a few seconds off is simply taken. */
    s_clock.tv_sec = T - 5;
    CHECK(ntp_line("+QNTP: 0,\"2026/09/28,07:12:33+00\"") && s_clock.tv_sec == T, "[6] small correction");

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
