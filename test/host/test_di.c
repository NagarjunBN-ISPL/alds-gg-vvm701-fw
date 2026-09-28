/* Host harness for the DI transition queue in src/di.c — di_enqueue() and
 * di_publish_buffer() are pulled out of the real file by extract.py.
 *
 * What it pins: a transition is published with the time it was SEEN, not the
 * time it is published (after an outage the two differ by hours); a change
 * seen before the clock was set falls back to the publish time; order is
 * kept; a full queue drops its oldest; a failed publish loses nothing. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "di.h"
#include "freertos/semphr.h"

int g_log_quiet = 1;

/* ── the globals di.c owns ─────────────────────────────────────────────── */
static int s_mutex_obj;
static SemaphoreHandle_t di_mutex = &s_mutex_obj;
DIEntry_t    di_fifo[DI_FIFO_SIZE];
volatile int di_head = 0, di_tail = 0, di_count = 0;

/* ── stubs ─────────────────────────────────────────────────────────────── */
static bool s_lock_fails = false;
int  xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) { (void)s; (void)t; return s_lock_fails ? 0 : 1; }
void xSemaphoreGive(SemaphoreHandle_t s) { (void)s; }

static char s_clock[24] = "1970-01-01 00:00:00";
static bool s_clock_ok  = false;
bool rtc_timestamp(char *buf, size_t size) {
    snprintf(buf, size, "%s", s_clock);
    return s_clock_ok;
}

static char s_sent[128][400];
static int  s_n_sent   = 0;
static int  s_fail_at  = -1;     /* publish number that fails, -1 = none */
static int  s_attempts = 0;
bool mqtt_publish(const char *topic, const char *payload) {
    (void)topic;
    if (s_attempts++ == s_fail_at) return false;
    if (s_n_sent < 128) snprintf(s_sent[s_n_sent++], sizeof(s_sent[0]), "%s", payload);
    return true;
}

#include "extracted_di.inc"

/* ── test scaffolding ──────────────────────────────────────────────────── */
static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do {                                        \
        checks++;                                                          \
        if (!(cond)) { fails++;                                            \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__); }                  \
    } while (0)

static void reset(void) {
    memset(di_fifo, 0, sizeof(di_fifo));
    di_head = di_tail = di_count = 0;
    s_n_sent = 0; s_fail_at = -1; s_attempts = 0; s_lock_fails = false;
}

static void at(const char *ts, bool ok) { snprintf(s_clock, sizeof(s_clock), "%s", ts); s_clock_ok = ok; }

#define HW  "35056817171133811"
#define NOW "2026-09-28 13:00:00"

int main(void) {
    char want[400];

    /* [1] DG on at 10:00, off at 12:00, link back at 13:00: each keeps its own time. */
    reset();
    at("2026-09-28 10:00:00", true);  di_enqueue("{\"DI1\":1}");
    at("2026-09-28 12:00:00", true);  di_enqueue("{\"DI1\":0}");
    bool ok = di_publish_buffer("t", HW, NOW, 1);
    CHECK(ok && s_n_sent == 2 && di_count == 0, "[1] ok=%d sent=%d left=%d", ok, s_n_sent, di_count);
    CHECK(strcmp(s_sent[0], "{\"HW_ID\":\"" HW "\",\"di\":{\"DI1\":1},\"TS\":\"2026-09-28 10:00:00\"}") == 0,
          "[1] first: %s", s_sent[0]);
    CHECK(strcmp(s_sent[1], "{\"HW_ID\":\"" HW "\",\"di\":{\"DI1\":0},\"TS\":\"2026-09-28 12:00:00\"}") == 0,
          "[1] second: %s", s_sent[1]);

    /* [2] Seen before the clock was set: published with the publish time,
     *     never with 1970. */
    reset();
    at("1970-01-01 00:00:10", false); di_enqueue("{\"DI2\":1}");
    di_publish_buffer("t", HW, NOW, 2);
    CHECK(s_n_sent == 1 && strstr(s_sent[0], "\"TS\":\"" NOW "\""), "[2] %s", s_sent[0]);

    /* [3] Full queue: the oldest go, the newest DI_FIFO_SIZE survive in order. */
    reset();
    for (int i = 0; i < DI_FIFO_SIZE + 5; i++) {
        char p[32], t[24];
        snprintf(p, sizeof(p), "{\"DI1\":%d}", i);
        snprintf(t, sizeof(t), "2026-09-28 %02d:%02d:00", 10 + i / 60, i % 60);
        at(t, true);
        di_enqueue(p);
    }
    CHECK(di_count == DI_FIFO_SIZE, "[3] count %d, want %d", di_count, DI_FIFO_SIZE);
    di_publish_buffer("t", HW, NOW, 1);
    snprintf(want, sizeof(want), "\"di\":{\"DI1\":%d}", 5);
    CHECK(s_n_sent == DI_FIFO_SIZE && strstr(s_sent[0], want), "[3] first kept: %s", s_sent[0]);
    snprintf(want, sizeof(want), "\"di\":{\"DI1\":%d}", DI_FIFO_SIZE + 4);
    CHECK(strstr(s_sent[DI_FIFO_SIZE - 1], want) != NULL, "[3] last kept: %s", s_sent[DI_FIFO_SIZE - 1]);

    /* [4] A failed publish loses nothing: the entry is retried, with its time. */
    reset();
    at("2026-09-28 10:00:00", true);  di_enqueue("{\"DI1\":1}");
    at("2026-09-28 11:00:00", true);  di_enqueue("{\"DI1\":0}");
    s_fail_at = 1;
    ok = di_publish_buffer("t", HW, NOW, 1);
    CHECK(!ok && s_n_sent == 1 && di_count == 1, "[4] first pass: ok=%d sent=%d left=%d", ok, s_n_sent, di_count);
    ok = di_publish_buffer("t", HW, NOW, 1);
    CHECK(ok && s_n_sent == 2 && strstr(s_sent[1], "\"TS\":\"2026-09-28 11:00:00\""), "[4] retry: %s", s_sent[1]);

    /* [5] Lock not obtained: nothing queued, so di_task retries on its next poll. */
    reset();
    s_lock_fails = true;
    CHECK(!di_enqueue("{\"DI1\":1}") && di_count == 0, "[5] enqueue without the lock must report failure");

    /* [6] The longest possible payload survives intact. */
    reset();
    at("2026-09-28 10:00:00", true);
    const char *big = "{\"DI1\":-2147483648,\"DI2\":-2147483648,\"DI3\":-2147483648,\"DI4\":-2147483648}";
    di_enqueue(big);
    di_publish_buffer("t", HW, NOW, 15);
    CHECK(s_n_sent == 1 && strstr(s_sent[0], big), "[6] %s", s_sent[0]);

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
