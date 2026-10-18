/* Host harness for the REAL src/yenen.c — compiled natively against a
 * simulated Yenen pump. Nothing here is a copy of firmware logic: the frames,
 * checksums, retries, address-echo check and payload all come from the
 * production source.
 *
 * The driver only READS (Command 2). A price change is the cloud's frame,
 * carried by the rsp tunnel (test_rsp_tunnel.c); the gateway-built RSP write
 * and its suite were removed on 2026-10-18. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "yenen.h"
#include "config.h"
#include "driver/uart.h"

int g_log_quiet = 1;
int g_wdt_feeds = 0;

/* ── virtual clock ─────────────────────────────────────────────────────── */
static int64_t s_now_us = 1000000;
int64_t esp_timer_get_time(void) { return s_now_us; }
void sim_clock_advance_ms(int64_t ms) { s_now_us += ms * 1000; }
void vTaskDelay(uint32_t ticks) { sim_clock_advance_ms((int64_t)ticks); }

int esp_task_wdt_status(void *h) { (void)h; return 0; }
int esp_task_wdt_reset(void) { g_wdt_feeds++; return 0; }

/* ── simulated pump ────────────────────────────────────────────────────── */
typedef struct {
    bool     present;        /* answers reads at all                          */
    bool     idle;           /* status field 00 vs 11                         */
    uint16_t price;          /* unit price it reports                         */
    bool     wrong_echo;     /* answer with the OTHER side's address          */
    int      fuzz;           /* answer with random bytes: a noisy bus, a
                              * wrong-protocol device, a half-powered pump    */
    int      a_frames;       /* Command 2 frames received                     */
    uint8_t  last_tx[32];
    int      last_tx_len;
} Pump_t;

static Pump_t P;

static uint8_t rx[256];
static int rx_len, rx_pos;

static void rx_reset(void) { rx_len = rx_pos = 0; }
static void rx_push(const uint8_t *b, int n) {
    if (rx_len + n > (int)sizeof(rx)) n = (int)sizeof(rx) - rx_len;
    memcpy(rx + rx_len, b, (size_t)n);
    rx_len += n;
}

/* Same checksum the protocol uses — recomputed independently here so the test
 * would catch the firmware and the model drifting apart. */
static void ck(const uint8_t *f, int len, uint8_t *out) {
    uint8_t x = 0;
    for (int i = 0; i < len; i++) x ^= f[i];
    out[0] = (uint8_t)(((x >> 4) & 0x0F) + 0x30);
    out[1] = (uint8_t)((x & 0x0F) + 0x30);
}

static uint8_t nzcode(uint8_t head, uint8_t side) {
    return (uint8_t)((side ? '5' : '0') + (head - 1));
}

/* 51-byte Command 2 reply, laid out per the cloud guide's appendix. */
static void build_read_reply(uint8_t head, uint8_t side, uint16_t price, bool idle,
                             uint8_t *out, int *out_len) {
    char pay[64];
    int  n = 0;
    n += sprintf(pay + n, "49");                       /* [0:2]   code        */
    n += sprintf(pay + n, "0%c%c", '0' + head, nzcode(head, side)); /* [2:5]  */
    n += sprintf(pay + n, "A");                        /* [5:6]   cmd echo    */
    n += sprintf(pay + n, "%04u", price);              /* [6:10]  unit price  */
    n += sprintf(pay + n, "0000");                     /* [10:14] preset amt  */
    n += sprintf(pay + n, "0000");                     /* [14:18] preset vol  */
    n += sprintf(pay + n, "359");                      /* [18:21] litres      */
    n += sprintf(pay + n, "024311");                   /* [21:27] amount      */
    n += sprintf(pay + n, "874333");                   /* [27:33] tot litres  */
    n += sprintf(pay + n, "22182451");                 /* [33:41] tot amount  */
    n += sprintf(pay + n, "%s", idle ? "00" : "11");   /* [41:43] status      */
    n += sprintf(pay + n, "0990");                     /* [43:47] trailing    */
    if (n != 47) { fprintf(stderr, "MODEL BUG: payload %d chars, expected 47\n", n); exit(2); }

    int i = 0;
    out[i++] = 0x02;
    memcpy(out + i, pay, 47); i += 47;
    out[i++] = 0x03;
    ck(out, i, out + i); i += 2;
    *out_len = i;
}

int uart_param_config(int port, const uart_config_t *cfg) { (void)port; (void)cfg; return 0; }

static uint32_t rng_s = 0x5EED1234;
static uint32_t rnd(void) {
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 17; rng_s ^= rng_s << 5;
    return rng_s;
}

int uart_write_bytes(int port, const void *src, size_t size) {
    (void)port;
    const uint8_t *f = (const uint8_t *)src;
    rx_reset();
    memcpy(P.last_tx, f, size < sizeof(P.last_tx) ? size : sizeof(P.last_tx));
    P.last_tx_len = (int)size;

    if (P.fuzz) {
        /* Whatever the command was, answer with garbage of a random length —
         * including lengths beyond the driver's own frame expectations. */
        uint8_t junk[YN_RESP_MAX * 2];
        int n = (int)(rnd() % sizeof(junk));
        for (int i = 0; i < n; i++) {
            uint32_t r = rnd() % 100;
            junk[i] = (r < 12) ? 0x02 : (r < 24) ? 0x03 : (r < 30) ? 0x06
                    : (r < 34) ? 0x15 : (uint8_t)(rnd() & 0xFF);
        }
        rx_push(junk, n);
        return (int)size;
    }

    if (size < 8 || f[6] != 'A') return (int)size;
    uint8_t head = (uint8_t)(f[4] - '0');
    uint8_t side = (f[5] >= '5') ? 1 : 0;
    P.a_frames++;
    if (!P.present) return (int)size;                  /* silence */
    uint8_t rep[64];
    int     rlen;
    build_read_reply(head, P.wrong_echo ? (uint8_t)!side : side, P.price, P.idle, rep, &rlen);
    rx_push(rep, rlen);
    return (int)size;
}

int uart_read_bytes(int port, void *buf, uint32_t length, uint32_t ticks) {
    (void)port;
    int avail = rx_len - rx_pos;
    if (avail <= 0) {
        sim_clock_advance_ms((int64_t)ticks);   /* the wait actually costs time */
        return 0;
    }
    int n = (int)length < avail ? (int)length : avail;
    memcpy(buf, rx + rx_pos, (size_t)n);
    rx_pos += n;
    return n;
}

/* ── test scaffolding ──────────────────────────────────────────────────── */
static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do {                                        \
        checks++;                                                          \
        if (!(cond)) { fails++;                                            \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__); }                  \
    } while (0)

static void pump_reset(bool present, bool idle, uint16_t price) {
    memset(&P, 0, sizeof(P));
    P.present = present; P.idle = idle; P.price = price;
    rx_reset();
}

static const char *hex(const uint8_t *b, int n) {
    static char s[256];
    int pos = 0;
    for (int i = 0; i < n && pos + 4 < (int)sizeof(s); i++)
        pos += sprintf(s + pos, "%s%02x", i ? "," : "", b[i]);
    s[pos] = 0;
    return s;
}

int main(void) {
    printf("== yenen poll path — host verification against the real driver ==\n");
    YenenNozzle_t nz;
    char payload[DU_PAYLOAD_MAX];

    /* 1. Command 2 on the wire = the vendor doc's frames, byte for byte. */
    printf("[1] Command 2 frames vs the vendor doc\n");
    struct { uint8_t h, s; const char *want; } vec[] = {
        {1, 0, "02,30,38,30,31,30,41,03,37,39"},
        {1, 1, "02,30,38,30,31,35,41,03,37,3c"},
        {2, 0, "02,30,38,30,32,31,41,03,37,3b"},
        {2, 1, "02,30,38,30,32,36,41,03,37,3c"},
    };
    for (unsigned i = 0; i < sizeof(vec) / sizeof(vec[0]); i++) {
        pump_reset(true, true, 6772);
        yn_poll_nozzle(vec[i].h, vec[i].s, &nz);
        const char *got = hex(P.last_tx, P.last_tx_len);
        CHECK(strcmp(got, vec[i].want) == 0, "head%u/%c sent %s (want %s)",
              vec[i].h, vec[i].s ? 'B' : 'A', got, vec[i].want);
    }

    /* 2. Healthy read: the pump's reply, whole and unchanged, in one attempt. */
    printf("[2] healthy read\n");
    {
        pump_reset(true, true, 6772);
        yn_poll_nozzle(1, 0, &nz);
        uint8_t want[64]; int wlen;
        build_read_reply(1, 0, 6772, true, want, &wlen);
        CHECK(nz.present && nz.total_len == wlen && memcmp(nz.total, want, (size_t)wlen) == 0,
              "present=%d len=%d (want %d)", nz.present, nz.total_len, wlen);
        CHECK(P.a_frames == 1, "%d reads for a healthy pump, want 1", P.a_frames);

        yn_build_nozzle_payload(&nz, 1, "86071008133255603", "2026-09-21 12:55:16", payload, sizeof(payload));
        char expect[DU_PAYLOAD_MAX];
        snprintf(expect, sizeof(expect),
                 "{\"HW_ID\":\"86071008133255603\",\"nozzles\":{\"1\":{\"data\":\"%s\"}},\"TS\":\"2026-09-21 12:55:16\"}",
                 hex(want, wlen));
        CHECK(strcmp(payload, expect) == 0, "payload: %s", payload);
    }

    /* 3. A dispensing nozzle is still read: the cloud classifies by the status
     *    field inside the frame (yenen.c's no-gate note). */
    printf("[3] dispensing nozzle\n");
    {
        pump_reset(true, false, 6772);
        yn_poll_nozzle(2, 1, &nz);
        CHECK(nz.present && nz.total_len == 51, "dispensing nozzle: present=%d len=%d", nz.present, nz.total_len);
    }

    /* 4. Silent nozzle: YN_TOTAL_RETRIES attempts, then "404". */
    printf("[4] silent nozzle\n");
    {
        pump_reset(false, true, 6772);
        g_wdt_feeds = 0;
        int64_t t0 = esp_timer_get_time();
        yn_poll_nozzle(1, 0, &nz);
        int64_t held_ms = (esp_timer_get_time() - t0) / 1000;
        CHECK(!nz.present && nz.total_len == 0, "silent: present=%d len=%d", nz.present, nz.total_len);
        CHECK(P.a_frames == YN_TOTAL_RETRIES, "%d attempts, want %d", P.a_frames, YN_TOTAL_RETRIES);
        CHECK(g_wdt_feeds > 100, "only %d WDT feeds while waiting on a silent pump", g_wdt_feeds);
        printf("       bus held %lld ms on a silent nozzle\n", (long long)held_ms);
        yn_build_nozzle_payload(&nz, 3, "86071008133255603", "2026-09-21 12:55:16", payload, sizeof(payload));
        CHECK(strstr(payload, "\"3\":{\"data\":\"404\"}") != NULL, "silent payload: %s", payload);
    }

    /* 5. The field defect: a pump answering for the other side is dropped, not
     *    published as this nozzle's totalizer. */
    printf("[5] wrong address echo\n");
    {
        pump_reset(true, true, 6772);
        P.wrong_echo = true;
        yn_poll_nozzle(1, 0, &nz);
        CHECK(!nz.present, "a reply for the other nozzle was accepted");
    }

    /* 6. Addresses the protocol cannot express never reach the bus. */
    printf("[6] address guards\n");
    {
        struct { uint8_t h, s; } bad[] = {{0, 0}, {DU_YENEN_MAX + 1, 0}, {1, 2}};
        for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
            pump_reset(true, true, 6772);
            yn_poll_nozzle(bad[i].h, bad[i].s, &nz);
            CHECK(!nz.present && P.a_frames == 0 && P.last_tx_len == 0,
                  "head%u side%u reached the bus", bad[i].h, bad[i].s);
        }
    }

    /* 7. A noisy bus: random bytes answering every command. Nothing here can be
     *    "correct" — the requirement is that the driver never reads out of
     *    bounds and always produces a publishable payload. */
    printf("[7] noisy bus (random replies)\n");
    {
        int rounds = 20000, accepted = 0;
        for (int it = 0; it < rounds; it++) {
            pump_reset(true, true, 6772);
            P.fuzz = 1;
            uint8_t head = (uint8_t)(1 + (rnd() % DU_YENEN_MAX));
            uint8_t side = (uint8_t)(rnd() % 2);
            yn_poll_nozzle(head, side, &nz);
            if (nz.present) accepted++;
            CHECK(nz.total_len >= 0 && nz.total_len <= YN_RESP_MAX,
                  "total_len %d escaped the buffer", nz.total_len);
            yn_build_nozzle_payload(&nz, 1 + (int)(rnd() % 12), "860987051512728",
                                    "2026-09-02 11:04:19", payload, sizeof(payload));
            CHECK(strlen(payload) < sizeof(payload), "payload not terminated in buffer");
            CHECK(payload[0] == '{' && payload[strlen(payload) - 1] == '}',
                  "payload is not a JSON object");
        }
        printf("       %d rounds, %d noise frames passed the frame checks\n", rounds, accepted);
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
