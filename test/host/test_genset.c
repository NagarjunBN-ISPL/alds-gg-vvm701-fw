/* Host harness for the REAL src/genset.c — compiled whole against a simulated
 * DG controller that answers with the frames captured in the DG integration
 * spec (Mahindra GC1211 / GC1111, Ashok Leyland "DG 1202/03/04", SGC 120 Mk II).
 *
 * There is no DG in the lab, so this is the protocol test: the firmware must
 * send exactly the spec's request frames (bytes and CRC), in the spec's order,
 * and publish each reply as its own message under the spec's key —
 * "registers", "Alarms" (capital A, Mahindra only) and "status". The failure
 * cases cover a controller that is powered down, answers one read with an
 * exception, cross-talks (a radar's valid-CRC reply) or sends the wrong size. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "genset.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/task.h"

int g_log_quiet = 1;
int g_wdt_feeds = 0;

/* ── virtual clock ─────────────────────────────────────────────────────── */
static int64_t s_now_us = 1000000;
int64_t esp_timer_get_time(void) { return s_now_us; }
void sim_clock_advance_ms(int64_t ms) { s_now_us += ms * 1000; }
void vTaskDelay(TickType_t ticks) { sim_clock_advance_ms((int64_t)ticks); }

esp_err_t esp_task_wdt_status(void *h) { (void)h; return ESP_OK; }
esp_err_t esp_task_wdt_reset(void) { g_wdt_feeds++; return ESP_OK; }

/* ── the spec's frames, verbatim ──────────────────────────────────────────
 * Request and reply per command, in the spec's order. Every one CRC-checks. */
typedef struct { const char *req; const char *rsp; } Exchange_t;

static const Exchange_t k_gc1211[] = {
    { "0A 04 00 01 00 24 A0 AA",                              /* 1: registers */
      "0A 04 48 01 3F 00 3E 00 22 00 89 00 89 05 48 07 39 05 DC 01 F3 09 69 09 "
      "5C 09 52 10 51 10 3D 10 48 00 00 00 00 00 04 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 04 00 01 00 "
      "00 1B 08 5E 2C" },
    { "0A 04 00 39 00 03 61 7D", "0A 04 06 11 10 11 11 11 11 48 44" },  /* 2: Alarms */
    { "0A 04 00 43 00 02 81 64", "0A 04 04 C0 00 00 00 7D 44" },        /* 3: status */
};

static const Exchange_t k_gc1111[] = {
    { "0A 03 00 01 00 24 15 6A",
      "0A 03 48 01 3F 00 3E 00 22 00 89 00 89 05 48 07 39 05 DC 01 F3 09 69 09 "
      "5C 09 52 10 51 10 3D 10 48 00 00 00 00 00 04 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 04 00 01 00 "
      "00 1B 08 E8 5E" },
    { "0A 03 00 39 00 03 D4 BD", "0A 03 06 11 10 11 11 11 11 09 A2" },
    { "0A 03 00 43 00 02 34 A4", "0A 03 04 C0 00 00 00 7C F3" },
};

static const Exchange_t k_ashok[] = {
    { "0A 04 00 01 00 24 A0 AA",
      "0A 04 48 01 E0 00 59 00 28 00 89 00 8B 14 FB 0E 80 05 DD 01 F3 09 68 09 "
      "65 09 67 10 43 10 49 10 4E 01 31 01 20 01 1D 00 3C 00 37 00 34 00 A7 00 "
      "49 00 45 00 44 00 D3 00 29 00 29 00 2C 00 80 00 52 00 4F 00 4B 00 4F 00 "
      "09 6D 3A 1C 46" },
    { "0A 04 00 43 00 01 C1 65", "0A 04 02 40 00 2D 31" },
};

static const Exchange_t k_sgc120[] = {
    { "0A 03 00 01 00 4E 95 45",
      "0A 03 9C 09 61 09 59 08 EE 10 34 0F D8 0F DB 01 F8 01 F8 01 F8 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 06 00 00 00 0A 00 00 00 08 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 31 01 90 01 18 00 1C 00 92 00 91 05 E7 00 2D 00 "
      "22 00 00 00 01 00 03 00 00 00 00 11 00 11 11 00 11 11 01 01 00 11 00 00 "
      "00 01 10 00 00 11 11 11 00 00 F1 00 01 23 10 24 7E" },
};

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))

static int parse_hex(const char *s, uint8_t *out, int max) {
    int n = 0;
    while (*s && n < max) {
        while (*s == ' ') s++;
        if (!*s) break;
        unsigned v;
        if (sscanf(s, "%2x", &v) != 1) break;
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

static uint16_t crc16(const uint8_t *b, int n) {
    uint16_t c = 0xFFFF;
    for (int i = 0; i < n; i++) {
        c ^= b[i];
        for (int j = 0; j < 8; j++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
    }
    return c;
}

/* ── simulated DG controller ──────────────────────────────────────────── */
typedef enum { B_SILENT, B_EXCEPTION, B_WRONG_SLAVE, B_WRONG_COUNT } Odd_t;

static struct {
    const Exchange_t *ex;
    int      n_ex;
    int      odd_idx;       /* exchange answered with `odd` instead; -1 none  */
    Odd_t    odd;
    bool     dead;          /* powered down: nothing answers                  */
    uint8_t  req[8][16];    /* every request the firmware sent, in order      */
    int      req_len[8];
    int      n_req;
    int      unknown;       /* requests matching no spec frame                */
} DG;

static uint8_t rx[300];
static int rx_len, rx_pos;

static void dg_load(const Exchange_t *ex, int n) {
    memset(&DG, 0, sizeof(DG));
    DG.ex = ex; DG.n_ex = n; DG.odd_idx = -1;
    rx_len = rx_pos = 0;
}

int uart_write_bytes(uart_port_t port, const void *src, size_t size) {
    (void)port;
    const uint8_t *f = (const uint8_t *)src;
    rx_len = rx_pos = 0;
    if (DG.n_req < 8) {
        int n = size < 16 ? (int)size : 16;
        memcpy(DG.req[DG.n_req], f, (size_t)n);
        DG.req_len[DG.n_req] = n;
    }
    DG.n_req++;

    int hit = -1;
    for (int i = 0; i < DG.n_ex && hit < 0; i++) {
        uint8_t want[16];
        int wn = parse_hex(DG.ex[i].req, want, (int)sizeof(want));
        if (wn == (int)size && memcmp(want, f, size) == 0) hit = i;
    }
    if (hit < 0) { DG.unknown++; return (int)size; }   /* a real one stays silent */
    if (DG.dead) return (int)size;

    uint8_t r[300];
    int rn = parse_hex(DG.ex[hit].rsp, r, (int)sizeof(r));
    if (hit == DG.odd_idx) {
        switch (DG.odd) {
            case B_SILENT:      return (int)size;
            case B_EXCEPTION:   r[1] |= 0x80; r[2] = 0x02; rn = 3; break;  /* illegal address */
            case B_WRONG_SLAVE: r[0] = 0x04; rn -= 2;              break;  /* a radar's reply */
            case B_WRONG_COUNT: r[2] -= 2;   rn -= 4;              break;  /* one register short */
        }
        uint16_t c = crc16(r, rn);                      /* every odd reply CRC-checks */
        r[rn++] = (uint8_t)(c & 0xFF);
        r[rn++] = (uint8_t)(c >> 8);
    }
    memcpy(rx, r, (size_t)rn);
    rx_len = rn;
    return (int)size;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length, uint32_t ticks) {
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

static const char *HWID = "86071008665145510";
static const char *TS   = "2026-08-05 18:29:41";

static const char *frame_str(const uint8_t *b, int n) {
    static char s[64];
    int p = 0;
    for (int i = 0; i < n && p < (int)sizeof(s) - 4; i++)
        p += snprintf(s + p, sizeof(s) - (size_t)p, i ? " %02X" : "%02X", b[i]);
    return s;
}

/* The data bytes of a spec reply as the firmware must publish them. */
static void reply_hex(const char *rsp, char *out, size_t out_size) {
    uint8_t r[300];
    parse_hex(rsp, r, (int)sizeof(r));
    int p = 0;
    for (int i = 0; i < r[2] && p < (int)out_size - 4; i++)
        p += snprintf(out + p, out_size - (size_t)p, i ? ",%02x" : "%02x", r[3 + i]);
}

static void expect_msg(const char *what, const char *got, const char *key, const char *hex) {
    char want[1400];
    snprintf(want, sizeof(want), "{\"HW_ID\":\"%s\",\"genset\":{\"%s\":\"%s\"},\"TS\":\"%s\"}",
             HWID, key, hex, TS);
    CHECK(strcmp(got, want) == 0, "%s %s message differs:\n     got  %s\n     want %s",
          what, key, got, want);
}

static void expect_frames(const char *what, const Exchange_t *ex, int n) {
    CHECK(DG.n_req == n, "%s: sent %d frame(s), the spec has %d", what, DG.n_req, n);
    CHECK(DG.unknown == 0, "%s: %d frame(s) not in the spec", what, DG.unknown);
    for (int i = 0; i < n && i < DG.n_req && i < 8; i++) {
        uint8_t want[16];
        int wn = parse_hex(ex[i].req, want, (int)sizeof(want));
        CHECK(DG.req_len[i] == wn && memcmp(DG.req[i], want, (size_t)wn) == 0,
              "%s: command %d is %s, the spec says %s", what, i + 1,
              frame_str(DG.req[i], DG.req_len[i]), ex[i].req);
    }
}

int main(void) {
    GensetData_t gd;
    static char buf[2048], hex[600];

    printf("== every model sends exactly the spec's commands and publishes its replies ==\n");
    struct {
        const char *name; uint8_t model; const Exchange_t *ex; int n; bool alarms; bool status;
    } cases[] = {
        { "GC1211",  GENSET_MODEL_GC1211, k_gc1211, N(k_gc1211), true,  true  },
        { "GC1111",  GENSET_MODEL_GC1111, k_gc1111, N(k_gc1111), true,  true  },
        { "GC1201",  GENSET_MODEL_GC1201, k_ashok,  N(k_ashok),  false, true  },
        { "GC1202",  GENSET_MODEL_GC1202, k_ashok,  N(k_ashok),  false, true  },
        { "GC1203",  GENSET_MODEL_GC1203, k_ashok,  N(k_ashok),  false, true  },
        { "GC902",   GENSET_MODEL_GC902,  k_ashok,  N(k_ashok),  false, true  },  /* not in the spec: Ashok read set */
        { "SGC120",  GENSET_MODEL_SGC120, k_sgc120, N(k_sgc120), false, false },
    };
    for (int c = 0; c < N(cases); c++) {
        dg_load(cases[c].ex, cases[c].n);
        bool ok = genset_poll(10, cases[c].model, &gd);
        CHECK(ok, "%s: poll failed against the spec's own replies", cases[c].name);
        expect_frames(cases[c].name, cases[c].ex, cases[c].n);
        CHECK(gd.has_alarms == cases[c].alarms && gd.has_status == cases[c].status,
              "%s: has_alarms %d has_status %d", cases[c].name, gd.has_alarms, gd.has_status);

        genset_build_registers_payload(&gd, HWID, TS, buf, sizeof(buf));
        reply_hex(cases[c].ex[0].rsp, hex, sizeof(hex));
        expect_msg(cases[c].name, buf, "registers", hex);
        CHECK(strlen(buf) < MQTT_MODEM_PAYLOAD_SAFE_MAX, "%s: registers message %zu bytes", cases[c].name, strlen(buf));
        if (cases[c].alarms) {
            genset_build_alarms_payload(&gd, HWID, TS, buf, sizeof(buf));
            reply_hex(cases[c].ex[1].rsp, hex, sizeof(hex));
            expect_msg(cases[c].name, buf, "Alarms", hex);
        }
        if (cases[c].status) {
            genset_build_status_payload(&gd, HWID, TS, buf, sizeof(buf));
            reply_hex(cases[c].ex[cases[c].n - 1].rsp, hex, sizeof(hex));
            expect_msg(cases[c].name, buf, "status", hex);
        }
        printf("       %-7s %d command(s) byte-exact, messages OK\n", cases[c].name, DG.n_req);
    }

    /* The GC1211 messages spelled out in full, independent of reply_hex(). */
    dg_load(k_gc1211, N(k_gc1211));
    genset_poll(10, GENSET_MODEL_GC1211, &gd);
    genset_build_registers_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("GC1211 literal", buf, "registers",
               "01,3f,00,3e,00,22,00,89,00,89,05,48,07,39,05,dc,01,f3,09,69,09,5c,09,52,"
               "10,51,10,3d,10,48,00,00,00,00,00,04,00,00,00,00,00,00,00,00,00,00,00,00,"
               "00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,04,00,01,00,00,1b,08");
    genset_build_alarms_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("GC1211 literal", buf, "Alarms", "11,10,11,11,11,11");
    printf("       %s\n", buf);
    genset_build_status_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("GC1211 literal", buf, "status", "c0,00,00,00");

    printf("== Alarms unanswered: the poll fails, status is still read ==\n");
    dg_load(k_gc1211, N(k_gc1211));
    DG.odd_idx = 1; DG.odd = B_SILENT;
    bool ok = genset_poll(10, GENSET_MODEL_GC1211, &gd);
    CHECK(!ok, "a missing Alarms reply must fail the poll, so the 60 s re-poll runs");
    CHECK(gd.reg_valid && !gd.al_valid && gd.st_valid, "only Alarms should be missing (%d %d %d)",
          gd.reg_valid, gd.al_valid, gd.st_valid);
    CHECK(DG.n_req == 3, "status must still be read after a failed Alarms read (%d sent)", DG.n_req);
    genset_build_alarms_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("silent Alarms", buf, "Alarms", "404");
    genset_build_status_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("silent Alarms", buf, "status", "c0,00,00,00");

    printf("== DG powered down: three 404 messages, bounded bus hold, WDT fed ==\n");
    dg_load(k_gc1211, N(k_gc1211));
    DG.dead = true; g_wdt_feeds = 0;
    int64_t t0 = esp_timer_get_time();
    ok = genset_poll(10, GENSET_MODEL_GC1211, &gd);
    int64_t held_ms = (esp_timer_get_time() - t0) / 1000;
    CHECK(!ok && !gd.reg_valid && !gd.al_valid && !gd.st_valid, "a dead DG must read nothing");
    CHECK(gd.has_alarms && gd.has_status, "the model still promises Alarms and status (404s)");
    CHECK(held_ms <= 1700, "one attempt held the bus %lld ms (budget ~1.6 s, config.h)", (long long)held_ms);
    CHECK(g_wdt_feeds > 0, "the watchdog was not fed while waiting");
    genset_build_registers_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("dead DG", buf, "registers", "404");
    genset_build_alarms_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("dead DG", buf, "Alarms", "404");
    genset_build_status_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("dead DG", buf, "status", "404");
    printf("       dead Mahindra: one attempt holds the bus %lld ms\n", (long long)held_ms);

    printf("== odd replies are rejected, never published as data ==\n");
    dg_load(k_gc1211, N(k_gc1211));
    DG.odd_idx = 1; DG.odd = B_EXCEPTION;
    t0 = esp_timer_get_time();
    ok = genset_poll(10, GENSET_MODEL_GC1211, &gd);
    held_ms = (esp_timer_get_time() - t0) / 1000;
    CHECK(!ok && gd.reg_valid && !gd.al_valid && gd.st_valid, "exception on Alarms: only Alarms lost");
    CHECK(held_ms < 400, "an exception must end the read at once, not wait out the timeout (%lld ms)",
          (long long)held_ms);

    dg_load(k_gc1211, N(k_gc1211));
    DG.odd_idx = 2; DG.odd = B_WRONG_SLAVE;
    ok = genset_poll(10, GENSET_MODEL_GC1211, &gd);
    CHECK(!ok && gd.reg_valid && gd.al_valid && !gd.st_valid, "a valid-CRC reply from slave 4 must be refused");

    dg_load(k_gc1211, N(k_gc1211));
    DG.odd_idx = 1; DG.odd = B_WRONG_COUNT;
    ok = genset_poll(10, GENSET_MODEL_GC1211, &gd);
    CHECK(!ok && !gd.al_valid, "a valid-CRC Alarms reply one register short must be refused");
    genset_build_alarms_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("short Alarms", buf, "Alarms", "404");

    dg_load(k_gc1211, N(k_gc1211));
    ok = genset_poll(10, 99, &gd);
    CHECK(!ok && DG.n_req == 0, "an unknown model must send nothing (%d sent)", DG.n_req);

    printf("== bus never obtained: genset_expect() still promises every message ==\n");
    struct { uint8_t model; bool alarms, status; } ex[] = {
        { GENSET_MODEL_GC1211, true,  true  }, { GENSET_MODEL_GC1111, true,  true  },
        { GENSET_MODEL_SGC120, false, false }, { GENSET_MODEL_GC1201, false, true  },
        { GENSET_MODEL_GC902,  false, true  }, { 99,                  false, false },
    };
    for (int e = 0; e < N(ex); e++) {
        dg_load(k_gc1211, N(k_gc1211));
        memset(&gd, 0, sizeof(gd));
        genset_expect(ex[e].model, &gd);
        CHECK(gd.has_alarms == ex[e].alarms && gd.has_status == ex[e].status && DG.n_req == 0,
              "model %u: expect alarms %d status %d, got %d %d (%d frames)",
              ex[e].model, ex[e].alarms, ex[e].status, gd.has_alarms, gd.has_status, DG.n_req);
        CHECK(!gd.reg_valid && !gd.al_valid && !gd.st_valid, "model %u: nothing valid without a read", ex[e].model);
    }
    /* poll_genset()'s skipped cycle: cleared, then expect — three 404s for a Mahindra */
    memset(&gd, 0, sizeof(gd));
    genset_expect(GENSET_MODEL_GC1211, &gd);
    genset_build_alarms_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("skipped cycle", buf, "Alarms", "404");
    genset_build_status_payload(&gd, HWID, TS, buf, sizeof(buf));
    expect_msg("skipped cycle", buf, "status", "404");

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
