/* Host harness for the BlueSky reply check in src/dispenser.c. The file is
 * #included whole, so the static bluesky_read_valid() is tested directly, and
 * du_poll_bluesky_nozzle() / du_exec_bluesky_passthrough() are driven against
 * a pump that replays exact bytes.
 *
 * What it pins:
 *   - every real field frame passes — the checksum rule is right (a wrong rule
 *     would turn every nozzle in the fleet into "404");
 *   - each kind of damaged frame is dropped, retried, and never published;
 *   - the idle gate, the payload shape and the passthrough are unchanged. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "dispenser.c"

int g_log_quiet = 1;
int g_wdt_feeds = 0;

/* ── virtual clock + ESP-IDF stubs ─────────────────────────────────────── */
static int64_t s_now_us = 1000000;
int64_t esp_timer_get_time(void) { return s_now_us; }
void vTaskDelay(TickType_t ticks) { s_now_us += (int64_t)ticks * 1000; }
esp_err_t esp_task_wdt_status(void *h) { (void)h; return ESP_OK; }
esp_err_t esp_task_wdt_reset(void) { g_wdt_feeds++; return ESP_OK; }
int uart_param_config(uart_port_t port, const uart_config_t *cfg) { (void)port; (void)cfg; return 0; }

/* ── frames ────────────────────────────────────────────────────────────── */
typedef struct { uint8_t b[40]; int len; } Frame;

static Frame fr(const char *hex) {
    Frame f;
    memset(&f, 0, sizeof(f));
    while (*hex && f.len < (int)sizeof(f.b)) {
        while (*hex == ',' || *hex == ' ') hex++;
        if (!*hex) break;
        char *end;
        unsigned long v = strtoul(hex, &end, 16);
        if (end == hex) break;
        f.b[f.len++] = (uint8_t)v;
        hex = end;
    }
    return f;
}

/* Nozzle 2, Vinayakapuram 2026-09-23 (status without the old firmware's stray
 * leading 0x78). V_RSP is the frame the pump should have sent for 65.82. */
#define V_STATUS "f5,02,a3,80,d5,01"
#define V_TOTAL  "f5,02,ac,02,79,78,94,62,50,27,56,81,18,c5,53"
#define V_RSP    "f5,02,a4,65,82,b6,02"
#define V_KFACT  "f5,02,a4,23,65,e2,77"          /* Sentry nozzle 2 */

/* ── the pump: replays exact bytes ─────────────────────────────────────────
 * A read command answers with the next frame queued for it, or once the queue
 * is empty with its standing reply (len 0 = silent). Anything longer than a
 * 5-byte read is a write and gets write_ack. */
static Frame standing[256];
static Frame queued[256][8];
static int   nqueued[256], qpos[256], reads[256];
static Frame write_ack;

static uint8_t rx[64];
static int     rx_len, rx_pos;

static void queue(uint8_t cmd, const char *hex, int times) {
    while (times-- > 0 && nqueued[cmd] < 8) queued[cmd][nqueued[cmd]++] = fr(hex);
}

static void pump_reset(void) {
    memset(queued, 0, sizeof(queued));
    memset(nqueued, 0, sizeof(nqueued));
    memset(qpos, 0, sizeof(qpos));
    memset(reads, 0, sizeof(reads));
    standing[BS_CMD_STATUS] = fr(V_STATUS);
    standing[BS_CMD_TOTAL]  = fr(V_TOTAL);
    standing[BS_CMD_RSP]    = fr(V_RSP);
    standing[BS_CMD_KFACT]  = fr(V_KFACT);
    write_ack = fr("f5,02,a2,b7,00");
}

int uart_write_bytes(uart_port_t port, const void *src, size_t size) {
    (void)port;
    const uint8_t *f = (const uint8_t *)src;
    const Frame *r = NULL;
    rx_len = rx_pos = 0;
    if (size == 5 && f[0] == 0xF5) {
        uint8_t cmd = f[3];
        reads[cmd]++;
        r = (qpos[cmd] < nqueued[cmd]) ? &queued[cmd][qpos[cmd]++] : &standing[cmd];
    } else if (size > 5 && f[0] == 0xF5) {
        r = &write_ack;
    }
    if (r && r->len) { memcpy(rx, r->b, (size_t)r->len); rx_len = r->len; }
    return (int)size;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length, uint32_t ticks) {
    (void)port;
    int avail = rx_len - rx_pos;
    if (avail <= 0) { s_now_us += (int64_t)ticks * 1000; return 0; }
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

static bool valid(const char *hex, uint8_t addr, uint8_t cmd) {
    Frame f = fr(hex);
    return bluesky_read_valid(f.b, f.len, addr, cmd, "test");
}

static const char *hexof(const uint8_t *b, int len) {
    static char s[4][128];
    static int k;
    k = (k + 1) & 3;
    if (len <= 0) return "404";
    du_bytes_to_hex(b, len, s[k], sizeof(s[k]));
    return s[k];
}

static DuNozzle_t poll2(void) {
    DuNozzle_t nz;
    du_poll_bluesky_nozzle(2, &nz, true);
    return nz;
}

int main(void) {
    /* [1] Every real field frame passes, as the command and nozzle it answers. */
    static const struct { const char *hex; uint8_t addr, cmd; const char *what; } FIELD[] = {
        { V_STATUS, 2, 0xD5, "Vinayakapuram status" },
        { V_TOTAL,  2, 0xC5, "Vinayakapuram totalizer" },
        { "f5,02,a4,00,65,b6,00", 2, 0xB6, "Vinayakapuram RSP 00.65 (well-formed, wrong value)" },
        { "f5,02,a3,4e,d7,4d",    2, 0xD7, "Vinayakapuram RFID" },
        { "f5,01,ac,00,02,15,18,26,23,30,19,35,34,c5,3f", 1, 0xC5, "Sentry totalizer nz1" },
        { "f5,01,a4,68,86,b6,08", 1, 0xB6, "Sentry RSP nz1" },
        { "f5,01,a4,23,60,e2,71", 1, 0xE2, "Sentry k-factor nz1" },
        { "f5,02,ac,04,30,83,81,09,40,83,66,07,33,c5,30", 2, 0xC5, "Sentry totalizer nz2" },
        { V_KFACT,  2, 0xE2, "Sentry k-factor nz2" },
    };
    const int NF = (int)(sizeof(FIELD) / sizeof(FIELD[0]));
    for (int i = 0; i < NF; i++)
        CHECK(valid(FIELD[i].hex, FIELD[i].addr, FIELD[i].cmd), "[1] field frame dropped: %s", FIELD[i].what);
    CHECK(valid(V_RSP, 2, 0xB6), "[1] the RSP 65.82 frame is dropped");

    /* [2] Every single-bit flip is dropped, except bit 7 of a data byte: the
     *     checksum clears bit 7, so it cannot see those (the BCD check still
     *     catches some of them in totalizer/RSP data). */
    int flips = 0, missed = 0;
    for (int i = 0; i < NF; i++) {
        Frame f = fr(FIELD[i].hex);
        for (int at = 0; at < f.len; at++) {
            for (int bit = 0; bit < 8; bit++) {
                if (bit == 7 && at >= 3 && at < f.len - 2) continue;
                Frame g = f;
                g.b[at] ^= (uint8_t)(1u << bit);
                flips++;
                if (bluesky_read_valid(g.b, g.len, FIELD[i].addr, FIELD[i].cmd, "flip")) {
                    missed++;
                    printf("  FAIL  [2] %s: flip of byte %d bit %d passed\n", FIELD[i].what, at, bit);
                }
            }
        }
    }
    CHECK(missed == 0, "[2] %d of %d single-bit flips passed", missed, flips);

    /* [3] Each damaged shape is dropped, with the reason it exists for. */
    CHECK(!valid("f5,02,ac,02,79,78,94,62,10,27,56,81,18,c5,53", 2, 0xC5), "[3] amount 5->1 bit flip (the 15:44 fault) passed");
    CHECK(!valid("f5,02,ac,02,79,78,94,62,70,27,56,81,18,c5,53", 2, 0xC5), "[3] amount 5->7 bit flip (the 16:08 fault) passed");
    CHECK(!valid("f5,02,ac,02,79,78,94,62,50,27,56,81,18,c5", 2, 0xC5),    "[3] totalizer cut short passed");
    CHECK(!valid("f5,02,a4,65", 2, 0xB6),                                  "[3] RSP cut to 4 bytes passed");
    CHECK(!valid(V_STATUS, 2, 0xB6),                                       "[3] a status frame accepted as the RSP reply");
    CHECK(!valid(V_RSP, 2, 0xC5),                                          "[3] an RSP frame accepted as the totalizer reply");
    CHECK(!valid("f5,01,a4,68,86,b6,08", 2, 0xB6),                         "[3] nozzle 1's reply accepted for nozzle 2");
    CHECK(!valid("f5,02,a2,b6,63", 2, 0xB6),                               "[3] our own command echoed back passed");
    CHECK(!valid("78,f5,02,a3,80,d5,01", 2, 0xD5),                         "[3] stray byte before F5 passed");
    {
        uint8_t nb[] = {0xF5, 0x02, 0xA4, 0x6A, 0x82, 0xB6, 0};      /* checksum right, digit 'a' */
        nb[6] = bluesky_chk(nb, 6);
        CHECK(!bluesky_read_valid(nb, 7, 2, 0xB6, "test"), "[3] non-BCD RSP with a good checksum passed");
        uint8_t kb[] = {0xF5, 0x02, 0xA4, 0x6A, 0x82, 0xE2, 0};      /* same data as a k-factor */
        kb[6] = bluesky_chk(kb, 6);
        CHECK(bluesky_read_valid(kb, 7, 2, 0xE2, "test"), "[3] the digit check must not apply to the k-factor");
        uint8_t ln[] = {0xF5, 0x02, 0xA5, 0x65, 0x82, 0xB6, 0};      /* checksum right, says 5 follow */
        ln[6] = bluesky_chk(ln, 6);
        CHECK(!bluesky_read_valid(ln, 7, 2, 0xB6, "test"), "[3] wrong length byte with a good checksum passed");
    }

    /* [4] End to end: a clean round publishes the pump's bytes unchanged, in
     *     the field payload shape. */
    pump_reset();
    DuNozzle_t nz = poll2();
    CHECK(nz.present && nz.idle && !nz.busy, "[4] clean round: present=%d idle=%d busy=%d", nz.present, nz.idle, nz.busy);
    CHECK(strcmp(hexof(nz.total, nz.total_len), V_TOTAL) == 0, "[4] totalizer: %s", hexof(nz.total, nz.total_len));
    CHECK(strcmp(hexof(nz.rsp, nz.rsp_len), V_RSP) == 0,       "[4] rsp: %s", hexof(nz.rsp, nz.rsp_len));
    CHECK(strcmp(hexof(nz.kfact, nz.kfact_len), V_KFACT) == 0, "[4] kfact: %s", hexof(nz.kfact, nz.kfact_len));
    CHECK(reads[0xD5] == 1 && reads[0xC5] == 1 && reads[0xB6] == 1 && reads[0xE2] == 1,
          "[4] one read each expected: D5=%d C5=%d B6=%d E2=%d", reads[0xD5], reads[0xC5], reads[0xB6], reads[0xE2]);
    {
        char buf[600];
        du_build_nozzle_payload(&nz, 3, "35056817171133801", "2026-09-23 15:51:45", buf, sizeof(buf));
        CHECK(strcmp(buf, "{\"HW_ID\":\"35056817171133801\",\"nozzles\":{\"3\":{\"data\":\"" V_TOTAL
                          "\",\"rsp\":\"" V_RSP "\",\"kfact\":\"" V_KFACT "\",\"rfid\":\"404\"}},"
                          "\"TS\":\"2026-09-23 15:51:45\"}") == 0, "[4] payload: %s", buf);
    }

    /* [5] A damaged reply is retried, and the good retry is what is published. */
    pump_reset();
    queue(0xC5, "f5,02,ac,02,79,78,94,62,10,27,56,81,18,c5,53", 1);
    queue(0xB6, V_STATUS, 1);                               /* a late reply to another read */
    nz = poll2();
    CHECK(strcmp(hexof(nz.total, nz.total_len), V_TOTAL) == 0 && reads[0xC5] == 2,
          "[5] totalizer after one bad reply: %s, %d reads", hexof(nz.total, nz.total_len), reads[0xC5]);
    CHECK(strcmp(hexof(nz.rsp, nz.rsp_len), V_RSP) == 0 && reads[0xB6] == 2,
          "[5] rsp after one bad reply: %s, %d reads", hexof(nz.rsp, nz.rsp_len), reads[0xB6]);

    /* [6] Damaged on every attempt: "404" for that field, never the bytes. */
    pump_reset();
    queue(0xC5, "f5,02,ac,02,79,78,94,62,50,27,56,81,18,c5", DU_TOTAL_RETRIES);
    queue(0xB6, "f5,02,a4,65", DU_RSP_RETRIES);
    nz = poll2();
    CHECK(nz.total_len == 0 && reads[0xC5] == DU_TOTAL_RETRIES, "[6] totalizer: len %d, %d reads", nz.total_len, reads[0xC5]);
    CHECK(nz.rsp_len == 0 && reads[0xB6] == DU_RSP_RETRIES,     "[6] rsp: len %d, %d reads", nz.rsp_len, reads[0xB6]);
    {
        char buf[600];
        du_build_nozzle_payload(&nz, 3, "35056817171133801", "2026-09-23 15:51:45", buf, sizeof(buf));
        CHECK(strstr(buf, "\"data\":\"404\",\"rsp\":\"404\",\"kfact\":\"" V_KFACT "\"") != NULL, "[6] payload: %s", buf);
    }

    /* [7] Idle gate. A corrupted status can never open it: a busy 0x81 frame
     *     with one bit flipped to read 0x80 is dropped, and so is our own
     *     command echoed back. The documented 0x00 idle still reads. */
    pump_reset();
    queue(0xD5, "f5,02,a3,80,d5,00", DU_STATUS_RETRIES);   /* busy f5,02,a3,81,d5,00, bit 0 flipped */
    nz = poll2();
    CHECK(!nz.present && !nz.idle && reads[0xC5] == 0, "[7] flipped busy status: present=%d idle=%d C5 reads=%d",
          nz.present, nz.idle, reads[0xC5]);
    pump_reset();
    queue(0xD5, "f5,02,a3,81,d5,00", DU_STATUS_RETRIES);   /* intact busy */
    nz = poll2();
    CHECK(nz.present && nz.busy && reads[0xC5] == 0, "[7] busy status: present=%d busy=%d C5 reads=%d",
          nz.present, nz.busy, reads[0xC5]);
    pump_reset();
    queue(0xD5, "f5,02,a2,d5,00", DU_STATUS_RETRIES);      /* our status command, echoed */
    nz = poll2();
    CHECK(!nz.present && reads[0xC5] == 0, "[7] echoed command: present=%d C5 reads=%d", nz.present, reads[0xC5]);
    pump_reset();
    standing[BS_CMD_STATUS] = fr("f5,02,a3,00,d5,01");     /* the 0x00 idle */
    nz = poll2();
    CHECK(nz.present && nz.idle && nz.total_len == 15, "[7] 0x00 idle: present=%d idle=%d total_len=%d",
          nz.present, nz.idle, nz.total_len);

    /* [8] Passthrough: the write's own reply is still checked for header and
     *     address only (its layout is unknown); the read-back is checked in full. */
    {
        char resp[64], rb[64], reason[32], hex[40];
        uint8_t w[] = {0xF5, 0x02, 0xA4, 0xB7, 0x65, 0x82, 0};
        w[6] = bluesky_chk(w, 6);
        du_bytes_to_hex(w, 7, hex, sizeof(hex));

        pump_reset();
        bool ok = du_exec_bluesky_passthrough(hex, 1, 2, BS_CMD_RSP, resp, sizeof(resp), rb, sizeof(rb),
                                              reason, sizeof(reason));
        CHECK(ok && strcmp(resp, "f5,02,a2,b7,00") == 0 && strcmp(rb, V_RSP) == 0,
              "[8] rsp write: ok=%d resp=%s rb=%s", ok, resp, rb);

        pump_reset();
        queue(0xB6, "f5,02,a4,65,82,b6,03", 2);            /* read-back damaged on both attempts */
        ok = du_exec_bluesky_passthrough(hex, 1, 2, BS_CMD_RSP, resp, sizeof(resp), rb, sizeof(rb),
                                         reason, sizeof(reason));
        CHECK(ok && rb[0] == '\0', "[8] damaged read-back must be dropped: ok=%d rb=%s", ok, rb);

        pump_reset();
        queue(0xD5, "f5,02,a3,80,d5,00", DU_STATUS_RETRIES); /* damaged status: gate refuses */
        ok = du_exec_bluesky_passthrough(hex, 1, 2, BS_CMD_RSP, resp, sizeof(resp), rb, sizeof(rb),
                                         reason, sizeof(reason));
        CHECK(!ok && strcmp(reason, "nozzle_absent") == 0, "[8] damaged status: ok=%d reason=%s", ok, reason);
    }

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
