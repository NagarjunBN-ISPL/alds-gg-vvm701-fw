/* Host harness for the REAL src/dispenser.c — compiled whole against a
 * simulated BlueSky pump — covering the k-factor cache.
 *
 * The k-factor is read every KFACTOR_POLL_INTERVAL_S and carried forward in
 * between. The bug this pins: a k-factor WRITE through the cloud passthrough
 * changed the pump, but every nozzle message kept publishing the OLD value
 * until the next scheduled read, up to 15 minutes later. Now the write's
 * read-back refreshes the cache, and a write whose effect is unconfirmed makes
 * the next round read the k-factor even when it is not due. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "dispenser.h"
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
int uart_param_config(uart_port_t port, const uart_config_t *cfg) { (void)port; (void)cfg; return 0; }

/* ── simulated BlueSky pump ────────────────────────────────────────────────
 * Answers the four read commands the driver uses in the real reply layout —
 * F5 <addr> A<n> <data...> <cmd> <chk>, chk = XOR of the bytes before it with
 * bit 7 cleared — since the driver drops anything else. And one k-factor WRITE
 * shape:
 *   F5 <addr> A4 E3 <k1> <k2> <cs>   → pump stores k1 k2, replies F5 <addr> A2 E3 <cs>
 * (the real write command set is the cloud's; the driver only checks the frame
 * shape and the address, so any well-formed frame exercises the same path. The
 * write reply's layout is unknown, so the driver checks only its header and
 * address — this ack, with no data and no real checksum, must keep passing). */
#define WR_KFACT 0xE3
static struct {
    uint8_t kf[16][2];         /* k-factor per address                           */
    int     drop_kf_reads;     /* next N 0xE2 reads go unanswered                */
    bool    write_silent;      /* applies a write but does not answer it         */
    int     kf_reads;          /* 0xE2 requests seen (answered or not)           */
    int     writes;            /* write frames seen                              */
} P;

static uint8_t rx[64];
static int rx_len, rx_pos;

static void reply(const uint8_t *r, int n) { memcpy(rx, r, (size_t)n); rx_len = n; rx_pos = 0; }

static void reply_read(uint8_t a, uint8_t cmd, const uint8_t *data, int n) {
    uint8_t r[32], x = 0;
    int k = 0;
    r[k++] = 0xF5; r[k++] = a; r[k++] = (uint8_t)(0xA0 | (n + 2));
    memcpy(r + k, data, (size_t)n); k += n;
    r[k++] = cmd;
    for (int i = 0; i < k; i++) x ^= r[i];
    r[k++] = (uint8_t)(x & 0x7F);
    reply(r, k);
}

int uart_write_bytes(uart_port_t port, const void *src, size_t size) {
    (void)port;
    const uint8_t *f = (const uint8_t *)src;
    rx_len = rx_pos = 0;
    if (size < 5 || f[0] != 0xF5) return (int)size;
    uint8_t a = f[1];
    if (a >= 16) return (int)size;

    if (size == 5) {
        switch (f[3]) {
            case 0xD5: { uint8_t d[] = {0x80};                                    reply_read(a, 0xD5, d, 1);  break; }
            case 0xC5: { uint8_t d[] = {0x00, 0x02, 0x15, 0x18, 0x26,
                                        0x23, 0x30, 0x19, 0x35, 0x34};            reply_read(a, 0xC5, d, 10); break; }
            case 0xB6: { uint8_t d[] = {0x68, 0x86};                              reply_read(a, 0xB6, d, 2);  break; }
            case 0xE2:
                P.kf_reads++;
                if (P.drop_kf_reads > 0) { P.drop_kf_reads--; break; }
                reply_read(a, 0xE2, P.kf[a], 2);
                break;
        }
    } else if (size == 7 && f[3] == WR_KFACT) {
        P.writes++;
        P.kf[a][0] = f[4];
        P.kf[a][1] = f[5];
        if (!P.write_silent) { uint8_t r[] = {0xF5, a, 0xA2, WR_KFACT, 0x00}; reply(r, 5); }
    } else if (size == 7) {
        P.writes++;   /* any other write (an RSP change): acknowledged, k-factor untouched */
        uint8_t r[] = {0xF5, a, 0xA2, f[3], 0x00};
        reply(r, 5);
    }
    return (int)size;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length, uint32_t ticks) {
    (void)port;
    int avail = rx_len - rx_pos;
    if (avail <= 0) { sim_clock_advance_ms((int64_t)ticks); return 0; }
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

/* One poll round for `addr`; returns the k-factor it would publish as
 * "k1k2" (hex) or "404", and how many 0xE2 reads the round spent. */
static const char *round_kf(uint8_t addr, bool due, int *reads) {
    static char s[16];
    DuNozzle_t nz;
    int before = P.kf_reads;
    du_poll_bluesky_nozzle(addr, &nz, due);
    if (reads) *reads = P.kf_reads - before;
    if (nz.kfact_len == 7) snprintf(s, sizeof(s), "%02x%02x", nz.kfact[3], nz.kfact[4]);
    else if (nz.kfact_len == 0) snprintf(s, sizeof(s), "404");
    else snprintf(s, sizeof(s), "len%d", nz.kfact_len);
    return s;
}

/* Passthrough a k-factor write to dispenser n (nozzles 2n-1 / 2n). */
static bool write_kf(int n, uint8_t addr, uint8_t k1, uint8_t k2, char *rb, char *reason) {
    char hex[40], resp[64];
    uint8_t cs = (uint8_t)(addr ^ WR_KFACT ^ k1 ^ k2);
    snprintf(hex, sizeof(hex), "F5,%02X,A4,%02X,%02X,%02X,%02X", addr, WR_KFACT, k1, k2, cs);
    reason[0] = rb[0] = '\0';
    return du_exec_bluesky_passthrough(hex, (uint8_t)(2 * n - 1), (uint8_t)(2 * n), BS_CMD_KFACT,
                                       resp, sizeof(resp), rb, 64, reason, 32);
}

int main(void) {
    char rb[64], reason[32];
    int reads;
    const char *kf;

    /* [1] Baseline: read when due, carried forward (no bus read) otherwise. */
    memset(&P, 0, sizeof(P));
    P.kf[1][0] = 0x03; P.kf[1][1] = 0xE8;
    kf = round_kf(1, true, &reads);
    CHECK(strcmp(kf, "03e8") == 0 && reads == 1, "[1] due round: got %s, %d reads", kf, reads);
    kf = round_kf(1, false, &reads);
    CHECK(strcmp(kf, "03e8") == 0 && reads == 0, "[1] non-due round: got %s, %d reads", kf, reads);
    kf = round_kf(3, false, &reads);
    CHECK(strcmp(kf, "404") == 0 && reads == 0, "[1] never-read nozzle: got %s, %d reads", kf, reads);

    /* [2] THE BUG: a write with a good read-back must show in the very next
     *     message, without waiting for the next due read. */
    bool ok = write_kf(1, 1, 0x04, 0x00, rb, reason);
    CHECK(ok, "[2] write refused: %s", reason);
    CHECK(strcmp(rb, "f5,01,a4,04,00,e2,36") == 0, "[2] read-back hex: %s", rb);
    kf = round_kf(1, false, &reads);
    CHECK(strcmp(kf, "0400") == 0, "[2] next message still carries the old k-factor: %s", kf);
    CHECK(reads == 0, "[2] read-back already confirmed it — no extra bus read expected, got %d", reads);

    /* [3] Write lands but BOTH read-back attempts go unanswered: the next
     *     round must re-read even though it is not due, then stop. */
    P.drop_kf_reads = 2;
    ok = write_kf(1, 2, 0x05, 0x00, rb, reason);
    CHECK(ok && rb[0] == '\0', "[3] write should succeed with no read-back (ok=%d rb=%s)", ok, rb);
    kf = round_kf(2, false, &reads);
    CHECK(strcmp(kf, "0500") == 0 && reads == 1, "[3] forced re-read: got %s, %d reads", kf, reads);
    kf = round_kf(2, false, &reads);
    CHECK(strcmp(kf, "0500") == 0 && reads == 0, "[3] re-read cleared the stale mark: got %s, %d reads", kf, reads);

    /* [4] Pump applies the write but never answers it (no_reply): the value
     *     may have changed, so the next round re-reads. */
    P.kf[3][0] = 0x03; P.kf[3][1] = 0xE8;
    (void)round_kf(3, true, NULL);                       /* cache 03e8 */
    P.write_silent = true;
    ok = write_kf(2, 3, 0x06, 0x00, rb, reason);
    P.write_silent = false;
    CHECK(!ok && strcmp(reason, "no_reply") == 0, "[4] expected no_reply, got ok=%d %s", ok, reason);
    kf = round_kf(3, false, &reads);
    CHECK(strcmp(kf, "0600") == 0 && reads == 1, "[4] forced re-read after no_reply: got %s, %d reads", kf, reads);

    /* [5] Stale and the forced re-read fails too: carry the last good value,
     *     stay stale, and read again next round until it succeeds. */
    P.kf[5][0] = 0x03; P.kf[5][1] = 0xE8;
    (void)round_kf(5, true, NULL);                       /* cache 03e8 */
    P.drop_kf_reads = 2 + DU_KFACTOR_RETRIES;            /* read-back and the whole next round */
    ok = write_kf(3, 5, 0x07, 0x00, rb, reason);
    CHECK(ok, "[5] write refused: %s", reason);
    kf = round_kf(5, false, &reads);
    CHECK(strcmp(kf, "03e8") == 0 && reads == DU_KFACTOR_RETRIES,
          "[5] failed forced re-read carries last good: got %s, %d reads", kf, reads);
    kf = round_kf(5, false, &reads);
    CHECK(strcmp(kf, "0700") == 0 && reads == 1, "[5] still stale, re-read next round: got %s, %d reads", kf, reads);

    /* [6] A refused write (wrong dispenser) never reached the pump — the
     *     cache stays trusted, no extra read. */
    ok = write_kf(1, 5, 0x09, 0x00, rb, reason);       /* nozzle 5 belongs to dispenser 3 */
    CHECK(!ok && strcmp(reason, "addr_not_this_dispenser") == 0, "[6] expected refusal, got ok=%d %s", ok, reason);
    kf = round_kf(5, false, &reads);
    CHECK(strcmp(kf, "0700") == 0 && reads == 0, "[6] refused write left cache alone: got %s, %d reads", kf, reads);

    /* [7] An RSP write is not a k-factor write — no stale mark, no re-read. */
    {
        char resp[64];
        uint8_t cs = (uint8_t)(1 ^ 0xB7 ^ 0x00 ^ 0x70);
        char hex[40];
        snprintf(hex, sizeof(hex), "F5,01,A4,B7,00,70,%02X", cs);
        ok = du_exec_bluesky_passthrough(hex, 1, 2, BS_CMD_RSP, resp, sizeof(resp), rb, 64, reason, 32);
        CHECK(ok, "[7] rsp write refused: %s", reason);
        kf = round_kf(1, false, &reads);
        CHECK(strcmp(kf, "0400") == 0 && reads == 0, "[7] rsp write touched the k-factor: got %s, %d reads", kf, reads);
    }

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
