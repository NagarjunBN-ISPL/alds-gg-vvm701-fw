/* Host harness for the REAL src/dispenser.c — compiled whole against a
 * simulated BlueSky pump — covering the k-factor cache.
 *
 * The k-factor is read every KFACTOR_POLL_INTERVAL_S and carried forward in
 * between. The cloud can change it through the rsp/kfactor tunnel
 * (rsp_tunnel.h), whose frames the gateway does not read. The bug this pins:
 * after such a write, every nozzle message kept publishing the OLD value until
 * the next scheduled read, up to 15 minutes later. Now a tunnel command marks
 * the cache stale (du_kfactor_mark_stale) and the next round reads the
 * k-factor even when it is not due. */
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
 * bit 7 cleared — since the driver drops anything else. A k-factor written
 * through the tunnel is modelled by changing P.kf[] behind the driver's back:
 * the tunnel's bytes never pass through dispenser.c. */
static struct {
    uint8_t kf[16][2];         /* k-factor per address                           */
    bool    busy[16];          /* status 0x81 (delivering) instead of 0x80       */
    int     drop_kf_reads;     /* next N 0xE2 reads go unanswered                */
    bool    never_kf[16];      /* this address never answers 0xE2                */
    int     kf_reads;          /* 0xE2 requests seen (answered or not)           */
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
    if (size != 5 || f[0] != 0xF5) return (int)size;
    uint8_t a = f[1];
    if (a >= 16) return (int)size;

    switch (f[3]) {
        case 0xD5: { uint8_t d[] = {(uint8_t)(P.busy[a] ? 0x81 : 0x80)};      reply_read(a, 0xD5, d, 1);  break; }
        case 0xC5: { uint8_t d[] = {0x00, 0x02, 0x15, 0x18, 0x26,
                                    0x23, 0x30, 0x19, 0x35, 0x34};            reply_read(a, 0xC5, d, 10); break; }
        case 0xB6: { uint8_t d[] = {0x68, 0x86};                              reply_read(a, 0xB6, d, 2);  break; }
        case 0xE2:
            P.kf_reads++;
            if (P.never_kf[a]) break;
            if (P.drop_kf_reads > 0) { P.drop_kf_reads--; break; }
            reply_read(a, 0xE2, P.kf[a], 2);
            break;
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

/* The cloud writes a new k-factor into nozzle `addr` through the tunnel. */
static void tunnel_write_kf(uint8_t addr, uint8_t k1, uint8_t k2) {
    P.kf[addr][0] = k1;
    P.kf[addr][1] = k2;
    du_kfactor_mark_stale();       /* what execute_pending() does after any BlueSky exchange */
}

int main(void) {
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

    /* [2] Why the mark exists: a pump changed behind the cache is NOT seen by a
     *     non-due round on its own ... */
    P.kf[1][0] = 0x04; P.kf[1][1] = 0x00;
    kf = round_kf(1, false, &reads);
    CHECK(strcmp(kf, "03e8") == 0 && reads == 0, "[2] unmarked change must stay cached: got %s, %d reads", kf, reads);
    /*     ... and after a tunnel command the very next round shows it, then stops
     *     re-reading. */
    tunnel_write_kf(1, 0x04, 0x00);
    kf = round_kf(1, false, &reads);
    CHECK(strcmp(kf, "0400") == 0 && reads == 1, "[2] next message after a tunnel write: got %s, %d reads", kf, reads);
    kf = round_kf(1, false, &reads);
    CHECK(strcmp(kf, "0400") == 0 && reads == 0, "[2] the re-read cleared the mark: got %s, %d reads", kf, reads);

    /* [3] Marked, and the forced re-read fails: carry the last good value, stay
     *     marked, and read again next round until it succeeds. */
    P.kf[5][0] = 0x03; P.kf[5][1] = 0xE8;
    (void)round_kf(5, true, NULL);                       /* cache 03e8 */
    tunnel_write_kf(5, 0x07, 0x00);
    P.drop_kf_reads = DU_KFACTOR_RETRIES;                /* the whole next round */
    kf = round_kf(5, false, &reads);
    CHECK(strcmp(kf, "03e8") == 0 && reads == DU_KFACTOR_RETRIES,
          "[3] failed forced re-read carries last good: got %s, %d reads", kf, reads);
    kf = round_kf(5, false, &reads);
    CHECK(strcmp(kf, "0700") == 0 && reads == 1, "[3] still marked, re-read next round: got %s, %d reads", kf, reads);
    kf = round_kf(5, false, &reads);
    CHECK(strcmp(kf, "0700") == 0 && reads == 0, "[3] cleared once read: got %s, %d reads", kf, reads);

    /* [4] The gateway cannot tell which nozzle a tunnel frame was for, so one
     *     mark re-reads EVERY nozzle that has a cached k-factor, once. */
    P.kf[2][0] = 0x01; P.kf[2][1] = 0x11;
    P.kf[3][0] = 0x03; P.kf[3][1] = 0x33;
    (void)round_kf(2, true, NULL);
    (void)round_kf(3, true, NULL);
    tunnel_write_kf(3, 0x09, 0x99);
    static const uint8_t cached[] = {1, 2, 3, 5};
    for (unsigned i = 0; i < sizeof(cached); i++) {
        (void)round_kf(cached[i], false, &reads);
        CHECK(reads == 1, "[4] nozzle %u: %d re-reads after a mark, want 1", cached[i], reads);
        (void)round_kf(cached[i], false, &reads);
        CHECK(reads == 0, "[4] nozzle %u: %d reads on the round after, want 0", cached[i], reads);
    }
    CHECK(strcmp(round_kf(3, false, NULL), "0999") == 0, "[4] nozzle 3 must publish the written value");

    /* [5] A nozzle that has never answered a k-factor read is not forced: it
     *     would spend its whole retry budget every round. It keeps "404" until
     *     its scheduled read. */
    P.never_kf[7] = true;
    (void)round_kf(7, true, NULL);                       /* due read fails */
    tunnel_write_kf(1, 0x04, 0x00);
    kf = round_kf(7, false, &reads);
    CHECK(strcmp(kf, "404") == 0 && reads == 0, "[5] never-answered nozzle forced: got %s, %d reads", kf, reads);

    /* [6] A busy nozzle reads nothing; the mark waits for its next idle round. */
    tunnel_write_kf(2, 0x02, 0x22);
    P.busy[2] = true;
    (void)round_kf(2, false, &reads);
    CHECK(reads == 0, "[6] busy nozzle read its k-factor (%d reads)", reads);
    P.busy[2] = false;
    kf = round_kf(2, false, &reads);
    CHECK(strcmp(kf, "0222") == 0 && reads == 1, "[6] first idle round after the mark: got %s, %d reads", kf, reads);

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
