/* Host harness for the REAL src/yenen.c — compiled natively against a
 * simulated Yenen pump. Nothing here is a copy of firmware logic: the frames,
 * checksums, retries, idle gate and verification all come from the production
 * source. */
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
typedef enum { W_ACK_STORE, W_ACK_NOSTORE, W_NAK, W_SILENT, W_ACK_PARTIAL } WriteBehav_t;

typedef struct {
    bool         present;        /* answers reads at all                     */
    bool         idle;           /* status field 00 vs 11                    */
    uint16_t     price;          /* what the pump currently holds            */
    WriteBehav_t wb;
    bool         wrong_echo;     /* answer with the OTHER side's address     */
    int          fuzz;           /* answer with random bytes: a noisy bus, a
                                  * wrong-protocol device, a half-powered pump */
    bool         flaky;          /* drop every 1st attempt, answer the 2nd —
                                  * forces every stage to spend its full retry
                                  * budget and still succeed: the true
                                  * worst-case bus hold */
    int          a_frames;       /* Command 2 (read) frames received         */
    int          k_frames;       /* Command K (write) frames received        */
    int          k_bad_checksum; /* write frames whose checksum did not verify*/
    uint16_t     last_written;   /* price digits from the last write frame    */
    uint8_t      last_k[32];
    int          last_k_len;
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

/* 51-byte Command 2 reply, laid out per Appendix B of the cloud guide. */
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

    if (size < 8) return (int)size;

    uint8_t head = (uint8_t)(f[4] - '0');
    uint8_t side = (f[5] >= '5') ? 1 : 0;
    uint8_t echo_side = P.wrong_echo ? (uint8_t)!side : side;

    if (f[6] == 'A') {                         /* Command 2 — read           */
        P.a_frames++;
        if (!P.present) return (int)size;      /* silence                    */
        if (P.flaky && (P.a_frames % 2) == 1) return (int)size;
        uint8_t rep[64]; int rlen;
        build_read_reply(head, echo_side, P.price, P.idle, rep, &rlen);
        rx_push(rep, rlen);
    } else if (f[6] == 'K') {                  /* Command K — write RSP      */
        P.k_frames++;
        memcpy(P.last_k, f, size < sizeof(P.last_k) ? size : sizeof(P.last_k));
        P.last_k_len = (int)size;

        uint8_t want[2];
        ck(f, 12, want);                       /* STX..ETX inclusive         */
        if (size != 14 || f[11] != 0x03 || f[12] != want[0] || f[13] != want[1])
            P.k_bad_checksum++;

        P.last_written = (uint16_t)((f[7]-'0')*1000 + (f[8]-'0')*100 +
                                    (f[9]-'0')*10   + (f[10]-'0'));
        if (P.flaky && (P.k_frames % 2) == 1) return (int)size;
        switch (P.wb) {
            case W_ACK_STORE:   P.price = P.last_written;
                                rx_push((const uint8_t *)"\x06" "01", 3); break;
            case W_ACK_NOSTORE: rx_push((const uint8_t *)"\x06" "01", 3); break;
            case W_ACK_PARTIAL: rx_push((const uint8_t *)"\x06", 1);      break;
            case W_NAK:         rx_push((const uint8_t *)"\x15", 1);      break;
            case W_SILENT:      break;
        }
    }
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

static void pump_reset(bool present, bool idle, uint16_t price, WriteBehav_t wb) {
    memset(&P, 0, sizeof(P));
    P.present = present; P.idle = idle; P.price = price; P.wb = wb;
    rx_reset();
}

static const char *hex(const uint8_t *b, int n) {
    static char s[128];
    int pos = 0;
    for (int i = 0; i < n && pos + 4 < (int)sizeof(s); i++)
        pos += sprintf(s + pos, "%s%02x", i ? "," : "", b[i]);
    s[pos] = 0;
    return s;
}

int main(void) {
    printf("== yenen RSP update — host verification against the real driver ==\n");

    /* 1. Frame builder: every documented vendor example, produced by the
     *    firmware's own builder (captured off the simulated wire). */
    struct { uint8_t h, s; uint16_t p; const char *want; } vec[] = {
        {1,0,6532,"02,30,38,30,31,30,4b,36,35,33,32,03,37,31"},
        {1,1,6532,"02,30,38,30,31,35,4b,36,35,33,32,03,37,34"},
        {1,0,6565,"02,30,38,30,31,30,4b,36,35,36,35,03,37,33"},
        {1,1,6565,"02,30,38,30,31,35,4b,36,35,36,35,03,37,36"},
        {1,0,9999,"02,30,38,30,31,30,4b,39,39,39,39,03,37,33"},
        {1,1,9999,"02,30,38,30,31,35,4b,39,39,39,39,03,37,36"},
        {1,0,6533,"02,30,38,30,31,30,4b,36,35,33,33,03,37,30"},
        {1,1,6533,"02,30,38,30,31,35,4b,36,35,33,33,03,37,35"},
    };
    printf("[1] vendor frame vectors\n");
    for (unsigned i = 0; i < sizeof(vec)/sizeof(vec[0]); i++) {
        pump_reset(true, true, 6500, W_ACK_STORE);
        yn_set_rsp(vec[i].h, vec[i].s, vec[i].p, NULL);
        const char *got = hex(P.last_k, P.last_k_len);
        CHECK(P.last_k_len == 14 && strcmp(got, vec[i].want) == 0,
              "head%u side%u %u -> %s (want %s)", vec[i].h, vec[i].s, vec[i].p, got, vec[i].want);
    }

    /* 2. Happy path. */
    printf("[2] happy path\n");
    {
        pump_reset(true, true, 6500, W_ACK_STORE);
        uint16_t back = 0;
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, &back);
        CHECK(r == YN_RSP_OK, "result %s, want RSP_updated", yn_rsp_result_str(r));
        CHECK(back == 6532, "readback %u, want 6532", back);
        CHECK(P.price == 6532, "pump holds %u", P.price);
        CHECK(P.k_frames == 1, "write frames %d, want 1", P.k_frames);
        CHECK(P.a_frames == 2, "read frames %d, want 2 (pre-read + verify)", P.a_frames);
        CHECK(P.k_bad_checksum == 0, "pump rejected %d checksums", P.k_bad_checksum);
    }

    /* 3. Mid-delivery: MUST NOT write. */
    printf("[3] nozzle dispensing\n");
    {
        pump_reset(true, false, 6500, W_ACK_STORE);
        uint16_t back = 1;
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, &back);
        CHECK(r == YN_RSP_BUSY, "result %s, want nozzle_busy", yn_rsp_result_str(r));
        CHECK(P.k_frames == 0, "wrote %d frames to a dispensing nozzle", P.k_frames);
        CHECK(P.price == 6500, "price changed under an active sale");
        CHECK(back == 0, "readback %u, want 0 when nothing was written", back);
    }

    /* 4. Silent nozzle: no write, bounded attempts. */
    printf("[4] silent nozzle\n");
    {
        pump_reset(false, true, 6500, W_ACK_STORE);
        int64_t t0 = esp_timer_get_time();
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, NULL);
        int64_t held_ms = (esp_timer_get_time() - t0) / 1000;
        CHECK(r == YN_RSP_NO_RESPONSE, "result %s", yn_rsp_result_str(r));
        CHECK(P.k_frames == 0, "wrote %d frames to a silent nozzle", P.k_frames);
        CHECK(P.a_frames == YN_RSP_RETRIES, "read attempts %d, want %d", P.a_frames, YN_RSP_RETRIES);
        CHECK(held_ms < 12000, "held the bus %lld ms — over the 11.5 s budget", (long long)held_ms);
        printf("       bus held %lld ms on a fully silent nozzle\n", (long long)held_ms);
    }

    /* 5. Pump rejects the write. */
    printf("[5] NAK\n");
    {
        pump_reset(true, true, 6500, W_NAK);
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, NULL);
        CHECK(r == YN_RSP_NAK, "result %s, want nak", yn_rsp_result_str(r));
        CHECK(P.price == 6500, "price changed despite NAK");
        CHECK(P.k_frames == YN_RSP_RETRIES, "write attempts %d, want %d", P.k_frames, YN_RSP_RETRIES);
    }

    /* 6. No answer to the write. */
    printf("[6] no ACK\n");
    {
        pump_reset(true, true, 6500, W_SILENT);
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, NULL);
        CHECK(r == YN_RSP_NO_ACK, "result %s, want no_ack", yn_rsp_result_str(r));
    }

    /* 7. Truncated ACK (06 with no "01") must not count as success. */
    printf("[7] partial ACK\n");
    {
        pump_reset(true, true, 6500, W_ACK_PARTIAL);
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, NULL);
        CHECK(r == YN_RSP_NO_ACK,
              "result %s — a truncated ACK is unconfirmed, not a rejection",
              yn_rsp_result_str(r));
    }

    /* 8. ACK but the pump kept the old price. */
    printf("[8] ACK without storing\n");
    {
        pump_reset(true, true, 6500, W_ACK_NOSTORE);
        uint16_t back = 0;
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, &back);
        CHECK(r == YN_RSP_VERIFY_FAILED, "result %s, want verify_failed", yn_rsp_result_str(r));
        CHECK(back == 6500, "readback %u — the ack must carry what the pump HOLDS", back);
    }

    /* 9. The field defect: pump answers for the other nozzle. */
    printf("[9] wrong address echo\n");
    {
        pump_reset(true, true, 6500, W_ACK_STORE);
        P.wrong_echo = true;
        YnRspResult_t r = yn_set_rsp(1, 0, 6532, NULL);
        CHECK(r == YN_RSP_NO_RESPONSE, "result %s", yn_rsp_result_str(r));
        CHECK(P.k_frames == 0, "wrote to a nozzle that answered for a different address");
    }

    /* 10. Argument guards must never reach the bus. */
    printf("[10] argument guards\n");
    {
        struct { uint8_t h, s; uint16_t p; const char *what; } bad[] = {
            {0,0,6532,"head 0"}, {DU_YENEN_MAX+1,0,6532,"head over max"},
            {1,2,6532,"side 2"}, {1,0,0,"price 0"}, {1,0,10000,"price 100.00"},
        };
        for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
            pump_reset(true, true, 6500, W_ACK_STORE);
            YnRspResult_t r = yn_set_rsp(bad[i].h, bad[i].s, bad[i].p, NULL);
            CHECK(r == YN_RSP_BAD_ARG && P.a_frames == 0 && P.k_frames == 0,
                  "%s: result %s, %d reads, %d writes", bad[i].what,
                  yn_rsp_result_str(r), P.a_frames, P.k_frames);
        }
    }

    /* 11. Every head/side the config allows produces a distinct, valid frame. */
    printf("[11] address space 1..%d x A/B\n", DU_YENEN_MAX);
    {
        char seen[16][64]; int n = 0;
        for (uint8_t h = 1; h <= DU_YENEN_MAX; h++) {
            for (uint8_t s = 0; s < 2; s++) {
                pump_reset(true, true, 6500, W_ACK_STORE);
                YnRspResult_t r = yn_set_rsp(h, s, 8250, NULL);
                CHECK(r == YN_RSP_OK, "head%u/%c failed: %s", h, s?'B':'A', yn_rsp_result_str(r));
                CHECK(P.k_bad_checksum == 0, "head%u/%c bad checksum", h, s?'B':'A');
                snprintf(seen[n], sizeof(seen[n]), "%s", hex(P.last_k, P.last_k_len));
                for (int j = 0; j < n; j++)
                    CHECK(strcmp(seen[j], seen[n]) != 0, "head%u/%c frame collides with an earlier nozzle", h, s?'B':'A');
                n++;
            }
        }
    }

    /* 12. Watchdog is fed while the driver waits on a silent pump. */
    printf("[12] watchdog feeding\n");
    {
        pump_reset(false, true, 6500, W_ACK_STORE);
        g_wdt_feeds = 0;
        yn_set_rsp(1, 0, 6532, NULL);
        CHECK(g_wdt_feeds > 100, "only %d WDT feeds during a silent-pump sequence", g_wdt_feeds);
        printf("       %d feeds\n", g_wdt_feeds);
    }

    /* 13. Boundary prices round-trip exactly. */
    printf("[13] price boundaries\n");
    {
        uint16_t prices[] = {1, 99, 100, 999, 1000, 6532, 9998, 9999};
        for (unsigned i = 0; i < sizeof(prices)/sizeof(prices[0]); i++) {
            pump_reset(true, true, 6500, W_ACK_STORE);
            uint16_t back = 0;
            YnRspResult_t r = yn_set_rsp(2, 1, prices[i], &back);
            CHECK(r == YN_RSP_OK && back == prices[i] && P.last_written == prices[i],
                  "price %u -> result %s, wire %u, readback %u",
                  prices[i], yn_rsp_result_str(r), P.last_written, back);
        }
    }

    /* 14. Worst-case bus hold across every failure mode — this is the number
     *     config.h's budget note and du_task's 25 s lock wait depend on. */
    printf("[14] worst-case bus hold per nozzle\n");
    {
        struct { const char *name; bool present; bool idle; WriteBehav_t wb; bool echo; } sc[] = {
            {"healthy, confirmed",      true,  true,  W_ACK_STORE,   false},
            {"dispensing",              true,  false, W_ACK_STORE,   false},
            {"silent nozzle",           false, true,  W_ACK_STORE,   false},
            {"answers read, no ACK",    true,  true,  W_SILENT,      false},
            {"answers read, NAK",       true,  true,  W_NAK,         false},
            {"ACK, wrong price",        true,  true,  W_ACK_NOSTORE, false},
            {"truncated ACK",           true,  true,  W_ACK_PARTIAL, false},
            {"wrong address echo",      true,  true,  W_ACK_STORE,   true},
        };
        int64_t worst = 0; const char *worst_name = "";
        for (unsigned i = 0; i <= sizeof(sc)/sizeof(sc[0]); i++) {
            if (i == sizeof(sc)/sizeof(sc[0])) {      /* the flaky-line case  */
                pump_reset(true, true, 6500, W_ACK_STORE);
                P.flaky = true;
                int64_t t0f = esp_timer_get_time();
                YnRspResult_t rf = yn_set_rsp(1, 0, 6532, NULL);
                int64_t msf = (esp_timer_get_time() - t0f) / 1000;
                printf("       %-22s %-14s %5lld ms\n", "flaky line (retries)",
                       yn_rsp_result_str(rf), (long long)msf);
                CHECK(rf == YN_RSP_OK, "flaky line should still succeed: %s",
                      yn_rsp_result_str(rf));
                if (msf > worst) { worst = msf; worst_name = "flaky line"; }
                continue;
            }
            pump_reset(sc[i].present, sc[i].idle, 6500, sc[i].wb);
            P.wrong_echo = sc[i].echo;
            int64_t t0 = esp_timer_get_time();
            YnRspResult_t r = yn_set_rsp(1, 0, 6532, NULL);
            int64_t ms = (esp_timer_get_time() - t0) / 1000;
            printf("       %-22s %-14s %5lld ms\n", sc[i].name, yn_rsp_result_str(r), (long long)ms);
            if (ms > worst) { worst = ms; worst_name = sc[i].name; }
        }
        printf("       worst: %s at %lld ms\n", worst_name, (long long)worst);
        CHECK(worst < RS485_BUS_LOCK_TIMEOUT_MS,
              "worst hold %lld ms exceeds du_task's %d ms wait — a poll round could be DROPPED",
              (long long)worst, RS485_BUS_LOCK_TIMEOUT_MS);
        /* Two sides back to back is the most one command can cost the bus, and
         * it is released in between — but even summed it must stay clear. */
        CHECK(worst * 2 < RS485_BUS_LOCK_TIMEOUT_MS,
              "two sides (%lld ms) exceed du_task's wait", (long long)worst * 2);
    }

    /* 15. A noisy bus: random bytes answering every command. Nothing here can
     *     be "correct" — the only requirement is that the driver never reads
     *     out of bounds, never accepts junk as a confirmed price, and always
     *     produces a publishable payload. */
    printf("[15] noisy bus (random replies)\n");
    {
        int rounds = 20000;
        int accepted_as_ok = 0, built = 0;
        char payload[DU_PAYLOAD_MAX];
        for (int it = 0; it < rounds; it++) {
            pump_reset(true, true, 6500, W_ACK_STORE);
            P.fuzz = 1;
            uint8_t head = (uint8_t)(1 + (rnd() % DU_YENEN_MAX));
            uint8_t side = (uint8_t)(rnd() % 2);

            uint16_t back = 0xFFFF;
            YnRspResult_t r = yn_set_rsp(head, side, (uint16_t)(1 + rnd() % 9999), &back);
            if (r == YN_RSP_OK) accepted_as_ok++;
            CHECK(back == 0 || (back >= 1 && back <= 9999) || r != YN_RSP_OK,
                  "confirmed an out-of-range price %u from noise", back);

            /* the poll path and the payload builder over the same noise */
            YenenNozzle_t nz;
            yn_poll_nozzle(head, side, &nz);
            CHECK(nz.total_len >= 0 && nz.total_len <= YN_RESP_MAX,
                  "total_len %d escaped the buffer", nz.total_len);
            yn_build_nozzle_payload(&nz, 1 + (int)(rnd() % 22), "860987051512728",
                                    "2026-09-02 11:04:19", payload, sizeof(payload));
            CHECK(strlen(payload) < sizeof(payload), "payload not terminated in buffer");
            CHECK(payload[0] == '{' && payload[strlen(payload)-1] == '}',
                  "payload is not a JSON object");
            built++;

            /* and the raw passthrough with random hex text */
            char hex[80];
            size_t hn = rnd() % (sizeof(hex) - 1);
            for (size_t i = 0; i < hn; i++) hex[i] = "0123456789abcdefABCDEF,: xyz"[rnd() % 28];
            hex[hn] = '\0';
            char resp[256];
            (void)yn_exec_passthrough(hex, resp, sizeof(resp));
            CHECK(strlen(resp) < sizeof(resp), "passthrough response overran its buffer");
        }
        printf("       %d rounds, %d payloads built, %d noise frames accepted as confirmed\n",
               rounds, built, accepted_as_ok);
        CHECK(accepted_as_ok == 0,
              "%d random frames were accepted as a CONFIRMED price update", accepted_as_ok);
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
