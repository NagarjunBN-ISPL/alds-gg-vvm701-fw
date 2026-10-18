/* Host harness for the REAL src/rsp_tunnel.c — the cloud's rsp/kfactor tunnel
 * — compiled whole against a simulated RS485 line with a virtual clock.
 *
 * What it pins (user decision 2026-10-18, the pilot's contract):
 *   - every payload form the pilot's strtol() parser read gives the same
 *     bytes, and anything else is refused before the bus sees a byte;
 *   - the bytes the cloud sent are the bytes that go out: once, nothing
 *     added, nothing checked, no retry, no read-back;
 *   - the reply is whatever the pump sends until the line is quiet for
 *     RS485_RECV_TIMEOUT_MS, published as received — valid frame or not —
 *     and "404" only when there is nothing whole to forward;
 *   - the queue keeps commands in arrival order and refuses the 17th. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "rsp_tunnel.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

int g_log_quiet = 1;
int g_wdt_feeds = 0;

/* ── virtual clock + ESP-IDF stubs ─────────────────────────────────────── */
static int64_t s_now_us = 1000000;
int64_t esp_timer_get_time(void) { return s_now_us; }
esp_err_t esp_task_wdt_status(void *h) { (void)h; return ESP_OK; }
esp_err_t esp_task_wdt_reset(void) { g_wdt_feeds++; return ESP_OK; }

/* ── simulated line ─────────────────────────────────────────────────────────
 * Every byte the pump sends is scheduled at an absolute virtual time; reads
 * deliver what is due and a read that has to wait advances the clock, so the
 * quiet window and the hard cap run exactly as on the board. */
#define LINE_MAX 4096
static struct { int64_t at_us; uint8_t b; } s_rx[LINE_MAX];
static int     s_rx_n, s_rx_pos;
static uint8_t s_tx[256];
static int     s_tx_len, s_writes;

/* What the pump does when a frame arrives: bursts, in time order. */
typedef struct { int delay_ms; const uint8_t *bytes; int n; int gap_ms; } Burst_t;
static Burst_t s_script[4];
static int     s_script_n;

static void line_reset(void) {
    s_rx_n = s_rx_pos = 0;
    s_tx_len = s_writes = 0;
    s_script_n = 0;
}

static void rx_at(int64_t at_us, uint8_t b) {
    if (s_rx_n < LINE_MAX) { s_rx[s_rx_n].at_us = at_us; s_rx[s_rx_n].b = b; s_rx_n++; }
}

static void answer(int delay_ms, const uint8_t *bytes, int n, int gap_ms) {
    if (s_script_n < 4) s_script[s_script_n++] = (Burst_t){delay_ms, bytes, n, gap_ms};
}

int uart_write_bytes(uart_port_t port, const void *src, size_t size) {
    (void)port;
    s_writes++;
    s_tx_len = (int)(size < sizeof(s_tx) ? size : sizeof(s_tx));
    memcpy(s_tx, src, (size_t)s_tx_len);
    for (int k = 0; k < s_script_n; k++)
        for (int i = 0; i < s_script[k].n; i++)
            rx_at(s_now_us + ((int64_t)s_script[k].delay_ms + (int64_t)i * s_script[k].gap_ms) * 1000,
                  s_script[k].bytes[i]);
    return (int)size;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length, uint32_t ticks) {
    (void)port;
    uint8_t *out = (uint8_t *)buf;
    if (s_rx_pos >= s_rx_n) { s_now_us += (int64_t)ticks * 1000; return 0; }
    if (s_rx[s_rx_pos].at_us > s_now_us) {               /* wait for the next byte */
        int64_t wait_end = s_now_us + (int64_t)ticks * 1000;
        if (s_rx[s_rx_pos].at_us > wait_end) { s_now_us = wait_end; return 0; }
        s_now_us = s_rx[s_rx_pos].at_us;
    }
    int n = 0;
    while (n < (int)length && s_rx_pos < s_rx_n && s_rx[s_rx_pos].at_us <= s_now_us)
        out[n++] = s_rx[s_rx_pos++].b;
    return n;
}

/* ── test scaffolding ──────────────────────────────────────────────────── */
static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do {                                        \
        checks++;                                                          \
        if (!(cond)) { fails++;                                            \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__); }                  \
    } while (0)

/* Vendor doc, Yenen head 1 side A, RSP 65.32 (checksum 71). */
static const uint8_t YN_WRITE[] = {0x02, 0x30, 0x38, 0x30, 0x31, 0x30, 0x4B,
                                   0x36, 0x35, 0x33, 0x32, 0x03, 0x37, 0x31};
/* BlueSky 664 RSP write, nozzle 2, 68.80; and the ack a station pump sent to
 * these writes on 2026-09-29. */
static const uint8_t BS_WRITE[] = {0xF5, 0x02, 0xA4, 0x68, 0x80, 0xB2, 0x09};
static const uint8_t BS_ACK[]   = {0xF5, 0x02, 0xA3, 0x59, 0xB2, 0x3F};

static bool decodes_to(const char *text, const uint8_t *want, size_t n) {
    uint8_t out[RSP_TUNNEL_REQ_MAX];
    size_t  got = rsp_tunnel_decode(text, out, sizeof(out));
    return got == n && memcmp(out, want, n) == 0;
}

static bool refused(const char *text) {
    uint8_t out[RSP_TUNNEL_REQ_MAX];
    return rsp_tunnel_decode(text, out, sizeof(out)) == 0;
}

static char s_reply[RSP_TUNNEL_REPLY_TEXT_MAX];

/* One exchange; returns its result and how long it held the line. */
static bool run(const uint8_t *req, size_t len, int64_t *ms) {
    int64_t t0 = s_now_us;
    bool ok = rsp_tunnel_exchange(req, len, s_reply, sizeof(s_reply));
    if (ms) *ms = (s_now_us - t0) / 1000;
    return ok;
}

static bool sent_exactly(const uint8_t *req, size_t len) {
    return s_writes == 1 && s_tx_len == (int)len && memcmp(s_tx, req, len) == 0;
}

static void t_decode(void) {
    printf("[1] payload forms\n");
    static const char *const YN_FORMS[] = {
        "02,30,38,30,31,30,4b,36,35,33,32,03,37,31",              /* the pilot's form       */
        "02,30,38,30,31,30,4B,36,35,33,32,03,37,31",
        "2,30,38,30,31,30,4b,36,35,33,32,3,37,31",                /* unpadded, as strtol    */
        "0x02,0x30,0x38,0x30,0x31,0x30,0x4B,0x36,0x35,0x33,0x32,0x03,0x37,0x31",
        "0X2, 0x30, 38, 30, 31, 30, 4B, 36, 35, 33, 32, 3, 37, 31",
        "02 30 38 30 31 30 4B 36 35 33 32 03 37 31",
        "02:30:38:30:31:30:4B:36:35:33:32:03:37:31",
        "02-30-38-30-31-30-4B-36-35-33-32-03-37-31",
        "0230383031304B36353332033731",
        "023038 3031304B 36353332 033731",
        "\"02,30,38,30,31,30,4b,36,35,33,32,03,37,31\"",          /* one JSON string        */
        "  \" 02 30 38 30 31 30 4b 36 35 33 32 03 37 31 \"\r\n",
        " 02,30,38,,30,31,30,4b,36,35,33,32,03,37,31, \r\n",      /* empty token skipped    */
        "{\"Status\":\"lock\",\"NZ1\":\"02,30,38,30,31,30,4B,36,35,33,32,03,37,31\"}",
        "{ \"Status\" : \"lock\" , \"NZ1\" : \"02 30 38 30 31 30 4B 36 35 33 32 03 37 31\" }\n",
        "{\"NZ1\":\"02,30,38,30,31,30,4B,36,35,33,32,03,37,31\",\"Status\":\"unlock\"}",
        "{\"Status\":\"lock\",\"NZ10\":\"02,30,38,30,31,30,4B,36,35,33,32,03,37,31\"}",
        "{\"status\":\"lock\",\"nz2\":\"02,30,38,30,31,30,4b,36,35,33,32,03,37,31\"}",
        "{\"Status\":\"NZ1\",\"NZ1\":\"02,30,38,30,31,30,4B,36,35,33,32,03,37,31\"}",
    };
    for (unsigned i = 0; i < sizeof(YN_FORMS) / sizeof(YN_FORMS[0]); i++)
        CHECK(decodes_to(YN_FORMS[i], YN_WRITE, sizeof(YN_WRITE)), "[1] not read as the Yenen write: %s", YN_FORMS[i]);
    CHECK(decodes_to("f5,02,a4,68,80,b2,09", BS_WRITE, sizeof(BS_WRITE)), "[1] BlueSky write");
    CHECK(decodes_to("F5,2,A4,68,80,B2,9", BS_WRITE, sizeof(BS_WRITE)),   "[1] BlueSky write, unpadded");
    CHECK(decodes_to("{\"Status\":\"lock\",\"NZ2\":\"F5,02,A4,68,80,B2,09\"}", BS_WRITE, sizeof(BS_WRITE)),
          "[1] BlueSky write in the pilot's wrapper");
    {   /* two frames in one wrapper: the first in the payload goes, nothing else */
        static const uint8_t two[] = {0xF5, 0x02};
        CHECK(decodes_to("{\"NZ2\":\"F5,02\",\"NZ1\":\"02,30\"}", two, sizeof(two)), "[1] first NZ value wins");
    }

    printf("[2] refused before the bus\n");
    static const char *const BAD[] = {
        "", "   ", "\"\"", ",,,", "02,1G,04", "02,10,0x", "021,10", "02,123", "+2,10",
        "\"02,10", "02,10\"", "f5 02 a4 68 80 b2 09 zz",
        "{\"frame\":\"02,10\"}", "{\"Status\":\"lock\"}", "{\"NZ1\":12}", "{\"NZ\":\"02\"}",
        "{\"NZ1\":\"02,30\"", "{\"NZ1\":\"02,3G\"}", "{\"NZ1\":\"\"}", "{\"NZ1\":\"02,30}",
        "{\"rsp\":65.32}", "{\"rsp\":65.32,\"nz\":1}",           /* the old setrsp payload */
        "{\"cmd\":\"setrsp\",\"sfx\":\"03\",\"rsp\":9800}",
    };
    for (unsigned i = 0; i < sizeof(BAD) / sizeof(BAD[0]); i++)
        CHECK(refused(BAD[i]), "[2] accepted: %s", BAD[i]);
    uint8_t out[4];
    CHECK(rsp_tunnel_decode(NULL, out, sizeof(out)) == 0, "[2] NULL payload");
    CHECK(rsp_tunnel_decode("02", NULL, 4) == 0,           "[2] NULL buffer");
    CHECK(rsp_tunnel_decode("02", out, 0) == 0,            "[2] zero-size buffer");

    /* Longer than allowed is refused WHOLE — a cut frame is a different frame. */
    char big[(RSP_TUNNEL_REQ_MAX + 1) * 3 + 1];
    int  pos = 0;
    for (int i = 0; i < RSP_TUNNEL_REQ_MAX + 1; i++)
        pos += snprintf(big + pos, sizeof(big) - (size_t)pos, "%s%02x", i ? "," : "", i);
    uint8_t buf[RSP_TUNNEL_REQ_MAX];
    CHECK(rsp_tunnel_decode(big, buf, sizeof(buf)) == 0, "[2] %d bytes accepted into %d", RSP_TUNNEL_REQ_MAX + 1, RSP_TUNNEL_REQ_MAX);
    big[RSP_TUNNEL_REQ_MAX * 3 - 1] = '\0';                  /* exactly REQ_MAX bytes */
    CHECK(rsp_tunnel_decode(big, buf, sizeof(buf)) == RSP_TUNNEL_REQ_MAX && buf[RSP_TUNNEL_REQ_MAX - 1] == RSP_TUNNEL_REQ_MAX - 1,
          "[2] an exact fit must be accepted");
}

static void t_exchange(void) {
    int64_t ms;
    bool    ok;

    printf("[3] the exchange\n");
    /* BlueSky RSP write: out once, unchanged; the ack forwarded as it came. */
    line_reset();
    answer(20, BS_ACK, (int)sizeof(BS_ACK), 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), &ms);
    CHECK(ok && strcmp(s_reply, "f5,02,a3,59,b2,3f") == 0, "[3] BlueSky ack: ok=%d reply=%s", ok, s_reply);
    CHECK(sent_exactly(BS_WRITE, sizeof(BS_WRITE)), "[3] BlueSky: %d writes, %d bytes — must be the frame, once", s_writes, s_tx_len);
    CHECK(ms >= RS485_RECV_TIMEOUT_MS && ms < RS485_RECV_TIMEOUT_MS + 100,
          "[3] reply must end on %d ms of quiet, took %lld ms", RS485_RECV_TIMEOUT_MS, (long long)ms);

    /* Yenen write, ACK 06 30 31 — not an STX frame, forwarded all the same. */
    static const uint8_t yn_ack[] = {0x06, 0x30, 0x31};
    line_reset();
    answer(15, yn_ack, (int)sizeof(yn_ack), 1);
    ok = run(YN_WRITE, sizeof(YN_WRITE), NULL);
    CHECK(ok && strcmp(s_reply, "06,30,31") == 0, "[3] Yenen ACK: ok=%d reply=%s", ok, s_reply);
    CHECK(sent_exactly(YN_WRITE, sizeof(YN_WRITE)), "[3] Yenen: the frame must go out once, unchanged");

    /* A 51-byte Command 2 reply comes back whole. */
    uint8_t c2[51];
    for (int i = 0; i < 51; i++) c2[i] = (uint8_t)(0x30 + i % 10);
    c2[0] = 0x02; c2[48] = 0x03;
    line_reset();
    answer(10, c2, 51, 1);
    ok = run(YN_WRITE, 10, NULL);
    CHECK(ok && strlen(s_reply) == 51 * 3 - 1 && strncmp(s_reply, "02,31,32", 8) == 0,
          "[3] 51-byte reply: ok=%d len=%zu", ok, strlen(s_reply));

    /* Nothing is validated: a NAK, a broken checksum, another nozzle's frame. */
    static const uint8_t nak[]     = {0x15};
    static const uint8_t bad_chk[] = {0xF5, 0x02, 0xA3, 0x59, 0xB2, 0x00};
    static const uint8_t other[]   = {0xF5, 0x05, 0xA3, 0x59, 0xB2, 0x38};
    line_reset(); answer(5, nak, 1, 1);
    ok = run(YN_WRITE, sizeof(YN_WRITE), NULL);
    CHECK(ok && strcmp(s_reply, "15") == 0, "[3] NAK must be forwarded: %s", s_reply);
    line_reset(); answer(5, bad_chk, (int)sizeof(bad_chk), 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), NULL);
    CHECK(ok && strcmp(s_reply, "f5,02,a3,59,b2,00") == 0, "[3] a bad checksum must be forwarded: %s", s_reply);
    line_reset(); answer(5, other, (int)sizeof(other), 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), NULL);
    CHECK(ok && strcmp(s_reply, "f5,05,a3,59,b2,38") == 0, "[3] another nozzle's frame must be forwarded: %s", s_reply);

    /* Silent pump: 404 after the first-byte window, ONE attempt. */
    line_reset();
    g_wdt_feeds = 0;
    ok = run(BS_WRITE, sizeof(BS_WRITE), &ms);
    CHECK(!ok && strcmp(s_reply, "404") == 0, "[3] silent: ok=%d reply=%s", ok, s_reply);
    CHECK(s_writes == 1, "[3] silent pump: %d writes — the tunnel never retries", s_writes);
    CHECK(ms >= RS485_RECV_TIMEOUT_MS && ms <= RS485_RECV_TIMEOUT_MS + 20, "[3] silent pump held the line %lld ms", (long long)ms);
    CHECK(g_wdt_feeds > 100, "[3] only %d WDT feeds while waiting", g_wdt_feeds);

    /* A pause shorter than the window is inside the reply; a longer one ends it. */
    static const uint8_t p1[] = {0xF5, 0x02, 0xA3}, p2[] = {0x59, 0xB2, 0x3F};
    line_reset(); answer(10, p1, 3, 1); answer(510, p2, 3, 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), NULL);
    CHECK(ok && strcmp(s_reply, "f5,02,a3,59,b2,3f") == 0, "[3] reply with a 500 ms pause: %s", s_reply);
    line_reset(); answer(10, p1, 3, 1); answer(10 + RS485_RECV_TIMEOUT_MS + 500, p2, 3, 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), NULL);
    CHECK(ok && strcmp(s_reply, "f5,02,a3") == 0, "[3] bytes after the quiet window are not the reply: %s", s_reply);

    /* Leftovers in the RX buffer are flushed, never published as this reply. */
    line_reset();
    rx_at(s_now_us - 5000, 0xAA);
    rx_at(s_now_us, 0xBB);
    answer(20, BS_ACK, (int)sizeof(BS_ACK), 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), NULL);
    CHECK(ok && strcmp(s_reply, "f5,02,a3,59,b2,3f") == 0, "[3] stale bytes leaked into the reply: %s", s_reply);

    printf("[4] never a cut frame\n");
    /* A line that never goes quiet: 404 at the cap, nothing partial. */
    static uint8_t babble[100];
    memset(babble, 0x55, sizeof(babble));
    line_reset(); answer(50, babble, (int)sizeof(babble), 100);
    ok = run(BS_WRITE, sizeof(BS_WRITE), &ms);
    CHECK(!ok && strcmp(s_reply, "404") == 0, "[4] babbling line: ok=%d reply=%.20s", ok, s_reply);
    CHECK(ms >= 3 * RS485_RECV_TIMEOUT_MS && ms < 3 * RS485_RECV_TIMEOUT_MS + 100, "[4] cap took %lld ms", (long long)ms);

    /* RESP_MAX bytes fit whole; one more is refused whole. */
    static uint8_t lots[RSP_TUNNEL_RESP_MAX + 1];
    for (int i = 0; i < RSP_TUNNEL_RESP_MAX + 1; i++) lots[i] = (uint8_t)i;
    line_reset(); answer(5, lots, RSP_TUNNEL_RESP_MAX, 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), NULL);
    CHECK(ok && strlen(s_reply) == (size_t)RSP_TUNNEL_RESP_MAX * 3 - 1, "[4] %d-byte reply: ok=%d len=%zu",
          RSP_TUNNEL_RESP_MAX, ok, strlen(s_reply));
    line_reset(); answer(5, lots, RSP_TUNNEL_RESP_MAX + 1, 1);
    ok = run(BS_WRITE, sizeof(BS_WRITE), NULL);
    CHECK(!ok && strcmp(s_reply, "404") == 0, "[4] %d-byte reply must be 404: ok=%d", RSP_TUNNEL_RESP_MAX + 1, ok);

    /* A reply buffer too small for the reply: 404, never cut. */
    char small[10];
    line_reset(); answer(5, BS_ACK, (int)sizeof(BS_ACK), 1);
    ok = rsp_tunnel_exchange(BS_WRITE, sizeof(BS_WRITE), small, sizeof(small));
    CHECK(!ok && strcmp(small, "404") == 0, "[4] small buffer: ok=%d reply=%s", ok, small);

    /* Bad arguments never reach the bus. */
    line_reset();
    ok = rsp_tunnel_exchange(BS_WRITE, 0, s_reply, sizeof(s_reply));
    CHECK(!ok && s_writes == 0 && strcmp(s_reply, "404") == 0, "[4] zero-length request reached the bus");
    ok = rsp_tunnel_exchange(BS_WRITE, RSP_TUNNEL_REQ_MAX + 1, s_reply, sizeof(s_reply));
    CHECK(!ok && s_writes == 0, "[4] over-long request reached the bus");
    ok = rsp_tunnel_exchange(NULL, 5, s_reply, sizeof(s_reply));
    CHECK(!ok && s_writes == 0, "[4] NULL request reached the bus");
    char tiny[3] = {'x', 'y', 'z'};
    ok = rsp_tunnel_exchange(BS_WRITE, sizeof(BS_WRITE), tiny, sizeof(tiny));
    CHECK(!ok && s_writes == 0 && tiny[0] == 'x', "[4] a reply buffer that cannot hold 404 must be left alone");
}

static void t_queue(void) {
    printf("[5] queue\n");
    RspTunnelCmd_t c;
    memset(&c, 0, sizeof(c));
    CHECK(!rsp_tunnel_pop(&c), "[5] pop from an empty queue");
    uint8_t f[RSP_TUNNEL_REQ_MAX];
    memset(f, 0, sizeof(f));
    for (int i = 0; i < RSP_TUNNEL_QUEUE_MAX; i++) {
        f[0] = (uint8_t)i;
        CHECK(rsp_tunnel_push((uint8_t)(i % 6), (uint8_t)(i & 1), f, (size_t)(i + 1)), "[5] push %d refused", i);
    }
    f[0] = 0xEE;
    CHECK(!rsp_tunnel_push(0, 0, f, 1), "[5] push %d accepted into a full queue", RSP_TUNNEL_QUEUE_MAX + 1);
    for (int i = 0; i < RSP_TUNNEL_QUEUE_MAX; i++) {
        bool ok = rsp_tunnel_pop(&c);
        CHECK(ok && c.unit == i % 6 && c.kind == (i & 1) && c.len == i + 1 && c.frame[0] == i,
              "[5] pop %d: unit=%u kind=%u len=%u first=%u", i, c.unit, c.kind, c.len, c.frame[0]);
    }
    CHECK(!rsp_tunnel_pop(&c), "[5] queue must be empty again");

    /* Wrap-around keeps arrival order. */
    int next_in = 0, next_out = 0;
    for (int round = 0; round < 200; round++) {
        int pushes = 1 + round % 3;
        for (int k = 0; k < pushes; k++) {
            f[0] = (uint8_t)next_in;
            if (rsp_tunnel_push(1, 0, f, 1)) next_in++;
        }
        int pops = 1 + (round + 1) % 3;
        for (int k = 0; k < pops && rsp_tunnel_pop(&c); k++) {
            CHECK(c.frame[0] == (uint8_t)next_out, "[5] order broken at %d: got %u", next_out, c.frame[0]);
            next_out++;
        }
    }
    while (rsp_tunnel_pop(&c)) {
        CHECK(c.frame[0] == (uint8_t)next_out, "[5] order broken at %d: got %u", next_out, c.frame[0]);
        next_out++;
    }
    CHECK(next_in == next_out, "[5] %d pushed, %d popped", next_in, next_out);

    /* Nothing that cannot be sent is queued. */
    CHECK(!rsp_tunnel_push(0, 0, f, 0), "[5] zero-length frame queued");
    CHECK(!rsp_tunnel_push(0, 0, f, RSP_TUNNEL_REQ_MAX + 1), "[5] over-long frame queued");
    CHECK(!rsp_tunnel_push(0, 0, NULL, 4), "[5] NULL frame queued");
    CHECK(!rsp_tunnel_pop(&c), "[5] a refused push left something behind");
}

int main(void) {
    printf("== rsp/kfactor tunnel — the real rsp_tunnel.c vs a simulated line ==\n");
    t_decode();
    t_exchange();
    t_queue();
    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
