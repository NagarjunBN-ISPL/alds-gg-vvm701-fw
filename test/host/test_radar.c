/* Host harness for the REAL src/radar.c — compiled whole against a simulated
 * Modbus radar on a virtual clock — covering the reply timing in config.h:
 * a radar gets MODBUS_REPLY_TIMEOUT_MS to start answering, the read then ends
 * at the frame length or after MODBUS_BYTE_GAP_MS of silence, and never runs
 * past 2 x MODBUS_REPLY_TIMEOUT_MS, so line noise cannot hold the bus. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "radar.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/task.h"

int g_log_quiet = 1;
int g_wdt_feeds = 0;

/* ── virtual clock + ESP-IDF stubs ─────────────────────────────────────── */
static int64_t s_now_us = 1000000;
int64_t esp_timer_get_time(void) { return s_now_us; }
void vTaskDelay(TickType_t ticks) { s_now_us += (int64_t)ticks * 1000; }
esp_err_t esp_task_wdt_status(void *h) { (void)h; return ESP_OK; }
esp_err_t esp_task_wdt_reset(void) { g_wdt_feeds++; return ESP_OK; }

static uint16_t crc16(const uint8_t *b, int n) {
    uint16_t c = 0xFFFF;
    for (int i = 0; i < n; i++) {
        c ^= b[i];
        for (int k = 0; k < 8; k++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
    }
    return c;
}

/* ── simulated radar ───────────────────────────────────────────────────────
 * Answers every request with R.reply, R.delay_ms after it; R.drip instead
 * trickles one noise byte every 90 ms, forever. */
static struct {
    uint8_t reply[32];
    int     reply_len;
    int     delay_ms;
    bool    drip;
    uint8_t req[8];
    int     requests;
} R;

static uint8_t rx[32];
static int     rx_len, rx_pos;
static int64_t ready_us, drip_next_us;

int uart_write_bytes(uart_port_t port, const void *src, size_t size) {
    (void)port;
    memcpy(R.req, src, size < 8 ? size : 8);
    R.requests++;
    memcpy(rx, R.reply, (size_t)R.reply_len);
    rx_len = R.reply_len;
    rx_pos = 0;
    ready_us = s_now_us + (int64_t)R.delay_ms * 1000;
    drip_next_us = s_now_us;
    return (int)size;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length, uint32_t ticks) {
    (void)port;
    if (R.drip) {
        if (s_now_us >= drip_next_us) {
            *(uint8_t *)buf = 0x55;
            drip_next_us = s_now_us + 90000;
            return 1;
        }
        s_now_us += (int64_t)ticks * 1000;
        return 0;
    }
    int avail = rx_len - rx_pos;
    if (s_now_us < ready_us || avail <= 0) { s_now_us += (int64_t)ticks * 1000; return 0; }
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

/* A well-formed reply from `slave`: fc, byte count, data, CRC. */
static void set_reply(uint8_t slave, uint8_t fc, const uint8_t *data, int n) {
    int k = 0;
    R.reply[k++] = slave; R.reply[k++] = fc; R.reply[k++] = (uint8_t)n;
    memcpy(R.reply + k, data, (size_t)n); k += n;
    uint16_t c = crc16(R.reply, k);
    R.reply[k++] = (uint8_t)(c & 0xFF);
    R.reply[k++] = (uint8_t)(c >> 8);
    R.reply_len = k;
}

static void reset(void) { memset(&R, 0, sizeof(R)); }

/* One Ztech read of slave 4; returns ok, the frame length and the time taken. */
static bool ztech(int *len, int64_t *ms) {
    uint8_t buf[64];
    int64_t t0 = s_now_us;
    bool ok = radar_ztech_read(4, 0, buf, len);
    *ms = (s_now_us - t0) / 1000;
    return ok;
}

int main(void) {
    static const uint8_t DATA[] = {0x2f, 0x13, 0x00, 0x00};
    int len;
    int64_t ms;

    /* [1] The guide's field frame comes back unchanged, and the request is the
     *     Ztech read: FC03, 2 registers from 0. */
    reset();
    set_reply(4, 0x03, DATA, 4);
    bool ok = ztech(&len, &ms);
    {
        static const uint8_t WANT[] = {0x04, 0x03, 0x04, 0x2f, 0x13, 0x00, 0x00, 0x56, 0x22};
        CHECK(R.reply_len == 9 && memcmp(R.reply, WANT, 9) == 0, "[1] sim frame differs from the guide's");
        static const uint8_t REQ[] = {0x04, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x5e};
        CHECK(memcmp(R.req, REQ, 8) == 0, "[1] Ztech request is not 04 03 00 00 00 02 c4 5e");
    }
    CHECK(ok && len == 9, "[1] prompt reply: ok=%d len=%d", ok, len);
    CHECK(ms < 50, "[1] prompt reply took %lld ms — no fixed wait expected", (long long)ms);

    /* [2] A slow radar answering 900 ms after the request is read. */
    reset();
    set_reply(4, 0x03, DATA, 4);
    R.delay_ms = 900;
    ok = ztech(&len, &ms);
    CHECK(ok && len == 9, "[2] 900 ms reply: ok=%d len=%d", ok, len);

    /* [3] 1100 ms is past the 1 s window: no reply, and the attempt costs
     *     ~1 s, not more. */
    reset();
    set_reply(4, 0x03, DATA, 4);
    R.delay_ms = 1100;
    g_wdt_feeds = 0;
    ok = ztech(&len, &ms);
    CHECK(!ok && len == 0, "[3] 1100 ms reply must count as none: ok=%d len=%d", ok, len);
    CHECK(ms >= MODBUS_REPLY_TIMEOUT_MS && ms <= MODBUS_REPLY_TIMEOUT_MS + 20,
          "[3] silent attempt took %lld ms, want ~%d", (long long)ms, MODBUS_REPLY_TIMEOUT_MS);
    CHECK(g_wdt_feeds > 0, "[3] the watchdog was not fed while waiting");

    /* [4] A Modbus exception (5 bytes) ends on the silence gap, not the window. */
    reset();
    {
        uint8_t ex[] = {0x04, 0x83, 0x02, 0, 0};
        uint16_t c = crc16(ex, 3);
        ex[3] = (uint8_t)(c & 0xFF); ex[4] = (uint8_t)(c >> 8);
        memcpy(R.reply, ex, 5); R.reply_len = 5;
    }
    ok = ztech(&len, &ms);
    CHECK(!ok, "[4] an exception reply must not pass");
    CHECK(ms <= MODBUS_BYTE_GAP_MS + 20, "[4] exception read took %lld ms, want ~%d", (long long)ms, MODBUS_BYTE_GAP_MS);

    /* [5] A 4-register reply (13 bytes) to our 2-register read is cut at 9
     *     and fails its CRC — behaviour unchanged by the timing work. */
    reset();
    {
        static const uint8_t D8[] = {0x2f, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        set_reply(4, 0x03, D8, 8);
    }
    ok = ztech(&len, &ms);
    CHECK(!ok && len == 0, "[5] a 13-byte reply must not pass: ok=%d len=%d", ok, len);

    /* [6] Replies from the wrong slave are refused. */
    reset();
    set_reply(5, 0x03, DATA, 4);
    ok = ztech(&len, &ms);
    CHECK(!ok, "[6] slave 5's reply accepted for slave 4");

    /* [7] Noise trickling in faster than the silence gap is cut off at
     *     2 x MODBUS_REPLY_TIMEOUT_MS. */
    reset();
    R.drip = true;
    ok = ztech(&len, &ms);
    CHECK(!ok, "[7] noise must not pass");
    CHECK(ms <= 2 * MODBUS_REPLY_TIMEOUT_MS + 20, "[7] noise held the read %lld ms, cap %d",
          (long long)ms, 2 * MODBUS_REPLY_TIMEOUT_MS);

    /* [8] Pantech: FC04 from its own slave. */
    reset();
    set_reply(5, 0x04, DATA, 4);
    {
        uint8_t buf[64];
        ok = radar_pantech_read(5, 0, buf, &len);
        static const uint8_t REQ[] = {0x05, 0x04, 0x00, 0x00, 0x00, 0x02, 0x70, 0x4f};
        CHECK(ok && len == 9 && memcmp(R.req, REQ, 8) == 0, "[8] Pantech read: ok=%d len=%d", ok, len);
    }

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
