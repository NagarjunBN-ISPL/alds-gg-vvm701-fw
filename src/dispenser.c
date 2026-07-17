#include "dispenser.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── BlueSky command frame generation ─────────────────────────────────────
 * Reverse-engineered and verified byte-exact against the original hardcoded
 * NZ1-NZ4 frames; generated for any address so unit count can scale.
 *
 * BlueSky: F5 <addr> A2 <cmd> <checksum>, checksum = addr ^ cmd ^ 0xD7.
 * addr is a global per-nozzle RS485 address, 1-based.
 */
#define BS_CMD_STATUS 0xD5
#define BS_CMD_TOTAL  0xC5
#define BS_CMD_RSP    0xB6
#define BS_CMD_KFACT  0xE2
#define BS_CMD_RFID   0xD7

static void bluesky_frame(uint8_t addr, uint8_t cmd_byte, uint8_t *out /* 5 bytes */) {
    out[0] = 0xF5;
    out[1] = addr;
    out[2] = 0xA2;
    out[3] = cmd_byte;
    out[4] = (uint8_t)(addr ^ cmd_byte ^ 0xD7);
}

/* ── RS485 mode helper ────────────────────────────────────────────────── */
void du_rs485_set_bluesky_mode(void) {
    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(RS485_UART, &cfg);
}

static const char *TAG = "du";

/* ── Low-level RS485 transaction ─────────────────────────────────────── */
/* 2000 ms per-byte-reset timeout (timer resets each time a byte arrives),
 * PLUS an absolute cap: on a noisy line, sporadic garbage bytes keep
 * resetting the per-byte timer, and without a hard ceiling a single
 * nozzle read can stretch toward a minute — several units of that in one
 * cycle would starve the task watchdog. With the cap, a noise storm
 * degrades to "404" data instead of a WDT panic reboot. */
static int rs485_xfer(const uint8_t *cmd, size_t cmd_len,
                      uint8_t *resp, size_t resp_max) {
    /* Flush RX */
    uint8_t tmp[64];
    while (uart_read_bytes(RS485_UART, tmp, sizeof(tmp), 0) > 0);

    uart_write_bytes(RS485_UART, cmd, cmd_len);

    int got = 0;
    uint32_t t0      = (uint32_t)(esp_timer_get_time() / 1000);
    int64_t  hard_dl = esp_timer_get_time()
                       + (int64_t)RS485_RECV_TIMEOUT_MS * 3 * 1000;
    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < RS485_RECV_TIMEOUT_MS
           && esp_timer_get_time() < hard_dl
           && got < (int)resp_max) {
        /* Feed the WDT inside the loop, matching modem_at()'s pattern. The
         * hard cap above bounds this at RS485_RECV_TIMEOUT_MS*3 = 6 s, well
         * inside the 120 s WDT — but that safety is arithmetic coincidence,
         * not design: raising RS485_RECV_TIMEOUT_MS past ~40 s would silently
         * make this loop able to trip the watchdog. Feeding here makes the
         * loop safe by construction regardless of that constant. */
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            resp[got++] = ch;
            t0 = (uint32_t)(esp_timer_get_time() / 1000); /* reset on each byte */
        }
    }
    return got;
}

/* ── BlueSky response validation ──────────────────────────────────────────
 * Every confirmed BlueSky frame starts 0xF5 followed by the nozzle address,
 * so both are enforced: a response must echo the header AND the address we
 * polled, or it is treated as line noise / crosstalk and dropped.
 *
 * A prior XOR-of-all-bytes-==-0x80 checksum check was removed here: field
 * data from a real dispenser (2026-07-08) showed every single response
 * frame — both nozzles, every command type — computing XOR=0x00, never
 * 0x80. That rule was written for our own OUTGOING command frames (where
 * 0xF5^0xA2^0xD7 does equal 0x80 by construction) and had only ever been
 * assumed, not confirmed, to also hold for the device's responses. The
 * field data disproves that assumption, so the check is gone rather than
 * left in as permanent log noise around a formula that doesn't apply here. */
static bool bluesky_resp_valid(const uint8_t *buf, int len, uint8_t addr,
                                const char *what) {
    if (len < 4) {
        ESP_LOGW(TAG, "BlueSky nz%u %s: short frame (%d bytes) — dropped",
                 addr, what, len);
        return false;
    }
    if (buf[0] != 0xF5 || buf[1] != addr) {
        ESP_LOGW(TAG, "BlueSky nz%u %s: bad header/addr echo (0x%02x 0x%02x) — dropped",
                 addr, what, buf[0], buf[1]);
        return false;
    }
    return true;
}

/* Unconditional raw-bytes log for field debugging — prints exactly what came
 * back over RS485 for this command, regardless of whether it passed
 * validation, so a station visit can see real wire data without needing a
 * logic analyzer. */
static void log_raw_frame(uint8_t addr, const char *what, const uint8_t *buf, int len) {
    if (len <= 0) {
        ESP_LOGI(TAG, "BlueSky nz%u %s: no response", addr, what);
        return;
    }
    char hex[DU_RESP_MAX * 3 + 1];
    du_bytes_to_hex(buf, len, hex, sizeof(hex));
    ESP_LOGI(TAG, "BlueSky nz%u %s: [%s]", addr, what, hex);
}

/* ── BlueSky polling ──────────────────────────────────────────────────── */
static void poll_one_bluesky(uint8_t addr, DuNozzle_t *nz) {
    memset(nz, 0, sizeof(*nz));
    nz->present = false;

    uint8_t cmd[5];

    /* Status read — sent ONCE, for presence/idle reporting only. We do NOT
     * gate the data reads on the idle (0x80) byte and do NOT retry waiting
     * for idle: the totalizer/RSP/K-factor/RFID commands are issued whether
     * the nozzle reports idle or busy. `idle` is kept purely informational.
     * (Header/address validation is still applied so line noise can't fake a
     * "present" nozzle.) */
    bluesky_frame(addr, BS_CMD_STATUS, cmd);
    nz->status_len = rs485_xfer(cmd, 5, nz->status, DU_RESP_MAX);
    esp_task_wdt_reset();
    log_raw_frame(addr, "status", nz->status, nz->status_len);
    if (nz->status_len > 0 &&
        !bluesky_resp_valid(nz->status, nz->status_len, addr, "status"))
        nz->status_len = 0;
    if (nz->status_len > 0) nz->present = true;
    nz->idle = (nz->status_len >= 4 && nz->status[3] == 0x80);

    /* A nozzle that gave NO valid status reply is treated as physically
     * absent — skip its four data reads so an unpopulated slot costs one
     * timeout, not five. (This is presence detection, not the idle check:
     * a busy nozzle still answers status, so it is polled normally.) */
    if (!nz->present) return;
    vTaskDelay(pdMS_TO_TICKS(30));

    /* Data reads — attempted whenever the nozzle is present, idle OR busy.
     * An invalid frame zeroes the length, so the payload builder reports
     * "404" for that field instead of forwarding noise. */

    /* Totalizer */
    bluesky_frame(addr, BS_CMD_TOTAL, cmd);
    nz->total_len = rs485_xfer(cmd, 5, nz->total, DU_RESP_MAX);
    esp_task_wdt_reset();
    log_raw_frame(addr, "total", nz->total, nz->total_len);
    if (nz->total_len > 0 &&
        !bluesky_resp_valid(nz->total, nz->total_len, addr, "total"))
        nz->total_len = 0;
    vTaskDelay(pdMS_TO_TICKS(30));

    /* RSP */
    bluesky_frame(addr, BS_CMD_RSP, cmd);
    nz->rsp_len = rs485_xfer(cmd, 5, nz->rsp, DU_RESP_MAX);
    esp_task_wdt_reset();
    log_raw_frame(addr, "rsp", nz->rsp, nz->rsp_len);
    if (nz->rsp_len > 0 &&
        !bluesky_resp_valid(nz->rsp, nz->rsp_len, addr, "rsp"))
        nz->rsp_len = 0;
    vTaskDelay(pdMS_TO_TICKS(30));

    /* K-Factor */
    bluesky_frame(addr, BS_CMD_KFACT, cmd);
    nz->kfact_len = rs485_xfer(cmd, 5, nz->kfact, DU_RESP_MAX);
    esp_task_wdt_reset();
    log_raw_frame(addr, "kfact", nz->kfact, nz->kfact_len);
    if (nz->kfact_len > 0 &&
        !bluesky_resp_valid(nz->kfact, nz->kfact_len, addr, "kfact"))
        nz->kfact_len = 0;
    vTaskDelay(pdMS_TO_TICKS(30));

    /* RFID */
    bluesky_frame(addr, BS_CMD_RFID, cmd);
    nz->rfid_len = rs485_xfer(cmd, 5, nz->rfid, DU_RESP_MAX);
    esp_task_wdt_reset();
    log_raw_frame(addr, "rfid", nz->rfid, nz->rfid_len);
    if (nz->rfid_len > 0 &&
        !bluesky_resp_valid(nz->rfid, nz->rfid_len, addr, "rfid"))
        nz->rfid_len = 0;
}

void du_poll_bluesky_unit(int unit_idx, DuNozzle_t *out_a, DuNozzle_t *out_b) {
    uint8_t addr_a = (uint8_t)(unit_idx * 2 + 1);
    uint8_t addr_b = (uint8_t)(unit_idx * 2 + 2);
    poll_one_bluesky(addr_a, out_a);
    poll_one_bluesky(addr_b, out_b);
}

/* ── Hex helpers ──────────────────────────────────────────────────────── */
void du_bytes_to_hex(const uint8_t *data, int len, char *out, size_t out_size) {
    int pos = 0;
    for (int i = 0; i < len && pos + 4 < (int)out_size; i++) {
        if (i > 0) out[pos++] = ',';
        pos += snprintf(out + pos, out_size - pos, "%02x", data[i]);
    }
    out[pos] = '\0';
}

/* snprintf returns the length it WANTED to write, not what actually fit. If
 * a section truncates, `pos` sails past buf_size and the next call's
 * (buf_size - pos) — both promoted to size_t — wraps to a huge value, turning
 * a truncated field into an out-of-bounds write. Same pattern already fixed
 * in genset.c; applied here since this function chains snprintf the same way. */
static inline int clamp_pos(int pos, size_t buf_size) {
    return (pos > (int)buf_size - 1) ? (int)buf_size - 1 : pos;
}

/* ── JSON payload builder ─────────────────────────────────────────────── */
void du_build_payload(const DuNozzle_t *nz_a, const DuNozzle_t *nz_b,
                      int label_a, int label_b,
                      const char *hw_id, const char *ts,
                      char *buf, size_t buf_size) {
    int  pos = 0;

    pos += snprintf(buf + pos, buf_size - pos,
                    "{\"HW_ID\":\"%s\",\"nozzles\":{", hw_id);
    pos = clamp_pos(pos, buf_size);

    const DuNozzle_t *nzs[2] = {nz_a, nz_b};
    int labels[2] = {label_a, label_b};

    for (int i = 0; i < 2 && pos < (int)buf_size - 10; i++) {
        const DuNozzle_t *nz = nzs[i];
        if (i > 0) buf[pos++] = ',';

        if (!nz->present) {
            pos += snprintf(buf + pos, buf_size - pos,
                            "\"%d\":{\"data\":\"404\"}", labels[i]);
            pos = clamp_pos(pos, buf_size);
            continue;
        }

        /* All 4 fields are reported regardless of idle/busy status — any
         * field whose read got no valid response becomes "404". */
        char h_tot[128];
        char h_rsp[128];
        char h_kf [128];
        char h_rf [128];
        if (nz->total_len > 0) du_bytes_to_hex(nz->total, nz->total_len, h_tot, sizeof(h_tot));
        else strcpy(h_tot, "404");
        if (nz->rsp_len > 0)   du_bytes_to_hex(nz->rsp,   nz->rsp_len,   h_rsp, sizeof(h_rsp));
        else strcpy(h_rsp, "404");
        if (nz->kfact_len > 0) du_bytes_to_hex(nz->kfact, nz->kfact_len, h_kf,  sizeof(h_kf));
        else strcpy(h_kf, "404");
        if (nz->rfid_len > 0)  du_bytes_to_hex(nz->rfid,  nz->rfid_len,  h_rf,  sizeof(h_rf));
        else strcpy(h_rf, "404");
        pos += snprintf(buf + pos, buf_size - pos,
                        "\"%d\":{\"data\":\"%s\",\"rsp\":\"%s\","
                        "\"kfact\":\"%s\",\"rfid\":\"%s\"}",
                        labels[i], h_tot, h_rsp, h_kf, h_rf);
        pos = clamp_pos(pos, buf_size);
    }

    pos += snprintf(buf + pos, buf_size - pos,
                    "},\"TS\":\"%s\"}", ts);
    pos = clamp_pos(pos, buf_size);

    /* clamp_pos prevents memory corruption but not broken JSON syntax — a
     * truncated payload (missing closing braces, possibly mid-string) would
     * be silently dropped by the cloud parser. Landing within 2 bytes of the
     * ceiling on a buffer sized with ~2x headroom over the worst case is
     * itself a strong "something's wrong" signal, so fall back to a
     * guaranteed-valid 404 shape for both nozzles instead. */
    if (pos >= (int)buf_size - 2) {
        ESP_LOGW(TAG, "dispenser payload truncated at %d/%zu bytes — publishing 404 instead", pos, buf_size);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"nozzles\":{\"%d\":{\"data\":\"404\"},\"%d\":{\"data\":\"404\"}},\"TS\":\"%s\"}",
                 hw_id, label_a, label_b, ts);
    }
}

/* ── Remote command execution ─────────────────────────────────────────── */
/* Parse comma-sep hex string into byte array; returns length */
static bool is_hex_ch(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

static int hex_str_to_bytes(const char *s, uint8_t *out, size_t out_max) {
    int n = 0;
    while (*s && n < (int)out_max) {
        /* Skip any separator/junk (spaces, commas, trailing CR/LF from the
         * MQTT publisher) — previously a trailing newline was fed to strtol
         * and became a bogus 0x00 byte appended to the RS485 frame. */
        while (*s && !is_hex_ch(*s)) s++;
        if (!*s) break;
        if (!is_hex_ch(s[1])) break;   /* need a full 2-char byte */
        char tmp[3] = {s[0], s[1], '\0'};
        out[n++] = (uint8_t)strtol(tmp, NULL, 16);
        s += 2;
    }
    return n;
}

bool du_exec_bluesky_rsp(const char *cmd_hex, char *resp_buf, size_t resp_size) {
    uint8_t cmd[32];
    int cmd_len = hex_str_to_bytes(cmd_hex, cmd, sizeof(cmd));
    if (cmd_len == 0) return false;

    uint8_t resp[DU_RESP_MAX];
    int rlen = rs485_xfer(cmd, cmd_len, resp, DU_RESP_MAX);
    if (rlen == 0) return false;
    du_bytes_to_hex(resp, rlen, resp_buf, resp_size);
    return true;
}

bool du_exec_bluesky_kfactor(const char *cmd_hex, char *resp_buf, size_t resp_size) {
    return du_exec_bluesky_rsp(cmd_hex, resp_buf, resp_size);
}
