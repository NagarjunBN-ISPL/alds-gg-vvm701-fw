#include "dispenser.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── Command frame generation ─────────────────────────────────────────────
 * Both tables below were originally 4 hardcoded frames (NZ1-NZ4, i.e. 2
 * fixed DU slots). Reverse-engineered and verified byte-exact against all
 * 4 original frames; now generated for any address so unit count can scale.
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

/* Yenen: STX '0' '8' <du_addr 2 ASCII digits> <register 2 hex ASCII chars>
 * ETX <checksum 2 bytes>, where checksum = XOR of all 8 preceding bytes,
 * split into nibbles each OR'd with 0x30 (high nibble first). Register =
 * (nozzle_b ? 0x5A : 0x0A) + (du_addr-1)*0x10. du_addr is 1-based. */
static void yenen_frame(int du_addr, bool nozzle_b, uint8_t *out /* 10 bytes */) {
    static const char hexch[] = "0123456789ABCDEF";
    uint8_t reg = (uint8_t)((nozzle_b ? 0x5A : 0x0A) + (du_addr - 1) * 0x10);

    out[0] = 0x02;                                    /* STX */
    out[1] = '0';
    out[2] = '8';
    out[3] = (uint8_t)('0' + (du_addr / 10));
    out[4] = (uint8_t)('0' + (du_addr % 10));
    out[5] = (uint8_t)hexch[(reg >> 4) & 0xF];
    out[6] = (uint8_t)hexch[reg & 0xF];
    out[7] = 0x03;                                    /* ETX */

    uint8_t x = 0;
    for (int i = 0; i < 8; i++) x ^= out[i];
    out[8] = (uint8_t)(0x30 | (x >> 4));
    out[9] = (uint8_t)(0x30 | (x & 0x0F));
}

/* ── RS485 mode helpers ───────────────────────────────────────────────── */
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

void du_rs485_set_yenen_mode(void) {
    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(RS485_UART, &cfg);
}

/* ── Low-level RS485 transaction ─────────────────────────────────────── */
/* 2000 ms per-byte-reset timeout (timer resets each time a byte arrives) */
static int rs485_xfer(const uint8_t *cmd, size_t cmd_len,
                      uint8_t *resp, size_t resp_max) {
    /* Flush RX */
    uint8_t tmp[64];
    while (uart_read_bytes(RS485_UART, tmp, sizeof(tmp), 0) > 0);

    uart_write_bytes(RS485_UART, cmd, cmd_len);

    int got = 0;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < RS485_RECV_TIMEOUT_MS
           && got < (int)resp_max) {
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            resp[got++] = ch;
            t0 = (uint32_t)(esp_timer_get_time() / 1000); /* reset on each byte */
        }
    }
    return got;
}

/* ── BlueSky polling ──────────────────────────────────────────────────── */
static void poll_one_bluesky(uint8_t addr, DuNozzle_t *nz) {
    memset(nz, 0, sizeof(*nz));
    nz->present = false;

    uint8_t cmd[5];

    /* Status check — up to 3 retries waiting for idle (buf[3] == 0x80) */
    bool idle = false;
    for (int retry = 0; retry < 3; retry++) {
        bluesky_frame(addr, BS_CMD_STATUS, cmd);
        nz->status_len = rs485_xfer(cmd, 5, nz->status, DU_RESP_MAX);
        if (nz->status_len > 0) {
            nz->present = true;
            if (nz->status_len >= 4 && nz->status[3] == 0x80) {
                idle = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    nz->idle = idle;
    if (!nz->present) return;
    if (!idle) return; /* Busy — payload builder will report "404" */

    /* Totalizer */
    bluesky_frame(addr, BS_CMD_TOTAL, cmd);
    nz->total_len = rs485_xfer(cmd, 5, nz->total, DU_RESP_MAX);
    vTaskDelay(pdMS_TO_TICKS(30));

    /* RSP */
    bluesky_frame(addr, BS_CMD_RSP, cmd);
    nz->rsp_len = rs485_xfer(cmd, 5, nz->rsp, DU_RESP_MAX);
    vTaskDelay(pdMS_TO_TICKS(30));

    /* K-Factor */
    bluesky_frame(addr, BS_CMD_KFACT, cmd);
    nz->kfact_len = rs485_xfer(cmd, 5, nz->kfact, DU_RESP_MAX);
    vTaskDelay(pdMS_TO_TICKS(30));

    /* RFID */
    bluesky_frame(addr, BS_CMD_RFID, cmd);
    nz->rfid_len = rs485_xfer(cmd, 5, nz->rfid, DU_RESP_MAX);
}

void du_poll_bluesky_unit(int unit_idx, DuNozzle_t *out_a, DuNozzle_t *out_b) {
    uint8_t addr_a = (uint8_t)(unit_idx * 2 + 1);
    uint8_t addr_b = (uint8_t)(unit_idx * 2 + 2);
    poll_one_bluesky(addr_a, out_a);
    poll_one_bluesky(addr_b, out_b);
}

/* ── Yenen polling ────────────────────────────────────────────────────── */
static void poll_one_yenen(int du_addr, bool nozzle_b, DuNozzle_t *nz) {
    memset(nz, 0, sizeof(*nz));
    uint8_t cmd[10];
    yenen_frame(du_addr, nozzle_b, cmd);
    nz->total_len = rs485_xfer(cmd, 10, nz->total, DU_RESP_MAX);
    nz->present   = (nz->total_len > 0);
    nz->idle      = nz->present; /* Yenen has no status check */
}

void du_poll_yenen_unit(int unit_idx, DuNozzle_t *out_a, DuNozzle_t *out_b) {
    int du_addr = unit_idx + 1;
    poll_one_yenen(du_addr, false, out_a);
    poll_one_yenen(du_addr, true,  out_b);
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

/* ── JSON payload builder ─────────────────────────────────────────────── */
void du_build_payload(const DuNozzle_t *nz_a, const DuNozzle_t *nz_b,
                      int label_a, int label_b,
                      bool is_yenen,
                      const char *hw_id, const char *ts,
                      char *buf, size_t buf_size) {
    char hex[128];
    int  pos = 0;

    pos += snprintf(buf + pos, buf_size - pos,
                    "{\"HW_ID\":\"%s\",\"nozzles\":{", hw_id);

    const DuNozzle_t *nzs[2] = {nz_a, nz_b};
    int labels[2] = {label_a, label_b};

    for (int i = 0; i < 2 && pos < (int)buf_size - 10; i++) {
        const DuNozzle_t *nz = nzs[i];
        if (i > 0) buf[pos++] = ',';

        if (!nz->present) {
            pos += snprintf(buf + pos, buf_size - pos,
                            "\"%d\":{\"data\":\"404\"}", labels[i]);
            continue;
        }

        if (is_yenen) {
            du_bytes_to_hex(nz->total, nz->total_len, hex, sizeof(hex));
            pos += snprintf(buf + pos, buf_size - pos,
                            "\"%d\":{\"data\":\"%s\"}", labels[i], hex);
            continue;
        }

        /* BlueSky busy (dispensing) — report 404, no data reads attempted */
        if (!nz->idle) {
            pos += snprintf(buf + pos, buf_size - pos,
                            "\"%d\":{\"data\":\"404\"}", labels[i]);
            continue;
        }

        /* Idle: all 4 fields — any field that got no response becomes "404" */
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
    }

    pos += snprintf(buf + pos, buf_size - pos,
                    "},\"TS\":\"%s\"}", ts);
}

/* ── Remote command execution ─────────────────────────────────────────── */
/* Parse comma-sep hex string into byte array; returns length */
static int hex_str_to_bytes(const char *s, uint8_t *out, size_t out_max) {
    int n = 0;
    while (*s && n < (int)out_max) {
        while (*s == ' ' || *s == ',') s++;
        if (!*s) break;
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
