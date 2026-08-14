#include "genset.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "genset";

/* ══════════════════════════════════════════════════════════════════════════
 * Genset polling, per the DG integration spec. Each model defines up to two
 * Modbus reads — a measurement block ("registers") and a DG status block
 * ("status") — and each read's DATA bytes are shipped to the cloud verbatim as
 * a comma-separated hex string in its OWN MQTT message. The firmware never
 * interprets a genset register; the cloud owns every register map and scaling.
 *
 * Pieces: Modbus RTU transport (crc16 / modbus_request), a data-driven model
 * table (g_models), the read helper (read_block), and the two serialisers.
 * Adding a model = add one g_models[] row. Nothing else changes.
 * ══════════════════════════════════════════════════════════════════════════ */

/* ── CRC-16/IBM (Modbus) ──────────────────────────────────────────────── */
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

/* ── Modbus RTU request / response ───────────────────────────────────────
 * Sends an FC03/FC04 read and returns the raw, CRC + slave/FC-echo validated
 * response in rx (length in *rx_len). */
#define MB_TIMEOUT_MS 500

static bool modbus_request(uint8_t slave, uint8_t fc,
                            uint16_t addr, uint16_t count,
                            uint8_t *rx, size_t rx_max, int *rx_len) {
    uint8_t req[8];
    req[0] = slave;
    req[1] = fc;
    req[2] = addr >> 8;
    req[3] = addr & 0xFF;
    req[4] = count >> 8;
    req[5] = count & 0xFF;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    /* Flush */
    uint8_t tmp[64];
    while (uart_read_bytes(RS485_UART, tmp, sizeof(tmp), 0) > 0);

    uart_write_bytes(RS485_UART, req, 8);
    vTaskDelay(pdMS_TO_TICKS(10)); /* inter-frame gap */

    int got = 0;
    int expected = 5 + count * 2; /* slave+fc+bytecount+data+CRC */
    int64_t dl = esp_timer_get_time() + (int64_t)MB_TIMEOUT_MS * 1000;
    while (esp_timer_get_time() < dl && got < (int)rx_max) {
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            rx[got++] = ch;
            if (got >= expected) break;
        }
    }
    *rx_len = got;
    if (got < 5) return false;

    /* Verify CRC */
    uint16_t calc = crc16(rx, got - 2);
    uint16_t recv = rx[got - 2] | ((uint16_t)rx[got - 1] << 8);
    if (calc != recv) {
        ESP_LOGW(TAG, "CRC mismatch: calc=0x%04x recv=0x%04x", calc, recv);
        return false;
    }
    /* Verify the response echoes OUR slave address and function code.
     * CRC alone is NOT enough on this bus: the radars share the same RS485
     * line (Ztech slaves 4.., Pantech 6..), and a late radar reply has a
     * perfectly valid CRC — without this check it could be accepted as
     * genset register data. */
    if (rx[0] != slave) {
        ESP_LOGW(TAG, "Wrong slave echo: got 0x%02x want 0x%02x (bus crosstalk?)",
                 rx[0], slave);
        return false;
    }
    /* Check FC (bit 7 set = exception) */
    if (rx[1] & 0x80) { ESP_LOGW(TAG, "Modbus exception 0x%02x", rx[2]); return false; }
    if (rx[1] != fc) {
        ESP_LOGW(TAG, "Wrong FC echo: got 0x%02x want 0x%02x", rx[1], fc);
        return false;
    }
    return true;
}

/* ── Read one block and keep only its DATA bytes ──────────────────────────
 * modbus_request has already validated CRC + slave/FC echo + exception bit, so
 * rx is well-formed. We additionally require the byte-count field and the total
 * length to match exactly what was asked for, so a valid-CRC-but-wrong-shape
 * reply can never be forwarded as good data. The 3-byte header and 2-byte CRC
 * are stripped — the cloud receives the register payload only. */
static bool read_block(uint8_t slave, uint8_t fc, uint16_t start, uint16_t count,
                       uint8_t *out_data, int out_max, int *out_len,
                       const char *what) {
    uint8_t rx[260];
    int rx_len = 0;
    if (!modbus_request(slave, fc, start, count, rx, sizeof(rx), &rx_len))
        return false;

    int bytecount = rx[2];
    if (bytecount != (int)count * 2 || rx_len != 3 + bytecount + 2 || bytecount > out_max) {
        ESP_LOGW(TAG, "%s: unexpected shape (bytecount=%d want=%d rx_len=%d)",
                 what, bytecount, (int)count * 2, rx_len);
        return false;
    }

    memcpy(out_data, rx + 3, (size_t)bytecount);
    *out_len = bytecount;
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Model table — the Modbus reads each controller answers, from the DG
 * integration spec (all frames CRC-verified against the documented captures):
 *
 *   GC1211        FC04  registers 1..36 (72 B)   status 67..68 (4 B)
 *   GC1111        FC03  registers 1..36 (72 B)   status 67..68 (4 B)
 *   SGC120        FC03  registers 1..78 (156 B)  — single overall read
 *   GC1201/02/03  FC04  registers 1..36 (72 B)   status 67    (2 B)
 *   GC902         FC04  registers 1..36 (72 B)   status 67    (2 B)
 *
 * NOTE on function codes: GC1211 uses FC04 and GC1111 uses FC03 — the two
 * differ ONLY in the function code, and the spec documents both explicitly.
 * NOTE on GC902: the spec does not cover it; it is mapped to the Ashok Leyland
 * GC12xx read set, which the vendor register map lists as one shared family
 * map. Verify against real GC902 hardware before relying on it.
 *
 * st_count == 0 means the model has no status command.
 * ══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t     model_id;
    const char *name;
    uint8_t     fc;          /* Modbus function code (0x03 / 0x04) */
    uint16_t    reg_start;   /* command 1: measurement block       */
    uint16_t    reg_count;
    uint16_t    st_start;    /* command 2: DG status (0 count = none) */
    uint16_t    st_count;
} GensetModel_t;

static const GensetModel_t g_models[] = {
    { GENSET_MODEL_GC1211, "Mahindra GC1211",       0x04, 1, 36, 67, 2 },
    { GENSET_MODEL_GC1111, "Mahindra GC1111",       0x03, 1, 36, 67, 2 },
    { GENSET_MODEL_SGC120, "SGC 120 Mk II",         0x03, 1, 78,  0, 0 },
    { GENSET_MODEL_GC1201, "Ashok Leyland GC1201",  0x04, 1, 36, 67, 1 },
    { GENSET_MODEL_GC1202, "Ashok Leyland GC1202",  0x04, 1, 36, 67, 1 },
    { GENSET_MODEL_GC1203, "Ashok Leyland GC1203",  0x04, 1, 36, 67, 1 },
    { GENSET_MODEL_GC902,  "Sedemac GC902",         0x04, 1, 36, 67, 1 },
};

bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out) {
    memset(out, 0, sizeof(*out));

    const GensetModel_t *mdl = NULL;
    for (size_t i = 0; i < sizeof(g_models) / sizeof(g_models[0]); i++) {
        if (g_models[i].model_id == model) { mdl = &g_models[i]; break; }
    }
    if (!mdl) {
        ESP_LOGW(TAG, "Unknown genset model %u", (unsigned)model);
        return false;
    }

    /* Set from the MODEL, not the read result, so the caller still knows a
     * status message is expected even when every read fails. */
    out->has_status = (mdl->st_count > 0);

    out->reg_valid = read_block(slave, mdl->fc, mdl->reg_start, mdl->reg_count,
                                out->reg_data, (int)sizeof(out->reg_data),
                                &out->reg_len, "registers");

    if (out->has_status) {
        /* Inter-command gap: give the controller time to release the bus before
         * the second read, matching the gap modbus_request already leaves after
         * its own transmit. */
        vTaskDelay(pdMS_TO_TICKS(50));
        out->st_valid = read_block(slave, mdl->fc, mdl->st_start, mdl->st_count,
                                   out->st_data, (int)sizeof(out->st_data),
                                   &out->st_len, "status");
    }

    return out->reg_valid && (!out->has_status || out->st_valid);
}

/* ══════════════════════════════════════════════════════════════════════════
 * JSON serialisers — one message per read. Data is emitted as a comma-separated
 * lowercase hex string, the same shape radar and dispenser use.
 * ══════════════════════════════════════════════════════════════════════════ */
static void build_payload(const char *key, const uint8_t *data, int len, bool valid,
                          const char *hw_id, const char *ts,
                          char *buf, size_t buf_size) {
    if (!valid || len <= 0) {
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"%s\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, key, ts);
        return;
    }

    /* len is bounded by the caller's buffer (GENSET_REG_DATA_MAX at most), so
     * worst case is 160 * "xx," = 480 chars, which fits hex[]. */
    char hex[GENSET_REG_DATA_MAX * 3 + 1];
    int  pos = 0;
    for (int i = 0; i < len && pos + 4 < (int)sizeof(hex); i++) {
        if (i > 0) hex[pos++] = ',';
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x", data[i]);
    }
    hex[pos] = '\0';

    int n = snprintf(buf, buf_size,
                     "{\"HW_ID\":\"%s\",\"genset\":{\"%s\":\"%s\"},\"TS\":\"%s\"}",
                     hw_id, key, hex, ts);

    /* A truncated or over-ceiling payload is broken JSON — fall back to the
     * guaranteed-valid 404 shape. Even the largest block (SGC120, 156 B → ~467
     * hex chars) sits far under both buf_size and the modem ceiling, so this is
     * belt-and-suspenders. */
    if (n < 0 || n >= (int)buf_size || n > MQTT_MODEM_PAYLOAD_SAFE_MAX) {
        ESP_LOGW(TAG, "genset %s payload %d bytes over limit (buf %zu, ceiling %d) — publishing 404 instead",
                 key, n, buf_size, MQTT_MODEM_PAYLOAD_SAFE_MAX);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"%s\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, key, ts);
    }
}

void genset_build_registers_payload(const GensetData_t *d,
                                    const char *hw_id, const char *ts,
                                    char *buf, size_t buf_size) {
    build_payload("registers", d->reg_data, d->reg_len, d->reg_valid,
                  hw_id, ts, buf, buf_size);
}

void genset_build_status_payload(const GensetData_t *d,
                                 const char *hw_id, const char *ts,
                                 char *buf, size_t buf_size) {
    build_payload("status", d->st_data, d->st_len, d->st_valid,
                  hw_id, ts, buf, buf_size);
}
