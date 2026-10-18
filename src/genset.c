#include "genset.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "genset";

/* ══════════════════════════════════════════════════════════════════════════
 * Genset polling, per the DG integration spec. Each model defines up to three
 * Modbus reads — a measurement block ("registers"), the alarm registers
 * ("Alarms", Mahindra only) and a DG status block ("status"). Each reply is
 * validated, then shipped to the cloud AS RECEIVED — the complete frame,
 * slave + FC + byte count + data + CRC, nothing removed — as a comma-separated
 * hex string in its OWN MQTT message, the same as radar and dispenser frames.
 * The firmware never interprets a genset register; the cloud owns every
 * register map and scaling.
 *
 * Pieces: Modbus RTU transport (crc16 / modbus_request), a data-driven model
 * table (g_models), the read helper (read_block), and the three serialisers.
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
 * response in rx (length in *rx_len). Timing is the shared Modbus rule in
 * config.h: the DG gets MODBUS_REPLY_TIMEOUT_MS to start answering, then the
 * read ends at the expected length, after MODBUS_BYTE_GAP_MS of silence, or at
 * the hard cap. (It used to be one 500 ms window for the whole reply, which
 * SGC120's 161-byte reply at 4800 baud — 335 ms on the wire — nearly filled.) */
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

    int got = 0;
    int expected = 5 + count * 2; /* slave+fc+bytecount+data+CRC */
    int64_t t0   = esp_timer_get_time();
    int64_t dl   = t0 + (int64_t)MODBUS_REPLY_TIMEOUT_MS * 1000;       /* first byte */
    int64_t hard = t0 + (int64_t)MODBUS_REPLY_TIMEOUT_MS * 2 * 1000;   /* whole reply */
    while (got < (int)rx_max) {
        int64_t now = esp_timer_get_time();
        if (now >= dl || now >= hard) break;
        /* Feed the WDT inside the read loop, matching every other UART loop in
         * the firmware. Today the arithmetic is safe (a 2 s cap per read x a
         * handful of reads/retries << 120 s), but the dispenser/radar loops
         * learned the hard way that "safe by arithmetic" silently breaks the
         * moment a timeout or retry constant is raised. This makes it safe by
         * construction instead. */
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            rx[got++] = ch;
            /* A Modbus exception reply is exactly 5 bytes (slave, fc|0x80,
             * code, CRC, CRC). Without shrinking the target the read would run
             * on to the silence gap waiting for `expected` data bytes that never
             * arrive. Detect the exception the instant the FC byte lands and
             * stop at the true length. A normal reply clears bit 7, so
             * `expected` is unchanged. */
            if (got == 2 && (rx[1] & 0x80)) expected = 5;
            if (got >= expected) break;
            dl = esp_timer_get_time() + (int64_t)MODBUS_BYTE_GAP_MS * 1000;
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

/* ── Read one block and keep the whole validated frame ────────────────────
 * modbus_request has already validated CRC + slave/FC echo + exception bit, so
 * rx is well-formed. We additionally require the byte-count field and the total
 * length to match exactly what was asked for, so a valid-CRC-but-wrong-shape
 * reply can never be forwarded as good data. Nothing is removed: the frame is
 * kept exactly as the controller sent it — slave, FC, byte count, data, CRC. */
static bool read_block(uint8_t slave, uint8_t fc, uint16_t start, uint16_t count,
                       uint8_t *out_frame, int out_max, int *out_len,
                       const char *what) {
    uint8_t rx[260];
    int rx_len = 0;
    *out_len = 0;
    if (!modbus_request(slave, fc, start, count, rx, sizeof(rx), &rx_len))
        return false;

    int bytecount = rx[2];
    if (bytecount != (int)count * 2 || rx_len != 3 + bytecount + 2 || rx_len > out_max) {
        ESP_LOGW(TAG, "%s: unexpected shape (bytecount=%d want=%d rx_len=%d)",
                 what, bytecount, (int)count * 2, rx_len);
        return false;
    }

    memcpy(out_frame, rx, (size_t)rx_len);
    *out_len = rx_len;
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Model table — the Modbus reads each controller answers, from the DG
 * integration spec (every frame CRC-verified against the documented captures,
 * re-checked 2026-09-23 against the spec's command/response table):
 *
 *   GC1211        FC04  registers 1..36 (72 B)   Alarms 57..59 (6 B)   status 67..68 (4 B)
 *   GC1111        FC03  registers 1..36 (72 B)   Alarms 57..59 (6 B)   status 67..68 (4 B)
 *   SGC120        FC03  registers 1..78 (156 B)  — single overall read; its alarm
 *                                                  (65..76) and status (78) registers
 *                                                  are inside it
 *   GC1201/02/03  FC04  registers 1..36 (72 B)   —                     status 67    (2 B)
 *   GC902         FC04  registers 1..36 (72 B)   —                     status 67    (2 B)
 *
 * Spec frames, slave 10 (the CRC here is what modbus_request computes):
 *   GC1211  0A 04 00 01 00 24 A0 AA   0A 04 00 39 00 03 61 7D   0A 04 00 43 00 02 81 64
 *   GC1111  0A 03 00 01 00 24 15 6A   0A 03 00 39 00 03 D4 BD   0A 03 00 43 00 02 34 A4
 *   Ashok   0A 04 00 01 00 24 A0 AA                             0A 04 00 43 00 01 C1 65
 *   SGC120  0A 03 00 01 00 4E 95 45
 *
 * NOTE on function codes: GC1211 uses FC04 and GC1111 uses FC03 — the two
 * differ ONLY in the function code, and the spec documents both explicitly.
 * NOTE on the Ashok Leyland rows: the spec titles them "DG 1202/03/04
 * (Bottling Plant DG)"; all three ids share one read set, so the label is all
 * that differs.
 * NOTE on GC902: the spec does not cover it; it is mapped to the Ashok Leyland
 * GC12xx read set, which the vendor register map lists as one shared family
 * map. The 2026-08-11 driver (5f7b940) instead gave it its own map (status at
 * 68, alarms from 58). Verify against real GC902 hardware before relying on it.
 *
 * al_count / st_count == 0 means the model has no such command.
 * ══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t     model_id;
    const char *name;
    uint8_t     fc;          /* Modbus function code (0x03 / 0x04) */
    uint16_t    reg_start;   /* command 1: measurement block       */
    uint16_t    reg_count;
    uint16_t    al_start;    /* command 2: alarms   (0 count = none) */
    uint16_t    al_count;
    uint16_t    st_start;    /* command 3: DG status (0 count = none) */
    uint16_t    st_count;
} GensetModel_t;

static const GensetModel_t g_models[] = {
    /* model                name                     fc    registers  Alarms   status */
    { GENSET_MODEL_GC1211, "Mahindra GC1211",       0x04, 1, 36,     57, 3,   67, 2 },
    { GENSET_MODEL_GC1111, "Mahindra GC1111",       0x03, 1, 36,     57, 3,   67, 2 },
    { GENSET_MODEL_SGC120, "SGC 120 Mk II",         0x03, 1, 78,      0, 0,    0, 0 },
    { GENSET_MODEL_GC1201, "Ashok Leyland GC1201",  0x04, 1, 36,      0, 0,   67, 1 },
    { GENSET_MODEL_GC1202, "Ashok Leyland GC1202",  0x04, 1, 36,      0, 0,   67, 1 },
    { GENSET_MODEL_GC1203, "Ashok Leyland GC1203",  0x04, 1, 36,      0, 0,   67, 1 },
    { GENSET_MODEL_GC902,  "Sedemac GC902",         0x04, 1, 36,      0, 0,   67, 1 },
};

static const GensetModel_t *find_model(uint8_t model) {
    for (size_t i = 0; i < sizeof(g_models) / sizeof(g_models[0]); i++)
        if (g_models[i].model_id == model) return &g_models[i];
    return NULL;
}

void genset_expect(uint8_t model, GensetData_t *out) {
    const GensetModel_t *mdl = find_model(model);
    out->has_alarms = mdl && mdl->al_count > 0;
    out->has_status = mdl && mdl->st_count > 0;
}

bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out) {
    memset(out, 0, sizeof(*out));

    const GensetModel_t *mdl = find_model(model);
    if (!mdl) {
        ESP_LOGW(TAG, "Unknown genset model %u", (unsigned)model);
        return false;
    }

    /* Set from the MODEL, not the read result, so the caller still knows an
     * Alarms / status message is expected even when every read fails. */
    genset_expect(model, out);

    out->reg_valid = read_block(slave, mdl->fc, mdl->reg_start, mdl->reg_count,
                                out->reg_frame, (int)sizeof(out->reg_frame),
                                &out->reg_len, "registers");

    /* The follow-up reads run in the spec's order (Alarms, then status), each
     * after an inter-command gap that gives the controller time to release
     * the bus, matching the gap modbus_request already leaves after its own
     * transmit. A failed read does not skip the next one: each is published
     * on its own, so a partial poll still delivers what it got. */
    if (out->has_alarms) {
        vTaskDelay(pdMS_TO_TICKS(50));
        out->al_valid = read_block(slave, mdl->fc, mdl->al_start, mdl->al_count,
                                   out->al_frame, (int)sizeof(out->al_frame),
                                   &out->al_len, "Alarms");
    }
    if (out->has_status) {
        vTaskDelay(pdMS_TO_TICKS(50));
        out->st_valid = read_block(slave, mdl->fc, mdl->st_start, mdl->st_count,
                                   out->st_frame, (int)sizeof(out->st_frame),
                                   &out->st_len, "status");
    }

    return out->reg_valid &&
           (!out->has_alarms || out->al_valid) &&
           (!out->has_status || out->st_valid);
}

/* ══════════════════════════════════════════════════════════════════════════
 * JSON serialisers — one message per read. The complete reply frame is emitted
 * as a comma-separated lowercase hex string, the same shape radar and
 * dispenser use.
 * ══════════════════════════════════════════════════════════════════════════ */
static void build_payload(const char *key, const uint8_t *frame, int len, bool valid,
                          const char *hw_id, const char *ts,
                          char *buf, size_t buf_size) {
    if (!valid || len <= 0) {
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"%s\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, key, ts);
        return;
    }

    /* len is bounded by the caller's buffer (GENSET_REG_FRAME_MAX at most), so
     * worst case is 161 * "xx," = 483 chars, which fits hex[]. */
    char hex[GENSET_REG_FRAME_MAX * 3 + 1];
    int  pos = 0;
    for (int i = 0; i < len && pos + 4 < (int)sizeof(hex); i++) {
        if (i > 0) hex[pos++] = ',';
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x", frame[i]);
    }
    hex[pos] = '\0';

    int n = snprintf(buf, buf_size,
                     "{\"HW_ID\":\"%s\",\"genset\":{\"%s\":\"%s\"},\"TS\":\"%s\"}",
                     hw_id, key, hex, ts);

    /* A truncated or over-ceiling payload is broken JSON — fall back to the
     * guaranteed-valid 404 shape. Even the largest frame (SGC120, 161 B → 482
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
    build_payload("registers", d->reg_frame, d->reg_len, d->reg_valid,
                  hw_id, ts, buf, buf_size);
}

/* "Alarms" with a capital A: the DG integration spec's spelling, which the
 * cloud parses verbatim. Do not "fix" it to match the lowercase keys. */
void genset_build_alarms_payload(const GensetData_t *d,
                                 const char *hw_id, const char *ts,
                                 char *buf, size_t buf_size) {
    build_payload("Alarms", d->al_frame, d->al_len, d->al_valid,
                  hw_id, ts, buf, buf_size);
}

void genset_build_status_payload(const GensetData_t *d,
                                 const char *hw_id, const char *ts,
                                 char *buf, size_t buf_size) {
    build_payload("status", d->st_frame, d->st_len, d->st_valid,
                  hw_id, ts, buf, buf_size);
}
