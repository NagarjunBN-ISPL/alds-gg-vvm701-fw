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
 * This file is a small driver framework. EVERY model now uses the RAW path:
 * one Modbus read per poll, the raw 16-bit register values are shipped to the
 * cloud as-is (as a decimal "registers" array plus named "alarms"/"status"/…
 * slices) and the cloud owns the register map and ALL scaling/bit-decoding.
 * The firmware never interprets a genset register. Nothing here decodes on
 * device any more — the previous per-model scaling/alarm-table decoder was
 * removed once the last model (GC1111) moved to raw.
 *
 * Pieces: Modbus RTU transport (crc16 / modbus_request), the single raw reader
 * genset_read_raw() driven by a static GensetRawSlice_t[] per model, the driver
 * table (g_drivers) mapping model id -> read function, and ONE serialiser
 * (genset_build_payload -> genset_build_raw_payload). Adding a model = write a
 * genset_read_raw() wrapper (fc/start/count + slice list) + one g_drivers row;
 * genset.h, the serialiser and main.c are untouched.
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
 * response in rx (length in *rx_len). The only caller, genset_read_raw(),
 * passes a 260-byte rx — enough for the largest read in use (SGC120: 79
 * registers = 5 + 158 = 163 bytes) and for the 125-register Modbus maximum. */
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

/* ══════════════════════════════════════════════════════════════════════════
 * Raw single-command path (cloud-side decode)
 * ──────────────────────────────────────────────────────────────────────────
 * One Modbus read, no scaling: the response's raw 16-bit register values are
 * stored in out->raw_regs[] and later emitted verbatim as a decimal array by
 * genset_build_payload(). The cloud owns the register map and all scaling, so
 * a model migrates to this path with just a driver row — no firmware knowledge
 * of what each register means, only which contiguous block to read and how to
 * slice it into the documented "registers" + "status" fields.
 *
 * fc/start/count       — the single Modbus command (start/count are protocol
 *                        register offsets, matching the vendor docs).
 * slices/slice_count   — how to name-slice the response into JSON fields.
 * Every slice index is relative to raw_regs[0] == register at offset `start`,
 * i.e. raw_regs[k] holds protocol offset (start + k). ═══════════════════════ */
static bool genset_read_raw(uint8_t slave, uint8_t fc, uint16_t start, uint16_t count,
                            const GensetRawSlice_t *slices, int slice_count,
                            GensetData_t *out) {
    uint8_t rx[260];   /* holds up to a 125-register (255-byte) response */
    int rx_len = 0;
    if (!modbus_request(slave, fc, start, count, rx, sizeof(rx), &rx_len))
        return false;

    /* modbus_request already validated CRC + slave/FC echo + exception bit, so
     * rx is a well-formed reply. rx[2] is the byte count; derive the register
     * count from it and sanity-check it against what we asked for and against
     * the buffer we parse out of. */
    int n = rx[2] / 2;
    if (n <= 0 || n > GENSET_RAW_MAX_REGS || n != (int)count || rx_len < 3 + 2 * n) {
        ESP_LOGW(TAG, "raw read: unexpected length (bytecount=%d n=%d count=%u rx_len=%d)",
                 rx[2], n, (unsigned)count, rx_len);
        return false;
    }
    for (int i = 0; i < n; i++)
        out->raw_regs[i] = ((uint16_t)rx[3 + 2 * i] << 8) | rx[4 + 2 * i];

    out->raw_reg_count   = n;
    out->raw_slices      = slices;
    out->raw_slice_count = slice_count;
    out->is_raw = true;
    out->valid  = true;
    return true;
}

/* Ashok Leyland GC1201/1202/1203/GC902 (Sedemac GC1200 family, "Nel Botl") —
 * one and the same register map for all four. Single FC04 read of input-register
 * offsets 1..67: measurements (1..36) + the DG status word (offset 67). The 30
 * registers in between (37..66) are read only to keep it a single command and
 * are not emitted. Published as {"registers":[1..36],"status":<67>}, no
 * firmware scaling — the cloud converts (per the Ashok Leyland protocol doc). */
static const GensetRawSlice_t k_gc1200_slices[] = {
    { "registers", 0,  36, true  },   /* offsets 1..36 = measurement block */
    { "status",    66, 1,  false },   /* offset 67     = DG status word     */
};
static bool sedemac_read_raw_gc1200(uint8_t slave, GensetData_t *out) {
    return genset_read_raw(slave, 0x04, /*start*/ 1, /*count*/ 67,
                           k_gc1200_slices,
                           sizeof(k_gc1200_slices) / sizeof(k_gc1200_slices[0]), out);
}

/* Sedemac GC1211 — same measurement map as GC1200 (offsets 1..36) plus the
 * alarm block and command status the GC1211 protocol doc adds. Single FC04 read
 * of offsets 1..68:
 *   offsets 1..36  → "registers"     (measurements, identical to GC1200)
 *   offsets 57..66 → "alarms"        (alarm regs 1..10; doc shows the first 3)
 *   offset  67     → "status"        (DG status word: 0x2000 off / 0xC000 on)
 *   offset  68     → "commandStatus" (0 none / 1 started / 2 stopped / ...)
 * Offsets 37..56 are read only to span the block in one command. All bit/scale
 * decoding is cloud-side — the firmware ships raw register integers only. */
static const GensetRawSlice_t k_gc1211_slices[] = {
    { "registers",     0,  36, true  },   /* offsets 1..36  = measurements     */
    { "alarms",        56, 10, true  },   /* offsets 57..66 = alarm regs 1..10  */
    { "status",        66, 1,  false },   /* offset 67      = DG status word    */
    { "commandStatus", 67, 1,  false },   /* offset 68      = command status    */
};
static bool sedemac_read_raw_gc1211(uint8_t slave, GensetData_t *out) {
    return genset_read_raw(slave, 0x04, /*start*/ 1, /*count*/ 68,
                           k_gc1211_slices,
                           sizeof(k_gc1211_slices) / sizeof(k_gc1211_slices[0]), out);
}

/* Sedemac GC902 — its OWN map, DISTINCT from the Ashok Leyland GC1201/02/03
 * despite the same GenControl family name: measurements run to offset 46, the
 * alarm block starts at 58 (not 57), and the DG status word is at 68 (not 67).
 * Single FC04 read of offsets 0..69:
 *   0..46  → "registers"     (0 protocol rev, 1 oil pressure, .. 46 mains L3-L1)
 *   58..66 → "alarms"        (Alarm 1..9)
 *   67     → "auxDiag"       (digital input/output state bitfield)
 *   68     → "status"        (DG status word)
 *   69     → "commandStatus" (0 none / 1 started / 2 stopped / ...)
 * Offsets 47..57 are reserved, read only to span the block. Cloud decodes. */
static const GensetRawSlice_t k_gc902_slices[] = {
    { "registers",     0,  47, true  },   /* offsets 0..46  = measurements  */
    { "alarms",        58, 9,  true  },   /* offsets 58..66 = Alarm 1..9     */
    { "auxDiag",       67, 1,  false },   /* offset 67      = DI/DO states   */
    { "status",        68, 1,  false },   /* offset 68      = DG status word */
    { "commandStatus", 69, 1,  false },   /* offset 69      = command status */
};
static bool sedemac_read_raw_gc902(uint8_t slave, GensetData_t *out) {
    return genset_read_raw(slave, 0x04, /*start*/ 0, /*count*/ 70,
                           k_gc902_slices,
                           sizeof(k_gc902_slices) / sizeof(k_gc902_slices[0]), out);
}

/* DEIF SGC 120 Mk II — FC03 (holding registers), one read of offsets 0..78:
 *   0..61  → "registers" (scattered measurements; cloud maps by index)
 *   65..76 → "alarms"    (12 alarm registers)
 *   78     → "status"    (DG status bitfield)
 * No command-status register on this controller. Offsets 62..64/77 are read
 * only to span the block. Cloud decodes. NOTE: Read B (regs 65..78) was never
 * captured on hardware per the DEIF decode doc — verify on a live unit. */
static const GensetRawSlice_t k_sgc120_slices[] = {
    { "registers", 0,  62, true  },   /* offsets 0..61  = measurements */
    { "alarms",    65, 12, true  },   /* offsets 65..76 = alarm block  */
    { "status",    78, 1,  false },   /* offset 78      = status word  */
};
static bool sgc120_read_raw(uint8_t slave, GensetData_t *out) {
    return genset_read_raw(slave, 0x03, /*start*/ 0, /*count*/ 79,
                           k_sgc120_slices,
                           sizeof(k_sgc120_slices) / sizeof(k_sgc120_slices[0]), out);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Driver table — add a model here (plus its GENSET_MODEL_* id in config.h and
 * the setconfig validation cap) and nothing else changes.
 * ══════════════════════════════════════════════════════════════════════════ */
typedef bool (*genset_read_fn)(uint8_t slave, GensetData_t *out);

typedef struct {
    uint8_t        model_id;
    const char    *name;
    genset_read_fn read;
} GensetDriver_t;

static const GensetDriver_t g_drivers[] = {
    { GENSET_MODEL_GC1211, "Sedemac GC1211 (raw)",     sedemac_read_raw_gc1211 },
    /* GC1111 field units answer the same 1..68 map as GC1211 (confirmed against
     * hardware) — NOT the non-contiguous 1024+/39425+ map in the GC1111 manual,
     * which cannot be read in one command. So it shares GC1211's raw driver. */
    { GENSET_MODEL_GC1111, "Sedemac GC1111 (raw, GC1211 map)", sedemac_read_raw_gc1211 },
    { GENSET_MODEL_SGC120, "DEIF SGC 120 Mk II (raw)", sgc120_read_raw         },
    { GENSET_MODEL_GC1201, "Ashok Leyland GC1201 (raw)", sedemac_read_raw_gc1200 },
    { GENSET_MODEL_GC1202, "Ashok Leyland GC1202 (raw)", sedemac_read_raw_gc1200 },
    { GENSET_MODEL_GC1203, "Ashok Leyland GC1203 (raw)", sedemac_read_raw_gc1200 },
    { GENSET_MODEL_GC902,  "Sedemac GC902 (raw)",        sedemac_read_raw_gc902  },
};

bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out) {
    memset(out, 0, sizeof(*out));
    for (size_t i = 0; i < sizeof(g_drivers) / sizeof(g_drivers[0]); i++) {
        if (g_drivers[i].model_id == model)
            return g_drivers[i].read(slave, out);
    }
    ESP_LOGW(TAG, "Unknown genset model %u", (unsigned)model);
    return false;
}

/* ══════════════════════════════════════════════════════════════════════════
 * JSON serialiser — model-agnostic. Emits the raw register values verbatim per
 * the driver's slice list; it never interprets a genset register.
 * ══════════════════════════════════════════════════════════════════════════ */

/* snprintf returns the length it WANTED to write, not what actually fit. If a
 * section truncates, `pos` sails past buf_size and the next call's
 * (buf_size - pos) wraps to a huge size_t → out-of-bounds write. Clamp after
 * every append so truncation degrades to a cut-off payload, not corruption. */
static inline int clamp_pos(int pos, size_t buf_size) {
    if (pos < 0) return 0;   /* snprintf returns <0 on encoding error — never let
                              * `pos` go negative, or buf+pos / buf_size-pos below
                              * would address outside the buffer. */
    return (pos > (int)buf_size - 1) ? (int)buf_size - 1 : pos;
}

/* Raw payload for the single-command models (is_raw): each configured slice
 * becomes one JSON field — an array or a scalar of raw decimal register values
 * straight from raw_regs[], with NO scaling; the cloud owns the register map
 * and converts. The chained snprintf + clamp_pos discipline keeps a truncating
 * payload from corrupting memory. Worst case (GC1211: 36 + 10 numbers + 2
 * scalars) is well under both buf_size and the modem ceiling. An out-of-range
 * slice index (shouldn't happen — slices are static per model) emits 0 rather
 * than reading past raw_regs[]. */
static void genset_build_raw_payload(const GensetData_t *d,
                                     const char *hw_id, const char *ts,
                                     char *buf, size_t buf_size) {
    int pos = 0;
    pos += snprintf(buf + pos, buf_size - pos,
                    "{\"HW_ID\":\"%s\",\"genset\":{", hw_id);
    pos = clamp_pos(pos, buf_size);

    for (int s = 0; s < d->raw_slice_count; s++) {
        const GensetRawSlice_t *sl = &d->raw_slices[s];
        if (s > 0) {
            pos += snprintf(buf + pos, buf_size - pos, ",");
            pos = clamp_pos(pos, buf_size);
        }
        if (sl->as_array) {
            pos += snprintf(buf + pos, buf_size - pos, "\"%s\":[", sl->key);
            pos = clamp_pos(pos, buf_size);
            for (int i = 0; i < sl->count; i++) {
                int idx = sl->start + i;
                unsigned v = (idx >= 0 && idx < d->raw_reg_count) ? d->raw_regs[idx] : 0;
                pos += snprintf(buf + pos, buf_size - pos, i ? ",%u" : "%u", v);
                pos = clamp_pos(pos, buf_size);
            }
            pos += snprintf(buf + pos, buf_size - pos, "]");
            pos = clamp_pos(pos, buf_size);
        } else {
            int idx = sl->start;
            unsigned v = (idx >= 0 && idx < d->raw_reg_count) ? d->raw_regs[idx] : 0;
            pos += snprintf(buf + pos, buf_size - pos, "\"%s\":%u", sl->key, v);
            pos = clamp_pos(pos, buf_size);
        }
    }

    pos += snprintf(buf + pos, buf_size - pos, "},\"TS\":\"%s\"}", ts);
    pos = clamp_pos(pos, buf_size);

    /* Fail-safe: a truncated or over-ceiling payload is broken JSON, so fall
     * back to the guaranteed-valid 404 shape. */
    if (pos >= (int)buf_size - 2 || pos > MQTT_MODEM_PAYLOAD_SAFE_MAX) {
        ESP_LOGW(TAG, "raw genset payload %d/%zu bytes (ceiling %d) — publishing 404 instead",
                 pos, buf_size, MQTT_MODEM_PAYLOAD_SAFE_MAX);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
    }
}

void genset_build_payload(const GensetData_t *d,
                          const char *hw_id, const char *ts,
                          char *buf, size_t buf_size) {
    if (!d->valid) {
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
        return;
    }

    genset_build_raw_payload(d, hw_id, ts, buf, buf_size);
}
