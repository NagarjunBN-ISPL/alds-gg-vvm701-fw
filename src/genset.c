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
 * This file is organised as a small driver framework so new genset
 * controllers drop in cleanly:
 *
 *   1. Shared Modbus RTU transport (crc16 / modbus_request / parse_regs).
 *   2. A shared nibble-alarm decoder (decode_nibble_alarms).
 *   3. One read function per model that fills the normalised GensetData_t
 *      with already-scaled physical values (sedemac_read_*, sgc120_read).
 *   4. A driver table (g_drivers) mapping model id -> read function.
 *   5. ONE model-agnostic JSON serialiser (genset_build_payload).
 *
 * Adding a model = write a read function + add one row to g_drivers. The
 * serialiser, the JSON schema and main.c are untouched.
 * ══════════════════════════════════════════════════════════════════════════ */

/* Alarm state strings, indexed by the raw nibble value (0..5). Only 2..5 are
 * ever emitted — Disabled/Inactive are dropped at decode time. */
static const char * const g_alarm_state_names[] =
    {"Disabled","Inactive","Warning","Shutdown","ElecTrip","Notification"};

/* ── Sedemac GC1211/GC1111 alarm table (10 registers × 4 nibbles, MSN→LSN) ─
 * Order verified against a live GC1211 wire capture (SEDMAC decode doc):
 * register 57 = 0x1110 decodes MSN→LSN as RadiatorWaterLevel, LowFuelLevel,
 * HighEngTemp, LOP — cross-checked against the GC1201/02/03/902 register map,
 * which independently agrees. Do NOT reorder without a fresh wire capture. */
static const char * const g_sedemac_alarm_names[10][4] = {
    {"RadiatorWaterLevel","LowFuelLevel","HighEngTemp","LOP"},
    {"FailToStop","FailToStart","OverSpeed","UnderSpeed"},
    {"GensetHighFreq","GensetLowFreq","GensetHighVolt","GensetLowVolt"},
    {"EmergencyStop","UnbalancedLoad","Overload","HighCurrent"},
    {"AirFilterMaint","FuelFilterMaint","OilFilterMaint","ChargingAltFail"},
    {"EngTempOpenCkt","OilPressOpenCkt","BattHighVolt","BattLowVolt"},
    {"Reserved","Reserved","MPULoss","FuelTheft"},
    {"AuxInputD","AuxInputC","AuxInputB","AuxInputA"},
    {"AuxInputH","AuxInputG","AuxInputF","AuxInputE"},
    {"Reserved","Reserved","Reserved","AuxInputI"}
};

/* ── DEIF SGC 120 Mk II alarm table (12 registers × 4 nibbles, MSN→LSN) ─────
 * Registers 65..76, 4 alarm nibbles each, in the order the DEIF Modbus table
 * (doc 4189341403) lists the "alarm status" parameters. The datatype column
 * confirms MSN→LSN sub-addressing (13/16..16/16 first, 1/16..4/16 last), which
 * matches decode_nibble_alarms' MSN-first walk.
 *
 * ⚠ DATASHEET-DERIVED, NOT WIRE-VERIFIED: the shared capture shows every alarm
 * register = 0x0000 (no active alarms), so the name↔nibble mapping and the
 * status-word bit positions below could not be confirmed against a live fault.
 * Trigger a known alarm on real hardware (e.g. force low oil pressure) and
 * confirm the right key lights up before trusting this in production. */
static const char * const g_sgc120_alarm_names[12][4] = {
    {"LOP","HighCoolantTemp","LowFuelLevel","WaterLevelSwitch"},        /* reg 65 */
    {"UnderSpeed","OverSpeed","FailToStart","FailToStop"},              /* reg 66 */
    {"ReversePower","LowLoad","GenLowFreq","GenHighFreq"},              /* reg 67 */
    {"GenHighCurrent","GenOverload","UnbalancedLoad","EmergencyStop"},  /* reg 68 */
    {"ChargeAltFail","MaintenanceDue","AFTActTimeout","AshLoadMaint"},  /* reg 69 */
    {"BattLowVolt","BattHighVolt","TempCktOpen","LOPShortBatt"},        /* reg 70 */
    {"FuelTheft","MPUFault","OilPressOpenCkt","AuxInputI"},             /* reg 71 */
    {"AuxInputA","AuxInputB","AuxInputC","AuxInputD"},                  /* reg 72 */
    {"AuxInputE","AuxInputF","AuxInputG","AuxInputH"},                  /* reg 73 */
    {"GenL1LowVolt","GenL1HighVolt","GenL2LowVolt","GenL2HighVolt"},    /* reg 74 */
    {"GenL3LowVolt","GenL3HighVolt","DGPhaseRotation","MainsPhaseRotation"}, /* reg 75 */
    {"FuelLevelOpenCkt","VBeltBroken","ECUPreheatFail","HighOilPress"}, /* reg 76 */
};

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
 * rx must be large enough for the biggest reply used by any driver: the SGC
 * 120 measurement read is 62 registers = 5 + 124 = 129 bytes, so callers pass
 * a 160-byte buffer. */
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
     * line (Ztech slaves 4.., Pantech 7..), and a late radar reply has a
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

/* ── Parse register values from a Modbus response ─────────────────────── */
static bool parse_regs(const uint8_t *rx, int rx_len, uint16_t *regs, int count) {
    int data_bytes = rx[2];
    if (data_bytes != count * 2 || rx_len < 3 + data_bytes) return false;
    for (int i = 0; i < count; i++)
        regs[i] = ((uint16_t)rx[3 + i * 2] << 8) | rx[4 + i * 2];
    return true;
}

/* One-shot "request + parse into regs[]" convenience used by the drivers. */
static bool modbus_read(uint8_t slave, uint8_t fc, uint16_t addr, uint16_t count,
                        uint16_t *regs) {
    uint8_t rx[160];
    int rlen;
    return modbus_request(slave, fc, addr, count, rx, sizeof(rx), &rlen)
        && parse_regs(rx, rlen, regs, count);
}

/* ── Shared nibble-alarm decoder ─────────────────────────────────────────
 * Walks `n_regs` registers, 4 nibbles each MSN→LSN, appending every active
 * alarm (nibble state 2..5) to out->alarms[] with its name from names[r][n].
 * Common to every model whose controller encodes alarms as 4 status nibbles
 * per register (both Sedemac and DEIF SGC do). Sets alarms_truncated if the
 * fixed alarm array fills up. */
static void decode_nibble_alarms(const uint16_t *regs, int n_regs,
                                 const char * const names[][4],
                                 GensetData_t *out) {
    for (int r = 0; r < n_regs; r++) {
        uint16_t word = regs[r];
        for (int n = 0; n < 4; n++) {
            uint8_t state = (word >> (12 - n * 4)) & 0x0F;
            if (state >= 2 && state <= 5) {
                if (out->alarm_count >= GENSET_MAX_ALARMS) {
                    out->alarms_truncated = true;
                    return;
                }
                out->alarms[out->alarm_count].name  = names[r][n];
                out->alarms[out->alarm_count].state = state;
                out->alarm_count++;
            }
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Model driver: Sedemac GenControl GC1211 / GC1111
 * Register map identical across both; only the read function code differs
 * (GC1211 = FC04 Read Input Registers, GC1111 = FC03 Read Holding Registers),
 * verified byte-for-byte against live wire captures of both. GC1211 also has a
 * separate FC03 holding space carrying command/operating-mode readback.
 * ══════════════════════════════════════════════════════════════════════════ */
static bool sedemac_read(uint8_t slave, uint8_t fc, bool probe_holding,
                         GensetData_t *out) {
    uint16_t b1[23], b2[12];

    /* Batch 1: addr 0, count 23 → protocol rev + measurements reg 0..22 */
    if (!modbus_read(slave, fc, 0, 23, b1)) {
        ESP_LOGW(TAG, "Sedemac batch1 failed");
        return false;
    }
    /* Batch 2: addr 57, count 12 → regs 57..68:
     *   b2[0..9] = reg 57..66 = Alarm 1..10
     *   b2[10]   = reg 67     = Genset Status word
     *   b2[11]   = reg 68     = Genset Command Status
     * NOTE: the GC1201/02/03/902 offset table lists these one register higher,
     * but the GC1211/GC1111 hardware we deploy uses the addresses above — do
     * not "correct" this to 58 without a fresh wire capture. */
    if (!modbus_read(slave, fc, 57, 12, b2)) {
        ESP_LOGW(TAG, "Sedemac batch2 failed");
        return false;
    }

    out->protocol_rev         = b1[0];
    out->lube_oil_pressure    = b1[1]  / 100.0f;
    out->engine_temp          = b1[2];
    out->fuel_level           = b1[3];
    out->charging_alt_voltage = b1[4]  / 10.0f;
    out->battery_voltage      = b1[5]  / 10.0f;
    out->no_of_starts         = b1[6];
    out->engine_run_time      = b1[7];
    out->engine_speed         = b1[8];
    out->genset_freq          = b1[9]  / 10.0f;
    out->genset_voltage[0]    = b1[10] / 10.0f;
    out->genset_voltage[1]    = b1[11] / 10.0f;
    out->genset_voltage[2]    = b1[12] / 10.0f;
    out->load_current[0]      = b1[16] / 100.0f;
    out->load_current[1]      = b1[17] / 100.0f;
    out->load_current[2]      = b1[18] / 100.0f;
    out->genset_power[0]      = b1[19] / 10.0f;
    out->genset_power[1]      = b1[20] / 10.0f;
    out->genset_power[2]      = b1[21] / 10.0f;
    out->genset_power[3]      = b1[22] / 10.0f;

    /* Alarms: regs 57..66 = b2[0..9], nibble MSN→LSN. */
    decode_nibble_alarms(b2, 10, g_sedemac_alarm_names, out);

    /* Raw first-3-alarm-register values, alongside (not instead of) the
     * decoded alarms[] above — no extra bus transaction, already in b2. */
    out->alr1     = b2[0];
    out->alr2     = b2[1];
    out->alr3     = b2[2];
    out->has_alr  = true;

    /* Cumulative energy, regs 35-36 — outside both existing batches, needs
     * its own read. NOT wire-verified: scale factor and word order (assumed
     * high-word-first) are unconfirmed. Optional/non-fatal like holdingRegs
     * below — a failure here must not fail measurements+alarms that already
     * succeeded. */
    uint16_t kwh_regs[2];
    if (modbus_read(slave, fc, 35, 2, kwh_regs)) {
        out->kwh     = ((uint32_t)kwh_regs[0] << 16) | kwh_regs[1];
        out->has_kwh = true;
    } else {
        ESP_LOGW(TAG, "Sedemac KWH regs (35-36) unavailable — omitting kwh");
    }

    /* Status word (b2[10] = reg67): bits 15,14,13,12,11 per SEDEMAC spec.
     * Verified vs GC1211: C000 = available+running; 2000 = stoppedNormally. */
    uint16_t sw = b2[10];
    out->available          = (sw >> 15) & 1;
    out->running            = (sw >> 14) & 1;
    out->stopped_normally   = (sw >> 13) & 1;
    out->stopped_with_fault = (sw >> 12) & 1;
    out->mode               = ((sw >> 11) & 1) ? "Auto" : "Manual";

    /* Command status (b2[11] = reg68): 1-based index, 0/out-of-range=Unknown. */
    static const char * const cs_labels[] =
        {"Unknown","Started","Stopped","AlreadyON","AlreadyOFF","ManualMode","NotStarted"};
    uint16_t cs = b2[11];
    out->command_status     = (cs >= 1 && cs <= 6) ? cs_labels[cs] : cs_labels[0];
    out->has_command_status = true;

    /* Holding regs (FC03 reg 0..1: command + operating mode) — GC1211 only.
     * OPTIONAL: some GC1211 variants don't expose the FC03 holding space at
     * all. A failure here must NOT fail the whole poll — the measurements and
     * alarms already succeeded and are what matter, and Auto/Manual is also in
     * the status word. Leave has_holding=false so the payload omits it. On the
     * GC1111 family the FC03 space IS the measurement map, so we never probe. */
    if (probe_holding) {
        uint16_t b3[2];
        if (modbus_read(slave, 0x03, 0, 2, b3)) {
            out->holding_command        = b3[0];
            out->holding_operating_mode = b3[1];
            out->has_holding            = true;
        } else {
            ESP_LOGW(TAG, "Sedemac holding regs unavailable — omitting holdingRegs");
        }
    }

    out->valid = true;
    return true;
}

static bool sedemac_read_gc1211(uint8_t slave, GensetData_t *out) {
    return sedemac_read(slave, 0x04, true, out);   /* FC04, has FC03 holding space */
}
static bool sedemac_read_gc1111(uint8_t slave, GensetData_t *out) {
    return sedemac_read(slave, 0x03, false, out);  /* FC03, no separate holding    */
}

/* ══════════════════════════════════════════════════════════════════════════
 * Model driver: DEIF SGC 120 Mk II
 * FC03 holding registers, DEIF Modbus table (doc 4189341403). Two reads:
 *   A) regs 0..61  — protocol rev + AC/engine measurements + running hours
 *   B) regs 65..78 — 12 alarm registers + I/O status(77) + DG status word(78)
 * Two reads not one: the live capture proved a single read up to reg 66 works,
 * so read A stays inside proven range; read B is a small separate block for
 * registers the capture didn't cover. Scalings per the DEIF datatype table.
 * ══════════════════════════════════════════════════════════════════════════ */
static bool sgc120_read(uint8_t slave, GensetData_t *out) {
    uint16_t m[62];   /* regs 0..61  */
    uint16_t a[14];   /* regs 65..78 (index = reg - 65) */

    if (!modbus_read(slave, 0x03, 0, 62, m)) {
        ESP_LOGW(TAG, "SGC120 read A (measurements) failed");
        return false;
    }
    if (!modbus_read(slave, 0x03, 65, 14, a)) {
        ESP_LOGW(TAG, "SGC120 read B (alarms/status) failed");
        return false;
    }

    out->protocol_rev         = m[0];
    out->lube_oil_pressure    = m[51] / 10.0f;              /* 0.1 Bar          */
    out->engine_temp          = (int16_t)m[52] / 10;        /* 0.1 °C signed    */
    out->fuel_level           = m[53];                      /* 1 %              */
    out->charging_alt_voltage = m[55] / 10.0f;              /* 0.1 V            */
    out->battery_voltage      = m[56] / 10.0f;              /* 0.1 V            */
    out->no_of_starts         = m[58];                      /* count            */
    out->engine_run_time      = ((uint32_t)m[60] << 16) | m[61]; /* 32-bit hrs, hi word first */
    out->engine_speed         = m[57];                      /* 1 RPM            */
    out->genset_freq          = m[7]  / 10.0f;              /* 0.1 Hz (Gen R)   */
    out->genset_voltage[0]    = m[1]  / 10.0f;              /* 0.1 V            */
    out->genset_voltage[1]    = m[2]  / 10.0f;
    out->genset_voltage[2]    = m[3]  / 10.0f;
    out->load_current[0]      = m[23] / 10.0f;              /* 0.1 A            */
    out->load_current[1]      = m[24] / 10.0f;
    out->load_current[2]      = m[25] / 10.0f;
    out->genset_power[0]      = (int16_t)m[26] / 10.0f;     /* 0.1 kW signed    */
    out->genset_power[1]      = (int16_t)m[27] / 10.0f;
    out->genset_power[2]      = (int16_t)m[28] / 10.0f;
    out->genset_power[3]      = (int16_t)m[29] / 10.0f;

    /* Alarms: regs 65..76 = a[0..11], nibble MSN→LSN. */
    decode_nibble_alarms(a, 12, g_sgc120_alarm_names, out);

    /* DG status word (reg 78 = a[13]). Bit positions per the DEIF datatype
     * table (1-indexed there; 0-indexed here). NOT wire-verified — see the
     * warning on g_sgc120_alarm_names. */
    uint16_t sw = a[13];
    out->available          = (sw >> 4) & 1;   /* bit5: Gen available          */
    out->running            = (sw >> 8) & 1;   /* bit9: Current DG status      */
    out->stopped_normally   = (sw >> 7) & 1;   /* bit8: DG stopped normally    */
    out->stopped_with_fault = (sw >> 6) & 1;   /* bit7: DG stopped with fault  */
    int md = (sw >> 11) & 0x7;                  /* bits 12..14: operation mode  */
    out->mode = (md == 5) ? "Auto"
              : (md == 4) ? "Manual"
              : (md == 6) ? "Scheduler"
              : "Unknown";

    /* SGC 120 has no read-back command status and no GC1211-style holding
     * space (its DG-mode change is FC16 write-only), so both stay absent. */
    out->valid = true;
    return true;
}

/* Placeholder for a Sedemac family member whose exact register offsets aren't
 * verified yet (GC1201/1202/1203/GC902 — see the comment on GENSET_MODEL_MAX
 * in config.h). Deliberately does nothing but fail: returning false here
 * always produces the standard genset 404 shape, never a wrong reading. This
 * is the safe alternative to reusing sedemac_read_gc1211/gc1111 against
 * hardware that's one register off from what those were verified against.
 * Replace with a real driver once the protocol document is available —
 * follow the sgc120_read() pattern above as the template. */
static bool sedemac_read_unverified(uint8_t slave, GensetData_t *out) {
    (void)slave;
    memset(out, 0, sizeof(*out));
    ESP_LOGW(TAG, "Genset model has no verified driver yet (awaiting protocol doc) — reporting 404");
    return false;
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
    { GENSET_MODEL_GC1211, "Sedemac GC1211",     sedemac_read_gc1211    },
    { GENSET_MODEL_GC1111, "Sedemac GC1111",     sedemac_read_gc1111    },
    { GENSET_MODEL_SGC120, "DEIF SGC 120 Mk II", sgc120_read            },
    { GENSET_MODEL_GC1201, "Sedemac GC1201 (unverified — 404 until driver added)", sedemac_read_unverified },
    { GENSET_MODEL_GC1202, "Sedemac GC1202 (unverified — 404 until driver added)", sedemac_read_unverified },
    { GENSET_MODEL_GC1203, "Sedemac GC1203 (unverified — 404 until driver added)", sedemac_read_unverified },
    { GENSET_MODEL_GC902,  "Sedemac GC902 (unverified — 404 until driver added)",  sedemac_read_unverified },
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
 * JSON serialiser — fully model-agnostic. Operates only on the normalised
 * GensetData_t, so it never needs to know which controller produced it.
 * ══════════════════════════════════════════════════════════════════════════ */

/* snprintf returns the length it WANTED to write, not what actually fit. If a
 * section truncates, `pos` sails past buf_size and the next call's
 * (buf_size - pos) wraps to a huge size_t → out-of-bounds write. Clamp after
 * every append so truncation degrades to a cut-off payload, not corruption. */
static inline int clamp_pos(int pos, size_t buf_size) {
    return (pos > (int)buf_size - 1) ? (int)buf_size - 1 : pos;
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

    int pos = 0;
    pos += snprintf(buf + pos, buf_size - pos,
        "{\"HW_ID\":\"%s\",\"genset\":{"
        "\"PREV\":%u,"
        "\"OPR\":%.2f,"
        "\"CT\":%d,"
        "\"FLVL\":%u,"
        "\"CAV\":%.1f,"
        "\"BV\":%.1f,"
        "\"NOS\":%u,"
        "\"RHR\":%lu,"
        "\"ENGSP\":%u,"
        "\"HZ\":%.1f,"
        "\"L1N\":%.1f,\"L2N\":%.1f,\"L3N\":%.1f,"
        "\"I1\":%.2f,\"I2\":%.2f,\"I3\":%.2f,"
        "\"KW1\":%.1f,\"KW2\":%.1f,\"KW3\":%.1f,\"KW\":%.1f,",
        hw_id,
        d->protocol_rev,
        d->lube_oil_pressure,
        d->engine_temp,
        d->fuel_level,
        d->charging_alt_voltage,
        d->battery_voltage,
        d->no_of_starts,
        (unsigned long)d->engine_run_time,
        d->engine_speed,
        d->genset_freq,
        d->genset_voltage[0], d->genset_voltage[1], d->genset_voltage[2],
        d->load_current[0],   d->load_current[1],   d->load_current[2],
        d->genset_power[0],   d->genset_power[1],   d->genset_power[2], d->genset_power[3]);
    pos = clamp_pos(pos, buf_size);

    /* Optional: cumulative energy + raw first-3-alarm-registers (Sedemac
     * only — absent on DEIF SGC 120 and unverified reserved models). */
    if (d->has_kwh) {
        pos += snprintf(buf + pos, buf_size - pos, "\"KWH\":%lu,", (unsigned long)d->kwh);
        pos = clamp_pos(pos, buf_size);
    }
    if (d->has_alr) {
        pos += snprintf(buf + pos, buf_size - pos,
                        "\"ALR1\":%u,\"ALR2\":%u,\"ALR3\":%u,",
                        d->alr1, d->alr2, d->alr3);
        pos = clamp_pos(pos, buf_size);
    }

    /* Alarms object. Reserve room for the fixed-size suffix (status +
     * commandStatus + holdingRegs + closing) so the alarm loop stops with
     * enough budget left that the FULL payload — not just the alarms block —
     * stays under the modem's real transmission ceiling. */
    int alarm_budget_end = MQTT_MODEM_PAYLOAD_SAFE_MAX - 280;
    if (alarm_budget_end > (int)buf_size - 50) alarm_budget_end = (int)buf_size - 50;

    pos += snprintf(buf + pos, buf_size - pos, "\"alarms\":{");
    pos = clamp_pos(pos, buf_size);
    bool first = true;
    int i;
    for (i = 0; i < d->alarm_count && pos < alarm_budget_end; i++) {
        if (!first) buf[pos++] = ',';
        pos = clamp_pos(pos, buf_size);
        pos += snprintf(buf + pos, buf_size - pos, "\"%s\":\"%s\"",
                        d->alarms[i].name, g_alarm_state_names[d->alarms[i].state]);
        pos = clamp_pos(pos, buf_size);
        first = false;
    }
    /* Truncated if the model couldn't capture all alarms OR the loop was cut
     * short by the payload budget — either way more alarms may be active than
     * are shown, so flag it rather than let the cloud read a short list as
     * complete. */
    bool truncated = d->alarms_truncated || (i < d->alarm_count);
    pos += snprintf(buf + pos, buf_size - pos,
                    truncated ? "},\"alarmsTruncated\":true," : "},");
    pos = clamp_pos(pos, buf_size);

    pos += snprintf(buf + pos, buf_size - pos,
        "\"status\":{"
        "\"available\":%s,"
        "\"running\":%s,"
        "\"stoppedNormally\":%s,"
        "\"stoppedWithFault\":%s,"
        "\"mode\":\"%s\"}",
        d->available          ? "true" : "false",
        d->running            ? "true" : "false",
        d->stopped_normally   ? "true" : "false",
        d->stopped_with_fault ? "true" : "false",
        d->mode ? d->mode : "Unknown");
    pos = clamp_pos(pos, buf_size);

    if (d->has_command_status) {
        pos += snprintf(buf + pos, buf_size - pos,
                        ",\"commandStatus\":\"%s\"", d->command_status);
        pos = clamp_pos(pos, buf_size);
    }
    if (d->has_holding) {
        pos += snprintf(buf + pos, buf_size - pos,
                        ",\"holdingRegs\":{\"command\":%u,\"operatingMode\":%u}",
                        d->holding_command, d->holding_operating_mode);
        pos = clamp_pos(pos, buf_size);
    }

    pos += snprintf(buf + pos, buf_size - pos, "},\"TS\":\"%s\"}", ts);
    pos = clamp_pos(pos, buf_size);

    /* clamp_pos stops truncation from corrupting memory, but a truncated
     * payload is still syntactically broken JSON — a cloud parser would reject
     * the whole message. Landing within 2 bytes of the ceiling, or over the
     * modem's real QMTPUBEX limit, is itself a strong signal something is
     * wrong, so fall back to the guaranteed-valid "404" shape instead. */
    if (pos >= (int)buf_size - 2) {
        ESP_LOGW(TAG, "genset payload truncated at %d/%zu bytes — publishing 404 instead",
                 pos, buf_size);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
    } else if (pos > MQTT_MODEM_PAYLOAD_SAFE_MAX) {
        ESP_LOGW(TAG, "genset payload %d bytes exceeds modem safe max %d — publishing 404 instead",
                 pos, MQTT_MODEM_PAYLOAD_SAFE_MAX);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
    }
}
