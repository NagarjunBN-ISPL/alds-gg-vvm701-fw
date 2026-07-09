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

/* ── Alarm name table (10 registers × 4 nibbles, MSN→LSN) ──────────────
 * Order verified against a live GC1211 wire capture (SEDMAC decode doc):
 * register 57 = 0x1110 decodes MSN→LSN as RadiatorWaterLevel, LowFuelLevel,
 * HighEngTemp, LOP — cross-checked against the GC1201/02/03/902 register
 * map, which independently agrees. (A previous revision of this table had
 * every row's nibble order mirrored — MSN/LSN swapped — which misattributed
 * every alarm to the wrong parameter; e.g. a real LOP shutdown would have
 * reported as "RadiatorWaterLevel: Shutdown".) */
static const char * const g_alarm_names[10][4] = {
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

/* ── Modbus RTU request / response ───────────────────────────────────── */
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
     * CRC alone is NOT enough on this bus: the radars share the same
     * RS485 line (Ztech slaves 4.., Pantech 7..), and a late radar reply
     * has a perfectly valid CRC — without this check it could be accepted
     * as genset register data. */
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

/* ── Parse register values from Modbus response ───────────────────────── */
static bool parse_regs(const uint8_t *rx, int rx_len, uint16_t *regs, int count) {
    int data_bytes = rx[2];
    if (data_bytes != count * 2 || rx_len < 3 + data_bytes) return false;
    for (int i = 0; i < count; i++)
        regs[i] = ((uint16_t)rx[3 + i * 2] << 8) | rx[4 + i * 2];
    return true;
}

/* ── Public API ───────────────────────────────────────────────────────── */
bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out) {
    memset(out, 0, sizeof(*out));

    uint8_t rx[64]; int rlen;

    /* Measurement/alarm/status reads use FC04 on the GC1211 family (Read
     * Input Registers) and FC03 on the GC1111 family (Read Holding Registers)
     * — same register addresses, verified byte-for-byte against live wire
     * captures of both models (only the function code differs). */
    uint8_t data_fc = (model == GENSET_MODEL_GC1111) ? 0x03 : 0x04;

    /* Batch 1: addr 0, count 23 (protocol rev + measurements reg 0..22) */
    if (!modbus_request(slave, data_fc, 0, 23, rx, sizeof(rx), &rlen)
        || !parse_regs(rx, rlen, out->batch1, 23)) {
        ESP_LOGW(TAG, "Batch1 failed");
        return false;
    }

    /* Batch 2: addr 57, count 12 → regs 57..68:
     *   b2[0..9] = reg 57..66 = Alarm 1..10
     *   b2[10]   = reg 67     = Genset Status word
     *   b2[11]   = reg 68     = Genset Command Status
     * NOTE: the GC1201/02/03/902 offset table lists these one register higher,
     * but the GC1211/GC1111 hardware we deploy uses the addresses above — do
     * not "correct" this to 58 without a fresh wire capture. */
    if (!modbus_request(slave, data_fc, 57, 12, rx, sizeof(rx), &rlen)
        || !parse_regs(rx, rlen, out->batch2, 12)) {
        ESP_LOGW(TAG, "Batch2 failed");
        return false;
    }

    /* Batch 3: FC03 holding regs 0..1 (command register + operating mode).
     * Only meaningful on the GC1211 family, where the FC03 space is separate
     * from measurements. On the GC1111 family the FC03 space IS the
     * measurement map (reg 0 = protocol rev, reg 1 = oil pressure), so reading
     * addr 0..1 there would return measurements mislabelled as command/mode —
     * skip it and leave holdingRegs unread. */
    if (model == GENSET_MODEL_GC1211) {
        /* Command/operating-mode readback (FC03 holding regs 0..1) is OPTIONAL
         * — some GC1211-family variants don't expose the FC03 holding space at
         * all. A failure here must NOT 404 the whole genset: the measurements
         * (batch1) and alarms/status (batch2) already succeeded and are the
         * data that matters, and the Auto/Manual mode is also in the status
         * word (bit 11). Leave has_holding=false so the payload simply omits
         * holdingRegs, matching the build side's conditional handling. */
        if (modbus_request(slave, 0x03, 0, 2, rx, sizeof(rx), &rlen)
            && parse_regs(rx, rlen, out->batch3, 2)) {
            out->has_holding = true;
        } else {
            ESP_LOGW(TAG, "Batch3 (holding regs) unavailable — reporting measurements/alarms/status only");
        }
    }

    out->valid = true;
    return true;
}

/* snprintf returns the length it WANTED to write, not what actually fit.
 * If a section truncates, `pos` sails past buf_size and the next call's
 * (buf_size - pos) wraps to a huge size_t → out-of-bounds write. Clamp
 * after every append so truncation degrades to a cut-off payload instead
 * of memory corruption. */
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

    const uint16_t *b1 = d->batch1;
    const uint16_t *b2 = d->batch2;
    const uint16_t *b3 = d->batch3;

    int pos = 0;
    pos += snprintf(buf + pos, buf_size - pos,
        "{\"HW_ID\":\"%s\",\"genset\":{"
        "\"protocolRevision\":%u,"
        "\"lubeOilPressure\":%.2f,"
        "\"engineTemp\":%u,"
        "\"fuelLevel\":%u,"
        "\"chargingAltVoltage\":%.1f,"
        "\"batteryVoltage\":%.1f,"
        "\"noOfStarts\":%u,"
        "\"engineRunTime\":%u,"
        "\"engineSpeed\":%u,"
        "\"gensetFrequency\":%.1f,"
        "\"gensetL1Voltage\":%.1f,\"gensetL2Voltage\":%.1f,\"gensetL3Voltage\":%.1f,"
        "\"loadL1Current\":%.2f,\"loadL2Current\":%.2f,\"loadL3Current\":%.2f,"
        "\"gensetL1Power\":%.1f,\"gensetL2Power\":%.1f,\"gensetL3Power\":%.1f,\"gensetTotalPower\":%.1f,",
        hw_id,
        b1[0],
        b1[1]  / 100.0f,
        b1[2],
        b1[3],
        b1[4]  / 10.0f,
        b1[5]  / 10.0f,
        b1[6],
        b1[7],
        b1[8],
        b1[9]  / 10.0f,
        b1[10] / 10.0f, b1[11] / 10.0f, b1[12] / 10.0f,
        b1[16] / 100.0f, b1[17] / 100.0f, b1[18] / 100.0f,
        b1[19] / 10.0f, b1[20] / 10.0f, b1[21] / 10.0f, b1[22] / 10.0f);
    pos = clamp_pos(pos, buf_size);

    /* Alarms: regs 57–66 = Alarm 1..10 (batch2[0..9]), nibble MSN→LSN,
     * object format, string states. Verified against the GC1211 decode:
     * Alarm 1 @ reg 57, MSN→LSN = RadiatorWaterLevel/LowFuelLevel/HighEngTemp/LOP. */
    static const char * const alarm_state_names[] =
        {"Disabled","Inactive","Warning","Shutdown","ElecTrip","Notification"};

    /* Reserve room for the fixed-size suffix (status + commandStatus +
     * holdingRegs + closing) so the alarm loop stops with enough budget left
     * that the FULL payload — not just the alarms block — stays under the
     * modem's real transmission ceiling. Alarms are the only unbounded part;
     * everything else here is fixed-size. See MQTT_MODEM_PAYLOAD_SAFE_MAX. */
    int alarm_budget_end = MQTT_MODEM_PAYLOAD_SAFE_MAX - 280;
    if (alarm_budget_end > (int)buf_size - 50) alarm_budget_end = (int)buf_size - 50;

    pos += snprintf(buf + pos, buf_size - pos, "\"alarms\":{");
    bool first_alarm = true;
    int  r;
    for (r = 0; r < 10 && pos < alarm_budget_end; r++) {
        uint16_t word = b2[r];
        for (int n = 0; n < 4; n++) {
            uint8_t state = (word >> (12 - n * 4)) & 0x0F;
            if (state >= 2 && state <= 5) {
                if (!first_alarm) buf[pos++] = ',';
                pos += snprintf(buf + pos, buf_size - pos,
                                "\"%s\":\"%s\"",
                                g_alarm_names[r][n], alarm_state_names[state]);
                first_alarm = false;
            }
        }
    }
    /* r < 10 means the loop was cut short by the modem-payload budget, not
     * because it ran out of alarm registers — flag it so the cloud side can
     * tell "no more alarms" apart from "more alarms exist but didn't fit". */
    bool alarms_truncated = (r < 10);
    pos = clamp_pos(pos, buf_size);
    pos += snprintf(buf + pos, buf_size - pos,
                    alarms_truncated ? "},\"alarmsTruncated\":true," : "},");
    pos = clamp_pos(pos, buf_size);

    /* Status word (batch2[10] = reg67, "Genset Status"): bits 15,14,13,12,11 per SEDEMAC spec.
     * Verified vs GC1211: C000 = available+running; 2000 = stoppedNormally. */
    uint16_t sw = b2[10];
    pos += snprintf(buf + pos, buf_size - pos,
        "\"status\":{"
        "\"available\":%s,"
        "\"running\":%s,"
        "\"stoppedNormally\":%s,"
        "\"stoppedWithFault\":%s,"
        "\"mode\":\"%s\"},",
        ((sw >> 15) & 1) ? "true" : "false",
        ((sw >> 14) & 1) ? "true" : "false",
        ((sw >> 13) & 1) ? "true" : "false",
        ((sw >> 12) & 1) ? "true" : "false",
        ((sw >> 11) & 1) ? "Auto" : "Manual");
    pos = clamp_pos(pos, buf_size);

    /* Command status (batch2[11] = reg68): 1-based index, 0 = Unknown */
    static const char * const cs_labels[] =
        {"Unknown","Started","Stopped","AlreadyON","AlreadyOFF","ManualMode","NotStarted"};
    uint16_t cs = b2[11];
    const char *cs_str = (cs >= 1 && cs <= 6) ? cs_labels[cs] : cs_labels[0];
    pos += snprintf(buf + pos, buf_size - pos,
                    "\"commandStatus\":\"%s\"", cs_str);
    pos = clamp_pos(pos, buf_size);

    /* Holding regs (batch3) — command/operating-mode readback, only read on
     * the GC1211 family (see genset_poll). Omitted entirely on GC1111 rather
     * than reported as zeros, matching the reference firmware's conditional
     * "if (holdingOk)" behaviour. */
    if (d->has_holding) {
        pos += snprintf(buf + pos, buf_size - pos,
                        ",\"holdingRegs\":{\"command\":%u,\"operatingMode\":%u}",
                        b3[0], b3[1]);
        pos = clamp_pos(pos, buf_size);
    }

    pos += snprintf(buf + pos, buf_size - pos,
                    "},\"TS\":\"%s\"}", ts);
    pos = clamp_pos(pos, buf_size);

    /* clamp_pos stops truncation from corrupting memory, but a truncated
     * payload is still syntactically broken JSON (missing closing braces,
     * possibly mid-string) — a cloud parser would silently reject the whole
     * message. Landing within 2 bytes of the ceiling on a 4096-byte buffer
     * sized for a ~2.2KB worst case is itself a strong signal something is
     * wrong, so fall back to the same guaranteed-valid "404" shape used for
     * a failed poll rather than publish something unparsable. */
    if (pos >= (int)buf_size - 2) {
        ESP_LOGW(TAG, "genset payload truncated at %d/%zu bytes — publishing 404 instead", pos, buf_size);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
    } else if (pos > MQTT_MODEM_PAYLOAD_SAFE_MAX) {
        /* Safety net only — the alarm-loop budget above should already keep
         * this under the modem's real QMTPUBEX ceiling (~1500 bytes on the
         * EC200U; data beyond it is rejected/omitted at the modem, not
         * detectable from here). If it still overshot, publishing anyway
         * would silently vanish at the modem — a 404 that's visible in
         * diagnostics beats data that disappears with no trace. */
        ESP_LOGW(TAG, "genset payload %d bytes exceeds modem safe max %d — publishing 404 instead",
                 pos, MQTT_MODEM_PAYLOAD_SAFE_MAX);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"genset\":{\"data\":\"404\"},\"TS\":\"%s\"}",
                 hw_id, ts);
    }
}
