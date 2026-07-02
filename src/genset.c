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

/* ── Alarm name table (10 registers × 4 nibbles) ─────────────────────── */
static const char * const g_alarm_names[10][4] = {
    {"LOP","HighEngTemp","LowFuelLevel","RadiatorWaterLevel"},
    {"UnderSpeed","OverSpeed","FailToStart","FailToStop"},
    {"GensetLowVolt","GensetHighVolt","GensetLowFreq","GensetHighFreq"},
    {"HighCurrent","Overload","UnbalancedLoad","EmergencyStop"},
    {"ChargingAltFail","OilFilterMaint","FuelFilterMaint","AirFilterMaint"},
    {"BattLowVolt","BattHighVolt","OilPressOpenCkt","EngTempOpenCkt"},
    {"FuelTheft","MPULoss","ExtendedOverloadTrip","Reserved"},
    {"AuxInputA","AuxInputB","AuxInputC","AuxInputD"},
    {"AuxInputE","AuxInputF","AuxInputG","AuxInputH"},
    {"AuxInputI","Reserved","Reserved","Reserved"}
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
    /* Check FC (bit 7 set = exception) */
    if (rx[1] & 0x80) { ESP_LOGW(TAG, "Modbus exception 0x%02x", rx[2]); return false; }
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
bool genset_poll(uint8_t slave, GensetData_t *out) {
    memset(out, 0, sizeof(*out));

    uint8_t rx[64]; int rlen;

    /* Batch 1: FC04, addr 0, count 23 */
    if (!modbus_request(slave, 0x04, 0, 23, rx, sizeof(rx), &rlen)
        || !parse_regs(rx, rlen, out->batch1, 23)) {
        ESP_LOGW(TAG, "Batch1 failed");
        return false;
    }

    /* Batch 2: FC04, addr 57, count 12 */
    if (!modbus_request(slave, 0x04, 57, 12, rx, sizeof(rx), &rlen)
        || !parse_regs(rx, rlen, out->batch2, 12)) {
        ESP_LOGW(TAG, "Batch2 failed");
        return false;
    }

    /* Batch 3: FC03, addr 0, count 2 */
    if (!modbus_request(slave, 0x03, 0, 2, rx, sizeof(rx), &rlen)
        || !parse_regs(rx, rlen, out->batch3, 2)) {
        ESP_LOGW(TAG, "Batch3 failed");
        return false;
    }

    out->valid = true;
    return true;
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

    /* Alarms: regs 57–66 (batch2[0..9]), nibble MSN→LSN, object format, string states */
    static const char * const alarm_state_names[] =
        {"Disabled","Inactive","Warning","Shutdown","ElecTrip","Notification"};

    pos += snprintf(buf + pos, buf_size - pos, "\"alarms\":{");
    bool first_alarm = true;
    for (int r = 0; r < 10 && pos < (int)buf_size - 50; r++) {
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
    pos += snprintf(buf + pos, buf_size - pos, "},");

    /* Status word (batch2[10] = reg67): bits 15,14,13,12,11 per SEDEMAC spec */
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

    /* Command status (batch2[11] = reg68): 1-based index, 0 = Unknown */
    static const char * const cs_labels[] =
        {"Unknown","Started","Stopped","AlreadyON","AlreadyOFF","ManualMode","NotStarted"};
    uint16_t cs = b2[11];
    const char *cs_str = (cs >= 1 && cs <= 6) ? cs_labels[cs] : cs_labels[0];
    pos += snprintf(buf + pos, buf_size - pos,
                    "\"commandStatus\":\"%s\",", cs_str);

    /* Holding regs (batch3) — nested object */
    pos += snprintf(buf + pos, buf_size - pos,
                    "\"holdingRegs\":{\"command\":%u,\"operatingMode\":%u}",
                    b3[0], b3[1]);

    pos += snprintf(buf + pos, buf_size - pos,
                    "},\"TS\":\"%s\"}", ts);
}
