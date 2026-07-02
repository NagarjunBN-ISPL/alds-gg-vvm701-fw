#include "radar.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "radar";

/* Standard Modbus RTU CRC16 (poly 0xA001, init 0xFFFF). Frames are built
 * on demand per tank_idx instead of a fixed table, so tank count can scale
 * without needing pre-computed entries for every possible slave address. */
static uint16_t modbus_crc16(const uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (uint16_t)((crc >> 1) ^ 0xA001);
            else         crc >>= 1;
        }
    }
    return crc;
}

/* FC03 (Ztech) / FC04 (Pantech), read 2 registers starting at 0 */
static void radar_frame(uint8_t slave, uint8_t func, uint8_t *out /* 8 bytes */) {
    out[0] = slave;
    out[1] = func;
    out[2] = 0x00; out[3] = 0x00;
    out[4] = 0x00; out[5] = 0x02;
    uint16_t crc = modbus_crc16(out, 6);
    out[6] = (uint8_t)(crc & 0xFF);
    out[7] = (uint8_t)((crc >> 8) & 0xFF);
}

/* 500 ms fixed read window for radar (no per-byte-reset needed) */
#define RADAR_TIMEOUT_MS 500

static int radar_xfer(const uint8_t *cmd, size_t cmd_len,
                      uint8_t *resp, size_t resp_max) {
    uint8_t tmp[64];
    while (uart_read_bytes(RS485_UART, tmp, sizeof(tmp), 0) > 0);

    uart_write_bytes(RS485_UART, cmd, cmd_len);
    vTaskDelay(pdMS_TO_TICKS(200)); /* TX→RX turn-around */

    int got = 0;
    int64_t dl = esp_timer_get_time() + (int64_t)RADAR_TIMEOUT_MS * 1000;
    while (esp_timer_get_time() < dl && got < (int)resp_max) {
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0)
            resp[got++] = ch;
    }
    return got;
}

bool radar_ztech_read(int tank_idx, uint8_t *rx_buf, int *rx_len) {
    if (tank_idx < 0 || tank_idx >= RADAR_MAX) { *rx_len = 0; return false; }
    uint8_t cmd[8];
    radar_frame((uint8_t)(4 + tank_idx), 0x03, cmd);
    *rx_len = radar_xfer(cmd, 8, rx_buf, 64);
    if (*rx_len == 0) {
        ESP_LOGW(TAG, "Ztech tank%d no response", tank_idx + 1);
        return false;
    }
    return true;
}

bool radar_pantech_read(int tank_idx, uint8_t *rx_buf, int *rx_len) {
    if (tank_idx < 0 || tank_idx >= RADAR_MAX) { *rx_len = 0; return false; }
    uint8_t cmd[8];
    radar_frame((uint8_t)(7 + tank_idx), 0x04, cmd);
    *rx_len = radar_xfer(cmd, 8, rx_buf, 64);
    if (*rx_len == 0) {
        ESP_LOGW(TAG, "Pantech tank%d no response", tank_idx + 1);
        return false;
    }
    return true;
}
