#include "modem.h"
#include "config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "modem";

void modem_uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = MODEM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(MODEM_UART, MODEM_BUF, MODEM_BUF, 0, NULL, 0);
    uart_param_config(MODEM_UART, &cfg);
    uart_set_pin(MODEM_UART, MODEM_TX_PIN, MODEM_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MODEM_PWRKEY),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(MODEM_PWRKEY, 0);
}

void modem_flush(void) {
    uint8_t tmp[64];
    while (uart_read_bytes(MODEM_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(30)) > 0);
}

bool modem_at(const char *cmd, const char *expect, int timeout_ms,
              char *resp, size_t resp_size) {
    modem_flush();

    static char line[512];
    int n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    ESP_LOGI(TAG, ">> %s", cmd);
    uart_write_bytes(MODEM_UART, line, n);

    static char buf[2048];
    memset(buf, 0, sizeof(buf));
    int  total = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            if (total < (int)sizeof(buf) - 1) {
                buf[total++] = (char)ch;
                buf[total]   = '\0';
            }
            if (expect && strstr(buf, expect)) {
                /* Found the marker, but the rest of this line (value, or the
                 * trailing OK) may not have arrived yet — drain a bit more
                 * before returning so callers don't see a truncated field. */
                vTaskDelay(pdMS_TO_TICKS(30));
                int space = (int)sizeof(buf) - 1 - total;
                while (space > 0) {
                    int n2 = uart_read_bytes(MODEM_UART, (uint8_t *)buf + total,
                                              space, pdMS_TO_TICKS(20));
                    if (n2 <= 0) break;
                    total += n2;
                    buf[total] = '\0';
                    space = (int)sizeof(buf) - 1 - total;
                }
                ESP_LOGI(TAG, "<< %s", buf);
                if (resp && resp_size > 0) {
                    strncpy(resp, buf, resp_size - 1);
                    resp[resp_size - 1] = '\0';
                }
                return true;
            }
            if (strstr(buf, "\r\nERROR") || strstr(buf, "+CME ERROR")) {
                ESP_LOGW(TAG, "<< %s", buf);
                if (resp && resp_size > 0) {
                    strncpy(resp, buf, resp_size - 1);
                    resp[resp_size - 1] = '\0';
                }
                return false;
            }
        }
    }
    ESP_LOGW(TAG, "<< (timeout %d ms) %s", timeout_ms, buf);
    if (resp && resp_size > 0) {
        strncpy(resp, buf, resp_size - 1);
        resp[resp_size - 1] = '\0';
    }
    return false;
}

bool modem_read_until(const char *expect, int timeout_ms,
                      char *resp, size_t resp_size) {
    /* Like modem_at but sends nothing — just reads until pattern found */
    static char buf[2048];
    memset(buf, 0, sizeof(buf));
    int  total = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            if (total < (int)sizeof(buf) - 1) {
                buf[total++] = (char)ch;
                buf[total]   = '\0';
            }
            if (expect && strstr(buf, expect)) {
                if (resp && resp_size > 0) {
                    strncpy(resp, buf, resp_size - 1);
                    resp[resp_size - 1] = '\0';
                }
                return true;
            }
        }
    }
    if (resp && resp_size > 0) {
        strncpy(resp, buf, resp_size - 1);
        resp[resp_size - 1] = '\0';
    }
    return false;
}

bool modem_power_on(void) {
    /* Try AT first — module may already be on */
    for (int i = 0; i < 3; i++) {
        if (modem_at("AT", "OK", 1000, NULL, 0)) {
            ESP_LOGI(TAG, "Modem already on");
            modem_at("ATE0", "OK", 2000, NULL, 0);
            return true;
        }
    }

    /* Pulse PWRKEY high for 1.5 s */
    gpio_set_level(MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    gpio_set_level(MODEM_PWRKEY, 0);

    /* Wait up to 15 s for the module to boot */
    for (int i = 0; i < 15; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (modem_at("AT", "OK", 1000, NULL, 0)) {
            ESP_LOGI(TAG, "Modem boot OK");
            modem_at("ATE0", "OK", 2000, NULL, 0);
            return true;
        }
    }
    ESP_LOGE(TAG, "Modem power-on failed");
    return false;
}

void modem_hard_reset(void) {
    ESP_LOGW(TAG, "Hard reset: power-cycling EC200U via PWRKEY ...");

    /* Graceful shutdown attempt — may fail if modem is hung, ignore either way */
    modem_at("AT+QPOWD=1", "OK", 5000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Force off: hold PWRKEY high > 2 s */
    gpio_set_level(MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
    gpio_set_level(MODEM_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));  /* allow full power-off */

    /* Power on: 1.5 s pulse (matches modem_power_on's pulse width) */
    gpio_set_level(MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    gpio_set_level(MODEM_PWRKEY, 0);
    ESP_LOGI(TAG, "EC200U power-cycled — waiting for boot ...");

    for (int i = 0; i < 15; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (modem_at("AT", "OK", 1000, NULL, 0)) {
            ESP_LOGI(TAG, "Modem boot OK after hard reset");
            modem_at("ATE0", "OK", 2000, NULL, 0);
            return;
        }
    }
    ESP_LOGE(TAG, "Modem unresponsive after hard reset");
}

bool modem_get_imei(char *imei, size_t size) {
    char resp[128] = {0};
    if (!modem_at("AT+CGSN", "OK", 5000, resp, sizeof(resp))) return false;

    /* Find a 15-digit run of digits */
    for (char *p = resp; *p; p++) {
        if (*p >= '0' && *p <= '9') {
            size_t len = 0;
            while (p[len] >= '0' && p[len] <= '9') len++;
            if (len == 15) {
                size_t copy = (len < size - 1) ? len : size - 1;
                memcpy(imei, p, copy);
                imei[copy] = '\0';
                return true;
            }
            p += len - 1;
        }
    }
    return false;
}

int modem_get_csq(void) {
    char resp[64] = {0};
    if (!modem_at("AT+CSQ", "+CSQ:", 3000, resp, sizeof(resp))) return -1;
    char *p = strstr(resp, "+CSQ:");
    if (!p) return -1;
    p += 5;
    while (*p == ' ') p++;
    return atoi(p);
}

bool modem_get_vbat(int *vbat_mv) {
    char resp[64] = {0};
    if (!modem_at("AT+CBC", "+CBC:", 3000, resp, sizeof(resp))) return false;
    char *p = strstr(resp, "+CBC:");
    if (!p) return false;
    /* +CBC: <bcs>,<bcl>,<voltage> */
    p = strchr(p, ','); if (!p) return false; p++;
    p = strchr(p, ','); if (!p) return false; p++;
    *vbat_mv = atoi(p);
    return true;
}

bool modem_get_timestamp(char *buf, size_t size) {
    char resp[128] = {0};

    /* Prefer AT+QLTS=2 → "+QLTS: "2026/03/22,10:30:00+22,0"" */
    if (modem_at("AT+QLTS=2", "+QLTS:", 5000, resp, sizeof(resp))) {
        char *q = strchr(resp, '"');
        if (q) {
            q++;
            int Y, Mo, D, h, m, s;
            if (sscanf(q, "%d/%d/%d,%d:%d:%d", &Y, &Mo, &D, &h, &m, &s) == 6) {
                snprintf(buf, size, "%04d-%02d-%02d %02d:%02d:%02d",
                         Y, Mo, D, h, m, s);
                return true;
            }
        }
    }

    /* Fallback: AT+CCLK? → "+CCLK: "26/03/22,10:30:00+22"" */
    memset(resp, 0, sizeof(resp));
    if (modem_at("AT+CCLK?", "+CCLK:", 3000, resp, sizeof(resp))) {
        char *q = strchr(resp, '"');
        if (q) {
            q++;
            int y, Mo, D, h, m, s;
            if (sscanf(q, "%d/%d/%d,%d:%d:%d", &y, &Mo, &D, &h, &m, &s) == 6) {
                snprintf(buf, size, "20%02d-%02d-%02d %02d:%02d:%02d",
                         y, Mo, D, h, m, s);
                return true;
            }
        }
    }

    snprintf(buf, size, "1970-01-01 00:00:00");
    return false;
}
