#include "di.h"
#include "config.h"
#include "mqtt.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

TaskHandle_t  g_di_task_handle = NULL;
DIEntry_t     di_fifo[DI_FIFO_SIZE];
volatile int  di_head  = 0;
volatile int  di_tail  = 0;
volatile int  di_count = 0;

static SemaphoreHandle_t di_mutex = NULL;
static uint8_t           di_enable_mask = 0;  /* set by di_task from shared var */

/* Shared config — written once from main before task starts */
static volatile uint8_t s_di_enable = 0;
void di_set_enable(uint8_t mask) { s_di_enable = mask; }

void di_uart_init(void) {
    uart_config_t cfg = {
        .baud_rate = NUVOTON_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(NUVOTON_UART, NUVOTON_BUF, NUVOTON_BUF, 0, NULL, 0);
    uart_param_config(NUVOTON_UART, &cfg);
    uart_set_pin(NUVOTON_UART, NUVOTON_TX_PIN, NUVOTON_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    di_mutex = xSemaphoreCreateMutex();
}

/* Very small JSON parser — finds "DI1": <val> patterns */
static int parse_int_field(const char *json, const char *key) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return -1;
    p += strlen(search);
    while (*p == ' ') p++;
    return atoi(p);
}

void di_task(void *param) {
    (void)param;
    g_di_task_handle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(NULL);
    di_enable_mask = s_di_enable;

    static char req[]  = "GETDATA\n";
    static char resp[NUVOTON_BUF];

    for (;;) {
        /* Refresh enable mask in case config was updated */
        di_enable_mask = s_di_enable;

        /* Send request */
        uart_write_bytes(NUVOTON_UART, req, strlen(req));
        vTaskDelay(pdMS_TO_TICKS(50));

        /* Read response */
        int got = uart_read_bytes(NUVOTON_UART, (uint8_t *)resp,
                                  NUVOTON_BUF - 1, pdMS_TO_TICKS(NUVOTON_TIMEOUT_MS));
        if (got > 0) resp[got] = '\0';
        else         resp[0]   = '\0';

        /* Build payload JSON */
        char payload[256];
        int  pos = 0;
        bool valid = (got > 4);

        static const char *di_keys[4] = {"DI1","DI2","DI3","DI4"};
        pos += snprintf(payload + pos, sizeof(payload) - pos, "{");
        bool first = true;
        for (int i = 0; i < 4; i++) {
            if (!(di_enable_mask & (1 << i))) continue;
            int val = valid ? parse_int_field(resp, di_keys[i]) : -1;
            if (!first) payload[pos++] = ',';
            if (val < 0)
                pos += snprintf(payload + pos, sizeof(payload) - pos,
                                "\"%s\":404", di_keys[i]);
            else
                pos += snprintf(payload + pos, sizeof(payload) - pos,
                                "\"%s\":%d", di_keys[i], val);
            first = false;
        }
        pos += snprintf(payload + pos, sizeof(payload) - pos, "}");

        /* Push to FIFO */
        if (xSemaphoreTake(di_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int slot = di_tail;
            strncpy(di_fifo[slot].payload, payload, sizeof(di_fifo[slot].payload) - 1);
            di_fifo[slot].payload[sizeof(di_fifo[slot].payload) - 1] = '\0';
            di_tail = (di_tail + 1) % DI_FIFO_SIZE;
            if (di_count < DI_FIFO_SIZE) {
                di_count++;
            } else {
                /* Overwrite oldest */
                di_head = (di_head + 1) % DI_FIFO_SIZE;
            }
            xSemaphoreGive(di_mutex);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2000)); /* poll every 2 s */
    }
}

void di_publish_buffer(const char *topic, const char *hw_id,
                       const char *ts_latest, uint8_t di_enable) {
    if (!di_enable || !di_mutex) return;

    for (;;) {
        char entry_payload[256];
        bool got = false;

        if (xSemaphoreTake(di_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (di_count > 0) {
                strncpy(entry_payload, di_fifo[di_head].payload,
                        sizeof(entry_payload) - 1);
                entry_payload[sizeof(entry_payload) - 1] = '\0';
                di_head = (di_head + 1) % DI_FIFO_SIZE;
                di_count--;
                got = true;
            }
            xSemaphoreGive(di_mutex);
        }

        if (!got) break;

        /* Wrap in envelope with HW_ID and TS */
        char msg[400];
        snprintf(msg, sizeof(msg),
                 "{\"HW_ID\":\"%s\",\"di\":%s,\"TS\":\"%s\"}",
                 hw_id, entry_payload, ts_latest);
        mqtt_publish(topic, msg);
    }
}
