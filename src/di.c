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

static const char *TAG = "di";

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

    /* di_mutex is created in di_uart_init(); if that xSemaphoreCreateMutex()
     * failed (heap exhaustion at boot), di_mutex is still NULL here. Every
     * other user of it (di_publish_buffer) already guards with `if
     * (!di_mutex)`, but this task calls xSemaphoreTake(di_mutex, ...)
     * unconditionally below — passing NULL to a FreeRTOS semaphore call is
     * undefined behaviour, typically an abort/hard fault. Fail loudly and
     * exit the task cleanly instead. */
    if (!di_mutex) {
        ESP_LOGE(TAG, "di_mutex is NULL (heap OOM at boot?) — di_task exiting, DI monitoring disabled");
        vTaskDelete(NULL);
        return;
    }

    esp_task_wdt_add(NULL);
    di_enable_mask = s_di_enable;

    static char req[]  = "GETDATA\n";
    static char resp[NUVOTON_BUF];
    static char last_payload[256] = "";  /* dedup: only enqueue on change */

    for (;;) {
        /* Refresh enable mask in case config was updated */
        di_enable_mask = s_di_enable;

        /* Send request */
        uart_write_bytes(NUVOTON_UART, req, strlen(req));
        vTaskDelay(pdMS_TO_TICKS(50));

        /* Read response — byte-by-byte with an idle-gap cutoff, not one
         * uart_read_bytes(..., NUVOTON_BUF-1, timeout) call as this used to
         * be. That single-call form requests up to 255 bytes for a ~105-byte
         * reply; esp-idf's uart_read_bytes() re-issues the FULL timeout on
         * every partial ring-buffer fetch (see uart.c's while(length) loop),
         * so once the real bytes were drained it kept blocking the entire
         * remaining NUVOTON_TIMEOUT_MS waiting for bytes that would never
         * arrive — turning every ~2s poll into a ~5.1s one (confirmed against
         * a field log: 7 consecutive di_task iterations at exactly 5140ms).
         * Here the "wait for a response to start at all" timeout stays the
         * full NUVOTON_TIMEOUT_MS (a genuinely offline Nuvoton still gets a
         * fair wait), but once the first byte lands, a 100ms gap with no
         * further byte (~96 byte-times at 9600 baud) is treated as end of
         * message. */
        int got = 0;
        int64_t dl_first = esp_timer_get_time() + (int64_t)NUVOTON_TIMEOUT_MS * 1000;
        int64_t dl_gap    = 0;
        for (;;) {
            int64_t dl = (got == 0) ? dl_first : dl_gap;
            if (esp_timer_get_time() >= dl || got >= NUVOTON_BUF - 1) break;
            uint8_t ch;
            if (uart_read_bytes(NUVOTON_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
                resp[got++] = ch;
                dl_gap = esp_timer_get_time() + 100000LL;   /* 100 ms idle → done */
            }
        }
        resp[got] = '\0';

        /* Raw Nuvoton response for field debugging — shows exactly what the
         * MS51 sent every read (every 2s), before parsing. */
        if (got > 0) ESP_LOGI(TAG, "Nuvoton RAW (%d bytes): %s", got, resp);
        else         ESP_LOGI(TAG, "Nuvoton RAW: no response");

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

        /* Digital inputs are event-driven: only enqueue when the reading
         * actually changes from the last one queued. This captures every
         * transition (e.g. a pulse between 30s publish cycles) without
         * flooding the cloud with dozens of identical steady-state messages
         * — which was also starving the modem UART of time to receive
         * inbound commands like setconfig. */
        if (strcmp(payload, last_payload) != 0) {
            /* last_payload is the dedup key — it must only advance once the
             * transition is ACTUALLY enqueued. Updating it unconditionally
             * here (as this used to) meant a mutex-take timeout silently
             * dropped the transition AND marked it "already seen", so an
             * identical next reading would never retry it — the change was
             * gone for good. Only commit last_payload inside the success
             * branch below, so a timeout leaves dedup state untouched and
             * the very next poll (2s later) naturally retries the enqueue. */
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

                strncpy(last_payload, payload, sizeof(last_payload) - 1);
                last_payload[sizeof(last_payload) - 1] = '\0';
            }
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2000)); /* poll every 2 s */
    }
}

bool di_publish_buffer(const char *topic, const char *hw_id,
                       const char *ts_latest, uint8_t di_enable) {
    if (!di_enable || !di_mutex) return false;

    bool all_ok = true;  /* stays true if nothing to drain (DI healthy, idle) */
    for (;;) {
        char entry_payload[256];
        bool got = false;

        /* Peek the head entry WITHOUT removing it — it's only popped after a
         * confirmed successful publish below. Popping first (as this used to)
         * meant a failed publish permanently lost that DI transition: the
         * entry was already gone from the FIFO by the time mqtt_publish()
         * told us it didn't make it to the broker. */
        if (xSemaphoreTake(di_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (di_count > 0) {
                strncpy(entry_payload, di_fifo[di_head].payload,
                        sizeof(entry_payload) - 1);
                entry_payload[sizeof(entry_payload) - 1] = '\0';
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

        /* What actually goes to the cloud — distinct from the "Nuvoton RAW"
         * log in di_task(), which is the unfiltered sensor reply before the
         * di_enable_mask filter and envelope wrap. mqtt_publish() itself only
         * logs this body at DEBUG (suppressed) — that suppression exists to
         * stop the OTHER sensors' 30s-periodic multi-KB payloads from
         * flooding the console, which doesn't apply here: DI publishes only
         * on a real transition, so logging every one at INFO is cheap. */
        ESP_LOGI(TAG, "DI PUBLISH %s: %s", topic, msg);

        if (!mqtt_publish(topic, msg)) {
            /* Leave the entry in the FIFO for the next poll cycle to retry —
             * and stop draining now rather than hammering a link that just
             * failed once, which would just fail the whole backlog too. */
            all_ok = false;
            break;
        }

        /* Publish confirmed — now it's safe to actually pop the entry. */
        if (xSemaphoreTake(di_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            di_head = (di_head + 1) % DI_FIFO_SIZE;
            if (di_count > 0) di_count--;
            xSemaphoreGive(di_mutex);
        }
    }
    return all_ok;
}
