#include "rs485_bus.h"
#include "config.h"
#include "dispenser.h"
#include "yenen.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "rs485";

static bool              s_inited = false;
static SemaphoreHandle_t s_lock   = NULL;
/* Failed lock acquisitions since boot — see rs485_bus_lock(). */
static uint32_t          s_lock_timeouts = 0;

void rs485_bus_init(void)
{
    if (s_inited) return;

    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t err = uart_driver_install(RS485_UART, RS485_BUF, RS485_BUF, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
    uart_param_config(RS485_UART, &cfg);
    uart_set_pin(RS485_UART, RS485_TX_PIN, RS485_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* xSemaphoreCreateMutex, NOT a binary semaphore: this one has priority
     * inheritance. du_task runs above the main task, so without inheritance a
     * main-task radar read holding the bus could be preempted indefinitely by
     * mid-priority work while du_task waits on it — classic priority
     * inversion, and it would show up as unexplained dispenser 404s. */
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock)
        ESP_LOGE(TAG, "bus mutex alloc failed — RS485 will be unarbitrated");

    s_inited = true;
    ESP_LOGI(TAG, "RS485 bus ready: UART%d TX=GPIO%d RX=GPIO%d",
             RS485_UART, RS485_TX_PIN, RS485_RX_PIN);
}

bool rs485_bus_lock(uint32_t timeout_ms)
{
    /* No mutex (alloc failed at boot) means degrade to the previous
     * single-owner behaviour rather than refusing every read: an unarbitrated
     * bus still mostly works, a permanently locked-out poll loop does not. */
    if (!s_lock) return true;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) return true;
    /* A timeout here is never harmless: for du_task it means a dispenser round
     * was skipped (two sales merge into one totalizer delta), for the main task
     * it means a radar/genset poll or a remote command was dropped. Counting
     * them turns "the bus is sometimes contended" into a number the fleet can
     * see — nothing else in telemetry reveals it. */
    s_lock_timeouts++;
    return false;
}

uint32_t rs485_bus_lock_timeouts(void) { return s_lock_timeouts; }

void rs485_bus_unlock(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

/* ── Line modes ───────────────────────────────────────────────────────────
 * Moved here verbatim from main.c so both the main task and du_task reach
 * the same implementations. Behaviour is unchanged. */

void rs485_bus_set_8n1(uint32_t baud)
{
    uart_wait_tx_idle_polling(RS485_UART);
    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(RS485_UART, &cfg);
}

void rs485_bus_set_bluesky(void)
{
    uart_wait_tx_idle_polling(RS485_UART);
    du_rs485_set_bluesky_mode();
}

void rs485_bus_set_yenen(void)
{
    uart_wait_tx_idle_polling(RS485_UART);
    yn_rs485_set_yenen_mode();
}
