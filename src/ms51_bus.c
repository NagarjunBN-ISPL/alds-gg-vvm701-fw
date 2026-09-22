#include "ms51_bus.h"
#include "config.h"
#include "driver/uart.h"
#include "driver/gpio.h"   /* GPIO_NUM_* used by NUVOTON_TX_PIN / _RX_PIN */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "ms51";

static bool              s_inited = false;
static SemaphoreHandle_t s_lock   = NULL;

void ms51_bus_init(void)
{
    if (s_inited) return;          /* idempotent — see header */

    uart_config_t cfg = {
        .baud_rate = NUVOTON_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    /* Not ESP_ERROR_CHECK: a driver already installed by an earlier caller is
     * the normal path here, not a fault. Any other error is logged and left
     * non-fatal — losing the MS51 link must degrade DI and the lock, never
     * panic a gateway whose dispenser polling is otherwise fine. */
    esp_err_t err = uart_driver_install(NUVOTON_UART, NUVOTON_BUF, NUVOTON_BUF, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));

    uart_param_config(NUVOTON_UART, &cfg);
    uart_set_pin(NUVOTON_UART, NUVOTON_TX_PIN, NUVOTON_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* Pull the RX line up. When the MS51 is in reset or between transmissions
     * its TX can go high-impedance; a floating RX in this high-EMI enclosure
     * (pump contactors, compressor motors) picks up noise the UART reads as
     * spurious start bits / framing errors — bytes that then stretch a DI read
     * and can misalign its parser. A UART line idles HIGH, so an idle-high pull
     * is the correct rest state and is harmless when the MS51 is actively
     * driving. uart_set_pin() does not set this on ESP32-S3, so do it explicitly
     * (after uart_set_pin, which configures the matrix first). */
    gpio_set_pull_mode(NUVOTON_RX_PIN, GPIO_PULLUP_ONLY);

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock)
        ESP_LOGE(TAG, "transaction mutex alloc failed — DI and relay will "
                      "share UART%d unarbitrated", NUVOTON_UART);

    s_inited = true;
    ESP_LOGI(TAG, "MS51 bus ready: UART%d TX=GPIO%d RX=GPIO%d @%d",
             NUVOTON_UART, NUVOTON_TX_PIN, NUVOTON_RX_PIN, NUVOTON_BAUD);
}

bool ms51_bus_lock(uint32_t timeout_ms)
{
    if (!s_lock) return true;      /* proceed unarbitrated — see header */
    return xSemaphoreTake(s_lock, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void ms51_bus_unlock(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}
