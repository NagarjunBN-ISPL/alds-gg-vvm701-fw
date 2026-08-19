#include "diagnostics.h"
#include "di.h"
#include "modem.h"
#include "mqtt.h"
#include "config.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "driver/temperature_sensor.h"
#include "esp_pm.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "diag";

/* Live counters written by the poll functions in main.c (pub_ok/pub_fail/
 * loop_max_ms and each subsystem's ok flag). Reported and cleared by
 * diag_publish(). */
DiagState_t g_diag;

/* Latched-once-at-boot / hardware-owned state (was sysmon.c). */
static temperature_sensor_handle_t s_temp_hdl    = NULL;
static esp_pm_lock_handle_t        s_pm_lock      = NULL;
static int                         s_reset_reason = 0;
static uint32_t                    s_boot_count   = 0;

/* ── Init (diag_init + sysmon_init merged) ──────────────────────────────── */
void diag_init(void) {
    memset(&g_diag, 0, sizeof(g_diag));
    g_diag.grace_start_us = esp_timer_get_time();

    s_reset_reason = (int)esp_reset_reason();
    /* One bump per boot — diag_init() is called exactly once from app_main,
     * after nvs_flash_init(). Lets the fleet spot a reboot loop that telemetry
     * gaps alone wouldn't reveal. */
    s_boot_count = nvs_boot_count_bump();

    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&tcfg, &s_temp_hdl) == ESP_OK)
        temperature_sensor_enable(s_temp_hdl);

    /* FIXED 160 MHz — no dynamic frequency switching. max == min pins the CPU:
     * there is no range to scale across, so no DFS transitions ever occur.
     * 160 is a valid PLL-derived frequency (480/3) on the ESP32-S3, so the
     * 80 MHz APB clock that feeds the modem / RS485 / DI UARTs is unchanged —
     * every UART baud rate stays exact. (Below 160 would fall to an XTAL-derived
     * CPU clock, dropping APB and corrupting UART timing — hence never below.)
     * This workload is I/O-bound (the cores spend their life blocked on UART
     * reads), so flat 160 runs cooler at lower peak current with no DFS events.
     * light_sleep stays off: three UARTs are always live. */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz       = 160,
        .min_freq_mhz       = 160,
        .light_sleep_enable = false,
    };
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err == ESP_OK) {
        /* Lock is inert (max == min → acquiring it changes nothing) but kept so
         * diag_pm_acquire/release stay valid no-ops. */
        esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_max", &s_pm_lock);
    } else {
        ESP_LOGW(TAG, "esp_pm_configure failed (%s) — DFS off, CPU fixed at %d MHz",
                 esp_err_to_name(pm_err), CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    }
}

void diag_pm_acquire(void) { if (s_pm_lock) esp_pm_lock_acquire(s_pm_lock); }
void diag_pm_release(void) { if (s_pm_lock) esp_pm_lock_release(s_pm_lock); }

/* ── Loop-time tracking ─────────────────────────────────────────────────── */
void diag_record_loop_time(int64_t iter_us) {
    if (!g_diag.grace_over) {
        int64_t elapsed = esp_timer_get_time() - g_diag.grace_start_us;
        if (elapsed >= (int64_t)DIAG_BOOT_GRACE_MS * 1000)
            g_diag.grace_over = true;
        else
            return;
    }
    uint32_t ms = (uint32_t)(iter_us / 1000);
    if (ms > g_diag.loop_max_ms) g_diag.loop_max_ms = ms;
}

/* ── Single combined report (diag_publish + sysmon_publish merged) ──────────
 * One message under the "diagnostics" key — the cloud already routes on it.
 * Kept lean: only fields with real fleet-ops value. Dropped vs. the old two
 * reports: cpu_freq_mhz (pinned at 160), heap_largest_block (heap_min_ever is
 * the leak signal we act on), thermal_zone (the cloud derives it from temp_c),
 * ota_partition (fota_completed already carries the running fw). The modem
 * round-trips (CSQ/VBAT/COPS) both reports used to make independently now
 * happen once. */
void diag_publish(const char *hw_id, const char *ts, const char *topic) {
    float    temp_c   = 0.0f;
    if (s_temp_hdl) temperature_sensor_get_celsius(s_temp_hdl, &temp_c);
    int      csq      = modem_get_csq();
    int      vbat_mv  = 0;
    modem_get_vbat(&vbat_mv);
    char     operator_name[24] = "";
    modem_get_operator(operator_name, sizeof(operator_name));

    uint32_t heap_free     = esp_get_free_heap_size();
    uint32_t heap_min_ever = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    uint32_t uptime_s      = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    uint32_t main_stack_hwm = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    uint32_t di_stack_hwm   = g_di_task_handle
                              ? (uint32_t)uxTaskGetStackHighWaterMark(g_di_task_handle)
                              : 0;

    /* Under the "diagnostics" key — the cloud identifies a message kind by which
     * top-level key is present (nozzles/tanks/genset/di/diagnostics). */
    char buf[640];
    int written = snprintf(buf, sizeof(buf),
        "{\"HW_ID\":\"%s\","
        "\"diagnostics\":{"
        "\"fw\":\"%s\","
        "\"uptime_s\":%lu,"
        "\"reset_reason\":%d,"
        "\"boot_count\":%lu,"
        "\"temp_c\":%.1f,"
        "\"heap_free\":%lu,"
        "\"heap_min_ever\":%lu,"
        "\"main_stack_hwm\":%lu,"
        "\"di_stack_hwm\":%lu,"
        "\"rss%i\":d,"
        "\"vbat_mv\":%d,"
        "\"operator\":\"%s\","
        "\"pub_ok\":%lu,"
        "\"pub_fail\":%lu,"
        "\"loop_max_ms\":%lu,"
        "\"du_ok\":%s,"
        "\"radar_ok\":%s,"
        "\"genset_ok\":%s,"
        "\"di_ok\":%s,"
        "\"gps_ok\":%s"
        "},"
        "\"TS\":\"%s\"}",
        hw_id, FW_VERSION,
        (unsigned long)uptime_s,
        s_reset_reason,
        (unsigned long)s_boot_count,
        temp_c,
        (unsigned long)heap_free,
        (unsigned long)heap_min_ever,
        (unsigned long)main_stack_hwm,
        (unsigned long)di_stack_hwm,
        csq, vbat_mv,
        operator_name,
        (unsigned long)g_diag.pub_ok,
        (unsigned long)g_diag.pub_fail,
        (unsigned long)g_diag.loop_max_ms,
        g_diag.du_ok     ? "true" : "false",
        g_diag.radar_ok  ? "true" : "false",
        g_diag.genset_ok ? "true" : "false",
        g_diag.di_ok     ? "true" : "false",
        g_diag.gps_ok    ? "true" : "false",
        ts);

    /* snprintf returns the length it WOULD have written; a single call can't
     * overflow buf, but an unexpectedly long operator name could truncate it
     * into invalid JSON — fall back to a guaranteed-valid minimal report. */
    if (written < 0 || written >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "payload truncated at %d/%zu bytes — publishing minimal report instead",
                 written, sizeof(buf));
        snprintf(buf, sizeof(buf),
                 "{\"HW_ID\":\"%s\",\"diagnostics\":{\"fw\":\"%s\",\"error\":\"payload_truncated\"},\"TS\":\"%s\"}",
                 hw_id, FW_VERSION, ts);
    }

    /* Clear the window counters ONLY on a confirmed publish — if MQTT is briefly
     * down at this cycle boundary, retaining them keeps the pub_ok/pub_fail/
     * loop_max_ms stats accumulating rather than under-reporting the failure
     * window right when it matters (a connectivity blip). */
    if (mqtt_publish(topic, buf)) {
        ESP_LOGI(TAG, "Diagnostics published");
        g_diag.pub_ok      = 0;
        g_diag.pub_fail    = 0;
        g_diag.loop_max_ms = 0;
    } else {
        ESP_LOGW(TAG, "Diagnostics publish failed — counters retained for next attempt");
    }
}
