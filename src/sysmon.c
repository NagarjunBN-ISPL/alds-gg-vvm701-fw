#include "sysmon.h"
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
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "sysmon";

static temperature_sensor_handle_t s_temp_hdl    = NULL;
static esp_pm_lock_handle_t        s_pm_lock      = NULL;
static int                         s_reset_reason = 0;
static uint32_t                    s_boot_count   = 0;
/* The CPU frequency held while the ESP_PM_CPU_FREQ_MAX lock is taken (i.e.
 * during every poll cycle, which is exactly when sysmon samples). Set from the
 * PM config below; left at the compiled default if DFS fails to engage. */
static uint32_t                    s_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

void sysmon_init(void) {
    s_reset_reason = (int)esp_reset_reason();
    /* One bump per boot — sysmon_init() is called exactly once from app_main,
     * after nvs_flash_init(). */
    s_boot_count   = nvs_boot_count_bump();

    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&tcfg, &s_temp_hdl) == ESP_OK)
        temperature_sensor_enable(s_temp_hdl);

    /* FIXED 160 MHz — no dynamic frequency switching. max == min pins the CPU:
     * there is no range to scale across, so no DFS transitions ever occur.
     * 160 is a valid PLL-derived frequency (480/3) on the ESP32-S3, so the
     * 80 MHz APB clock that feeds the modem / RS485 / DI UARTs is unchanged —
     * every UART baud rate stays exact. (180 MHz is not selectable: the PLL
     * only divides to 240/160/80. Below 160 would fall to an XTAL-derived CPU
     * clock, which drops APB and corrupts UART timing — hence never below 160.)
     * This workload is I/O-bound (the cores spend their life blocked on UART
     * reads), so flat 160 costs only a marginally slower TLS handshake per
     * reconnect while running cooler, at lower peak current, with no DFS
     * transition events. light_sleep stays off: three UARTs are always live. */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz       = 160,
        .min_freq_mhz       = 160,
        .light_sleep_enable = false,
    };
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err == ESP_OK) {
        s_cpu_freq_mhz = pm_cfg.max_freq_mhz;   /* fixed 160 */
        /* Lock is now inert (max == min → acquiring it changes nothing), but
         * kept so sysmon_pm_acquire/release remain valid no-ops. */
        esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_max", &s_pm_lock);
    } else {
        /* DFS didn't engage (e.g. CONFIG_PM_ENABLE off) — CPU stays pinned at
         * the compiled default, which is still >= 160. Surface it rather than
         * silently losing the 240 MHz boost during polling. */
        ESP_LOGW(TAG, "esp_pm_configure failed (%s) — DFS off, CPU fixed at %d MHz",
                 esp_err_to_name(pm_err), CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    }
}

void sysmon_pm_acquire(void) { if (s_pm_lock) esp_pm_lock_acquire(s_pm_lock); }
void sysmon_pm_release(void) { if (s_pm_lock) esp_pm_lock_release(s_pm_lock); }

float sysmon_get_temp_c(void) {
    float t = 0.0f;
    if (s_temp_hdl) temperature_sensor_get_celsius(s_temp_hdl, &t);
    return t;
}

const char *sysmon_thermal_zone(float temp_c) {
    if (temp_c >= THERMAL_CRITICAL_MIN_C) return "critical";
    if (temp_c >= THERMAL_WARM_MAX_C)     return "hot";
    if (temp_c >= THERMAL_NORMAL_MAX_C)   return "warm";
    return "normal";
}

void sysmon_sample(SysmonSnapshot_t *out) {
    memset(out, 0, sizeof(*out));

    if (s_temp_hdl)
        temperature_sensor_get_celsius(s_temp_hdl, &out->cpu_temp_c);

    out->heap_free          = esp_get_free_heap_size();
    out->heap_min_ever      = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    out->heap_largest_block = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    /* CPU is pinned at a fixed 160 MHz (no DFS — see sysmon_init()), so
     * s_cpu_freq_mhz is simply that configured value, always accurate.
     * Deliberately not using the live-frequency API esp_clk_cpu_freq(),
     * which lives in an unsupported private header. */
    out->cpu_freq_mhz       = s_cpu_freq_mhz;
    out->uptime_s           = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    out->reset_reason       = s_reset_reason;
    out->boot_count         = s_boot_count;

    out->rssi_csq = modem_get_csq();
    modem_get_vbat(&out->vbat_mv);
    modem_get_operator(out->operator_name, sizeof(out->operator_name));

    const esp_partition_t *part = esp_ota_get_running_partition();
    if (part)
        strncpy(out->ota_label, part->label, sizeof(out->ota_label) - 1);

    out->main_stack_hwm = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    out->di_stack_hwm   = g_di_task_handle
                          ? (uint32_t)uxTaskGetStackHighWaterMark(g_di_task_handle)
                          : 0;

    out->pub_ok      = g_diag.pub_ok;
    out->pub_fail    = g_diag.pub_fail;
    out->loop_max_ms = g_diag.loop_max_ms;
    out->du_ok       = g_diag.du_ok;
    out->radar_ok    = g_diag.radar_ok;
    out->genset_ok   = g_diag.genset_ok;
    out->di_ok       = g_diag.di_ok;
}

void sysmon_publish(const char *hw_id, const char *ts, const char *topic) {
    SysmonSnapshot_t s;
    sysmon_sample(&s);

    char buf[768];
    int written = snprintf(buf, sizeof(buf),
        "{"
        "\"HW_ID\":\"%s\","
        "\"fw\":\"%s\","
        "\"cpu_temp_c\":%.1f,"
        "\"thermal_zone\":\"%s\","
        "\"cpu_freq_mhz\":%lu,"
        "\"heap_free\":%lu,"
        "\"heap_min_ever\":%lu,"
        "\"heap_largest_block\":%lu,"
        "\"uptime_s\":%lu,"
        "\"reset_reason\":%d,"
        "\"boot_count\":%lu,"
        "\"rssi\":%d,"
        "\"vbat_mv\":%d,"
        "\"operator\":\"%s\","
        "\"ota_partition\":\"%s\","
        "\"main_stack_hwm\":%lu,"
        "\"di_stack_hwm\":%lu,"
        "\"pub_ok\":%lu,"
        "\"pub_fail\":%lu,"
        "\"loop_max_ms\":%lu,"
        "\"du_ok\":%s,"
        "\"radar_ok\":%s,"
        "\"genset_ok\":%s,"
        "\"di_ok\":%s,"
        "\"TS\":\"%s\""
        "}",
        hw_id, FW_VERSION,
        s.cpu_temp_c,
        sysmon_thermal_zone(s.cpu_temp_c),
        (unsigned long)s.cpu_freq_mhz,
        (unsigned long)s.heap_free,
        (unsigned long)s.heap_min_ever,
        (unsigned long)s.heap_largest_block,
        (unsigned long)s.uptime_s,
        s.reset_reason,
        (unsigned long)s.boot_count,
        s.rssi_csq, s.vbat_mv,
        s.operator_name,
        s.ota_label,
        (unsigned long)s.main_stack_hwm,
        (unsigned long)s.di_stack_hwm,
        (unsigned long)s.pub_ok,
        (unsigned long)s.pub_fail,
        (unsigned long)s.loop_max_ms,
        s.du_ok     ? "true" : "false",
        s.radar_ok  ? "true" : "false",
        s.genset_ok ? "true" : "false",
        s.di_ok     ? "true" : "false",
        ts);

    /* snprintf's return value is the length it WOULD have written, not what
     * actually fit — a single call like this can never overflow buf (unlike
     * the chained snprintf+clamp_pos pattern in genset.c/dispenser.c), but it
     * can still silently truncate into invalid JSON if some field (e.g. an
     * unexpectedly long operator name) pushes the total over 768 bytes. */
    if (written >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "payload truncated at %d/%zu bytes — publishing minimal report instead",
                 written, sizeof(buf));
        snprintf(buf, sizeof(buf),
                 "{\"HW_ID\":\"%s\",\"fw\":\"%s\",\"error\":\"payload_truncated\",\"TS\":\"%s\"}",
                 hw_id, FW_VERSION, ts);
    }

    mqtt_publish(topic, buf);
}
