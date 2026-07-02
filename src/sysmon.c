#include "sysmon.h"
#include "diagnostics.h"
#include "di.h"
#include "modem.h"
#include "mqtt.h"
#include "config.h"
#include "driver/temperature_sensor.h"
#include "esp_pm.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_private/esp_clk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static temperature_sensor_handle_t s_temp_hdl    = NULL;
static esp_pm_lock_handle_t        s_pm_lock      = NULL;
static int                         s_reset_reason = 0;

void sysmon_init(void) {
    s_reset_reason = (int)esp_reset_reason();

    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&tcfg, &s_temp_hdl) == ESP_OK)
        temperature_sensor_enable(s_temp_hdl);

    esp_pm_config_t pm_cfg = {
        .max_freq_mhz       = 240,
        .min_freq_mhz       = 160,
        .light_sleep_enable = false,
    };
    if (esp_pm_configure(&pm_cfg) == ESP_OK)
        esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_max", &s_pm_lock);
}

void sysmon_pm_acquire(void) { if (s_pm_lock) esp_pm_lock_acquire(s_pm_lock); }
void sysmon_pm_release(void) { if (s_pm_lock) esp_pm_lock_release(s_pm_lock); }

float sysmon_get_temp_c(void) {
    float t = 0.0f;
    if (s_temp_hdl) temperature_sensor_get_celsius(s_temp_hdl, &t);
    return t;
}

void sysmon_sample(SysmonSnapshot_t *out) {
    memset(out, 0, sizeof(*out));

    if (s_temp_hdl)
        temperature_sensor_get_celsius(s_temp_hdl, &out->cpu_temp_c);

    out->heap_free          = esp_get_free_heap_size();
    out->heap_min_ever      = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    out->heap_largest_block = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    out->cpu_freq_mhz       = (uint32_t)(esp_clk_cpu_freq() / 1000000);
    out->uptime_s           = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    out->reset_reason       = s_reset_reason;

    out->rssi_csq = modem_get_csq();
    modem_get_vbat(&out->vbat_mv);

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
    snprintf(buf, sizeof(buf),
        "{"
        "\"HW_ID\":\"%s\","
        "\"fw\":\"%s\","
        "\"cpu_temp_c\":%.1f,"
        "\"cpu_freq_mhz\":%lu,"
        "\"heap_free\":%lu,"
        "\"heap_min_ever\":%lu,"
        "\"heap_largest_block\":%lu,"
        "\"uptime_s\":%lu,"
        "\"reset_reason\":%d,"
        "\"rssi\":%d,"
        "\"vbat_mv\":%d,"
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
        (unsigned long)s.cpu_freq_mhz,
        (unsigned long)s.heap_free,
        (unsigned long)s.heap_min_ever,
        (unsigned long)s.heap_largest_block,
        (unsigned long)s.uptime_s,
        s.reset_reason,
        s.rssi_csq, s.vbat_mv,
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

    mqtt_publish(topic, buf);
}
