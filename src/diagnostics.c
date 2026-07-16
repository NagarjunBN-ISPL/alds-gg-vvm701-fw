#include "diagnostics.h"
#include "sysmon.h"
#include "modem.h"
#include "mqtt.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "diag";

DiagState_t g_diag;

void diag_init(void) {
    memset(&g_diag, 0, sizeof(g_diag));
    g_diag.grace_start_us = esp_timer_get_time();
}

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

void diag_publish(const char *hw_id, const char *ts, const char *topic) {
    float    temp_c   = sysmon_get_temp_c();
    int      csq      = modem_get_csq();
    int      vbat_mv  = 0;
    modem_get_vbat(&vbat_mv);
    char     operator_name[24] = "";
    modem_get_operator(operator_name, sizeof(operator_name));

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t uptime_s  = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    char buf[512];
    int written = snprintf(buf, sizeof(buf),
        "{\"HW_ID\":\"%s\","
        "\"type\":\"diagnostics\","
        "\"fw\":\"%s\","
        "\"temp_c\":%.1f,"
        "\"thermal_zone\":\"%s\","
        "\"heap_free\":%lu,"
        "\"rssi\":%d,"
        "\"vbat_mv\":%d,"
        "\"operator\":\"%s\","
        "\"uptime_s\":%lu,"
        "\"pub_ok\":%lu,"
        "\"pub_fail\":%lu,"
        "\"loop_max_ms\":%lu,"
        "\"du_ok\":%s,"
        "\"radar_ok\":%s,"
        "\"genset_ok\":%s,"
        "\"di_ok\":%s,"
        "\"TS\":\"%s\"}",
        hw_id, FW_VERSION,
        temp_c,
        sysmon_thermal_zone(temp_c),
        (unsigned long)free_heap,
        csq, vbat_mv,
        operator_name,
        (unsigned long)uptime_s,
        (unsigned long)g_diag.pub_ok,
        (unsigned long)g_diag.pub_fail,
        (unsigned long)g_diag.loop_max_ms,
        g_diag.du_ok     ? "true" : "false",
        g_diag.radar_ok  ? "true" : "false",
        g_diag.genset_ok ? "true" : "false",
        g_diag.di_ok     ? "true" : "false",
        ts);

    if (written >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "payload truncated at %d/%zu bytes — publishing minimal report instead",
                 written, sizeof(buf));
        snprintf(buf, sizeof(buf),
                 "{\"HW_ID\":\"%s\",\"fw\":\"%s\",\"error\":\"payload_truncated\",\"TS\":\"%s\"}",
                 hw_id, FW_VERSION, ts);
    }

    /* Only clear the accumulated window on a CONFIRMED publish — if MQTT is
     * briefly down exactly at this cycle boundary, the pub_ok/pub_fail/
     * loop_max_ms stats for the whole DIAG_CYCLE_INTERVAL window would
     * otherwise be silently discarded and the next report starts from zero,
     * under-reporting the failure rate right when it matters most (a
     * connectivity blip). Left uncleared, they keep accumulating into the
     * next attempt instead. */
    if (mqtt_publish(topic, buf)) {
        ESP_LOGI(TAG, "Diagnostics published");
        g_diag.pub_ok      = 0;
        g_diag.pub_fail    = 0;
        g_diag.loop_max_ms = 0;
    } else {
        ESP_LOGW(TAG, "Diagnostics publish failed — counters retained for next attempt");
    }
}
