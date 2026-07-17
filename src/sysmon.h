#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    cpu_temp_c;
    uint32_t heap_free;
    uint32_t heap_min_ever;
    uint32_t heap_largest_block;
    uint32_t cpu_freq_mhz;
    uint32_t uptime_s;
    int      reset_reason;
    uint32_t boot_count;
    int      rssi_csq;
    int      vbat_mv;
    char     operator_name[24];
    char     ota_label[16];
    uint32_t main_stack_hwm;
    uint32_t di_stack_hwm;
    uint32_t pub_ok;
    uint32_t pub_fail;
    uint32_t loop_max_ms;
    bool     du_ok, radar_ok, genset_ok, di_ok, gps_ok;
} SysmonSnapshot_t;

void  sysmon_init(void);
void  sysmon_pm_acquire(void);
void  sysmon_pm_release(void);
float sysmon_get_temp_c(void);
/* "normal" / "warm" / "hot" / "critical" — see THERMAL_*_C in config.h */
const char *sysmon_thermal_zone(float temp_c);
void  sysmon_sample(SysmonSnapshot_t *out);
void  sysmon_publish(const char *hw_id, const char *ts, const char *topic);
