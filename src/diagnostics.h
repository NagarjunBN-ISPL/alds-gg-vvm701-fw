#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t pub_ok;
    uint32_t pub_fail;
    uint32_t loop_max_ms;
    bool     grace_over;
    int64_t  grace_start_us;
    int      cycle_count;
    bool     du_ok;
    bool     radar_ok;
    bool     genset_ok;
    bool     di_ok;
    bool     gps_ok;
} DiagState_t;

extern DiagState_t g_diag;

void diag_init(void);
void diag_record_loop_time(int64_t iter_us);
void diag_publish(const char *hw_id, const char *ts,
                  const char *topic);
