#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* ── Device diagnostics (diagnostics + system monitor, merged) ──────────────
 * This module is the old diagnostics.c + sysmon.c combined into one. It owns the
 * shared runtime counters (g_diag), the on-chip temperature sensor and the CPU
 * power-management lock, and it publishes ONE "diagnostics" report per window
 * (DIAG_CYCLE_INTERVAL ≈ 15 min) instead of the two separate "diagnostics" +
 * "sysmon" messages that used to go out.
 *
 * g_diag holds the live counters every subsystem writes into during a poll
 * cycle (publish success/fail, worst loop time, and each subsystem's ok flag).
 * diag_publish() reports them and clears the window on a confirmed publish. */
typedef struct {
    uint32_t pub_ok;         /* publishes accepted this window                 */
    uint32_t pub_fail;       /* publishes rejected this window                 */
    uint32_t loop_max_ms;    /* worst poll-cycle duration this window          */
    bool     grace_over;     /* boot warm-up finished (loop_max_ms armed)      */
    int64_t  grace_start_us; /* esp_timer time diag_init() ran                 */
    bool     du_ok;          /* last dispenser round: all nozzles read+pub ok  */
    bool     radar_ok;       /* last radar round: all tanks read+pub ok        */
    bool     genset_ok;      /* last genset poll: read+pub ok                  */
    bool     di_ok;          /* last DI drain: all queued entries published    */
    bool     gps_ok;         /* last GPS poll: fix read+pub ok                 */
} DiagState_t;

extern DiagState_t g_diag;

/* Call once from app_main (after nvs_flash_init): zeroes g_diag, records the
 * boot-grace start, latches reset reason + boot count, installs the temperature
 * sensor and pins the CPU frequency. Combines the old diag_init + sysmon_init. */
void diag_init(void);

/* CPU-frequency lock, taken/released around each poll cycle. Inert no-ops today
 * (CPU is pinned at a fixed 160 MHz) but kept as the hook if DFS is ever used. */
void diag_pm_acquire(void);
void diag_pm_release(void);

/* Feed the per-cycle loop duration; tracks the window maximum after boot grace. */
void diag_record_loop_time(int64_t iter_us);

/* Publish the single combined diagnostics report to `topic` and clear the window
 * counters on a confirmed publish. Replaces diag_publish() + sysmon_publish(). */
void diag_publish(const char *hw_id, const char *ts, const char *topic);
