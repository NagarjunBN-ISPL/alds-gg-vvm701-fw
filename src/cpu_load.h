#pragma once
#include <stdbool.h>
#include <stdint.h>

/*
 * cpu_load.h — per-core CPU load for the diagnostics report.
 *
 * FreeRTOS run-time stats (CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS, 64-bit
 * esp_timer counter, so it never wraps) accumulate how long every task has
 * run, in microseconds since boot, plus a total that is wall time since boot —
 * each core's budget. A core's load over an interval is the share of it that
 * core's idle task did NOT get. ESP-IDF names the idle tasks "IDLE0" and
 * "IDLE1" (tasks.c, prvCreateIdleTasks); TaskStatus_t.xCoreID needs a config
 * that is off, so the core is read from the name.
 *
 * What the report carries, per core:
 *   avg    over the report's own window — since the previous report was
 *          PUBLISHED (about 15 min, DIAG_CYCLE_INTERVAL), like every other
 *          windowed counter in it. Not a fixed last minute: hsd-snf's console
 *          line always read "over the last 62 s" because that was all it
 *          measured. A report that fails to publish keeps its window open, and
 *          window_s says how long the next one covers.
 *   peak   the busiest one-minute sample inside that window, so a burst (an
 *          offline-queue compaction, a FOTA download) is not averaged away.
 *
 * Pure — no FreeRTOS here; diagnostics.c reads the task table. Host-tested
 * (test/host/test_cpu_load.c).
 */

#define CPU_CORES 2

typedef struct {
    uint64_t total_us;              /* run-time counter total = wall time   */
    uint64_t idle_us[CPU_CORES];    /* each core's idle task run time       */
} CpuSnap_t;

/* Folds one task into a snapshot: its run time is added to a core's idle total
 * when the name is that core's idle task ("IDLE0", "IDLE1"), else ignored. */
void cpu_snap_add_task(CpuSnap_t *s, const char *name, uint64_t run_us);

/* Busy share of `core` from `a` to `b`, whole percent 0..100 (rounded), or -1
 * when there is nothing to measure: `b` not later than `a`, or an idle counter
 * that went backwards (not the same boot). */
int cpu_load_pct(const CpuSnap_t *a, const CpuSnap_t *b, int core);

typedef struct {
    bool      open;                 /* a window has been started            */
    CpuSnap_t start;                /* where the report window began        */
    CpuSnap_t last;                 /* the previous one-minute sample       */
    int       peak[CPU_CORES];      /* busiest sample so far, -1 = none yet */
} CpuWindow_t;

typedef struct {
    uint32_t window_s;
    int      avg[CPU_CORES];
    int      peak[CPU_CORES];
} CpuReport_t;

/* Starts a window at `now`: at boot, and again once a report is published. */
void cpu_window_begin(CpuWindow_t *w, const CpuSnap_t *now);

/* A one-minute sample: the load since the previous sample can raise the peak.
 * Starts the window if none is open. */
void cpu_window_sample(CpuWindow_t *w, const CpuSnap_t *now);

/* The report for the window so far, ending at `now`. The peak is the busiest
 * completed sample, and never below the window's own average (a window shorter
 * than one sample has no samples: its peak is its average). False when there is
 * nothing to report — no window, or no time has passed; `out` is then
 * untouched. Does not end the window: call cpu_window_begin() once the report
 * carrying it has been published. */
bool cpu_window_report(const CpuWindow_t *w, const CpuSnap_t *now, CpuReport_t *out);
