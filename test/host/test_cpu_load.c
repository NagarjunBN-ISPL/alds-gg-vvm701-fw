/* The REAL src/cpu_load.c: per-core CPU load as the diagnostics report carries
 * it — the average over the report's own window (since the last PUBLISHED
 * report) and the busiest one-minute sample inside it.
 *
 * The task table is simulated: each core's idle task gets whatever share of
 * the time the scenario says, and the snapshot is built through
 * cpu_snap_add_task() from task names exactly as uxTaskGetSystemState()
 * reports them on the ESP32-S3. */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "cpu_load.h"

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

/* The simulated gateway: wall time since boot and each core's idle time. */
static uint64_t s_wall_us, s_idle_us[CPU_CORES];

/* Runs for `s` seconds with each core `busy0` / `busy1` percent busy. */
static void run(uint64_t s, unsigned busy0, unsigned busy1) {
    uint64_t us = s * 1000000ULL;
    s_wall_us    += us;
    s_idle_us[0] += us * (100 - busy0) / 100;
    s_idle_us[1] += us * (100 - busy1) / 100;
}

/* The snapshot diagnostics.c would build: the whole task table, idle tasks
 * among the rest, names as FreeRTOS gives them. */
static CpuSnap_t snap(void) {
    CpuSnap_t s;
    memset(&s, 0, sizeof(s));
    s.total_us = s_wall_us;
    cpu_snap_add_task(&s, "main",      s_wall_us / 3);
    cpu_snap_add_task(&s, "du_task",   s_wall_us / 7);
    cpu_snap_add_task(&s, "IDLE0",     s_idle_us[0]);
    cpu_snap_add_task(&s, "IDLE1",     s_idle_us[1]);
    cpu_snap_add_task(&s, "esp_timer", 12345);
    cpu_snap_add_task(&s, "ipc0",      777);
    return s;
}

int main(void) {
    printf("[1] which tasks count as a core's idle task\n");
    {
        CpuSnap_t s;
        memset(&s, 0, sizeof(s));
        cpu_snap_add_task(&s, "IDLE0", 100);
        cpu_snap_add_task(&s, "IDLE1", 200);
        cpu_snap_add_task(&s, "IDLE", 1000);       /* no core digit        */
        cpu_snap_add_task(&s, "IDLE2", 1000);      /* no such core here    */
        cpu_snap_add_task(&s, "IDLE01", 1000);     /* not an idle task name */
        cpu_snap_add_task(&s, "idle0", 1000);      /* case matters         */
        cpu_snap_add_task(&s, "IDLEX", 1000);
        cpu_snap_add_task(&s, NULL, 1000);
        CHECK(s.idle_us[0] == 100 && s.idle_us[1] == 200,
              "idle totals %llu/%llu, want 100/200", (unsigned long long)s.idle_us[0],
              (unsigned long long)s.idle_us[1]);
    }

    printf("[2] load between two snapshots\n");
    {
        CpuSnap_t a = { 1000000, { 500000, 500000 } };
        CpuSnap_t b = { 2000000, { 1500000, 500000 } };   /* core0 all idle, core1 all busy */
        CHECK(cpu_load_pct(&a, &b, 0) == 0,   "an idle core reads %d %%", cpu_load_pct(&a, &b, 0));
        CHECK(cpu_load_pct(&a, &b, 1) == 100, "a busy core reads %d %%", cpu_load_pct(&a, &b, 1));
        CpuSnap_t c = { 3000000, { 2250000, 1166667 } };  /* 25 % / 33.3 % busy */
        CHECK(cpu_load_pct(&b, &c, 0) == 25, "25 %% busy reads %d", cpu_load_pct(&b, &c, 0));
        CHECK(cpu_load_pct(&b, &c, 1) == 33, "33.3 %% busy reads %d", cpu_load_pct(&b, &c, 1));
        CpuSnap_t d = { 3000000, { 1750000, 1166666 } };
        CpuSnap_t e = { 3000010, { 1750030, 1166676 } };  /* idle 30 µs in 10 µs: read skew */
        CHECK(cpu_load_pct(&d, &e, 0) == 0, "idle beyond wall time reads %d, want 0", cpu_load_pct(&d, &e, 0));
        CHECK(cpu_load_pct(&a, &a, 0) == -1, "no elapsed time must not divide by zero");
        CHECK(cpu_load_pct(&b, &a, 0) == -1, "time going backwards must not be measured");
        CpuSnap_t f = { 4000000, { 100, 2000000 } };      /* idle fell: not the same boot */
        CHECK(cpu_load_pct(&c, &f, 0) == -1, "an idle counter going backwards must not be measured");
        CHECK(cpu_load_pct(&a, &b, 2) == -1 && cpu_load_pct(&a, &b, -1) == -1, "no such core");
    }

    printf("[3] the report window: average over it, busiest minute in it\n");
    {
        CpuWindow_t w;
        memset(&w, 0, sizeof(w));
        CpuReport_t r;
        CpuSnap_t now = snap();
        CHECK(!cpu_window_report(&w, &now, &r), "a window that was never opened reported");

        cpu_window_begin(&w, &now);                        /* boot */
        CHECK(!cpu_window_report(&w, &now, &r), "a window with no time in it reported");

        /* 15 minutes: core0 10 % except one minute at 70 %; core1 steady 5 %. */
        for (int m = 0; m < 15; m++) {
            run(60, m == 7 ? 70 : 10, 5);
            now = snap();
            cpu_window_sample(&w, &now);
        }
        CHECK(cpu_window_report(&w, &now, &r), "no report after 15 min");
        CHECK(r.window_s == 900, "window %lu s, want 900", (unsigned long)r.window_s);
        CHECK(r.avg[0] == 14 && r.avg[1] == 5, "averages %d/%d, want 14/5", r.avg[0], r.avg[1]);
        CHECK(r.peak[0] == 70 && r.peak[1] == 5, "peaks %d/%d, want 70/5", r.peak[0], r.peak[1]);

        /* Published: the next window starts here and knows nothing of the burst. */
        cpu_window_begin(&w, &now);
        for (int m = 0; m < 15; m++) {
            run(60, 20, 2);
            now = snap();
            cpu_window_sample(&w, &now);
        }
        CHECK(cpu_window_report(&w, &now, &r) && r.window_s == 900 && r.avg[0] == 20 &&
              r.peak[0] == 20 && r.avg[1] == 2, "second window %lu s %d/%d %d, want 900 s 20/20 2",
              (unsigned long)r.window_s, r.avg[0], r.peak[0], r.avg[1]);
    }

    printf("[4] a report that failed to publish keeps its window open\n");
    {
        CpuWindow_t w;
        CpuReport_t r;
        CpuSnap_t now = snap();
        cpu_window_begin(&w, &now);
        for (int m = 0; m < 15; m++) { run(60, 30, 10); now = snap(); cpu_window_sample(&w, &now); }
        CHECK(cpu_window_report(&w, &now, &r) && r.window_s == 900, "first report");
        /* publish failed: no cpu_window_begin() — the next report covers both */
        for (int m = 0; m < 15; m++) { run(60, 50, 10); now = snap(); cpu_window_sample(&w, &now); }
        CHECK(cpu_window_report(&w, &now, &r) && r.window_s == 1800 && r.avg[0] == 40 &&
              r.peak[0] == 50, "after a failed publish: %lu s avg %d peak %d, want 1800 s 40 50",
              (unsigned long)r.window_s, r.avg[0], r.peak[0]);
    }

    printf("[5] short windows and odd calls\n");
    {
        CpuWindow_t w;
        CpuReport_t r;
        memset(&w, 0, sizeof(w));
        run(5, 0, 0);
        CpuSnap_t now = snap();
        cpu_window_sample(&w, &now);                       /* first sample opens the window */
        CHECK(w.open, "a sample did not open the window");
        run(40, 60, 30);                                   /* less than one sample period */
        now = snap();
        CHECK(cpu_window_report(&w, &now, &r) && r.avg[0] == 60 && r.peak[0] == 60 &&
              r.avg[1] == 30 && r.peak[1] == 30,
              "a window shorter than a sample: avg %d peak %d, want the average as the peak",
              r.avg[0], r.peak[0]);
        cpu_window_sample(&w, &now);
        cpu_window_sample(&w, &now);                       /* same instant twice: no-op */
        CHECK(w.peak[0] == 60, "a repeated sample changed the peak to %d", w.peak[0]);
        /* A heavy stretch after the last sample still counts toward the
         * average, and the peak is never below the average. */
        run(30, 100, 0);
        now = snap();
        CHECK(cpu_window_report(&w, &now, &r) && r.peak[0] >= r.avg[0],
              "peak %d below average %d", r.peak[0], r.avg[0]);
        CpuReport_t keep = { 7, { 1, 2 }, { 3, 4 } };
        CpuSnap_t past = { 1, { 0, 0 } };
        CHECK(!cpu_window_report(&w, &past, &keep) && keep.window_s == 7 && keep.avg[0] == 1,
              "a refused report touched its output");
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
