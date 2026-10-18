#include "cpu_load.h"
#include <string.h>

void cpu_snap_add_task(CpuSnap_t *s, const char *name, uint64_t run_us) {
    if (!s || !name || strncmp(name, "IDLE", 4) != 0) return;
    /* Exactly "IDLE" + one core digit: not "IDLE", not "IDLE2", not "IDLE01". */
    if (name[4] < '0' || name[4] >= '0' + CPU_CORES || name[5] != '\0') return;
    s->idle_us[name[4] - '0'] += run_us;
}

int cpu_load_pct(const CpuSnap_t *a, const CpuSnap_t *b, int core) {
    if (!a || !b || core < 0 || core >= CPU_CORES) return -1;
    if (b->total_us <= a->total_us || b->idle_us[core] < a->idle_us[core]) return -1;
    uint64_t elapsed = b->total_us - a->total_us;
    uint64_t idle    = b->idle_us[core] - a->idle_us[core];
    /* The task table is not read in one instant, so a core idle the whole time
     * can show a few µs more idle than wall time: that is 0 % busy, never less. */
    if (idle > elapsed) idle = elapsed;
    uint64_t busy = elapsed - idle;
    return (int)((busy * 100u + elapsed / 2) / elapsed);
}

void cpu_window_begin(CpuWindow_t *w, const CpuSnap_t *now) {
    w->open  = true;
    w->start = *now;
    w->last  = *now;
    for (int c = 0; c < CPU_CORES; c++) w->peak[c] = -1;
}

void cpu_window_sample(CpuWindow_t *w, const CpuSnap_t *now) {
    if (!w->open) {
        cpu_window_begin(w, now);
        return;
    }
    for (int c = 0; c < CPU_CORES; c++) {
        int load = cpu_load_pct(&w->last, now, c);
        if (load > w->peak[c]) w->peak[c] = load;
    }
    w->last = *now;
}

bool cpu_window_report(const CpuWindow_t *w, const CpuSnap_t *now, CpuReport_t *out) {
    if (!w->open) return false;
    CpuReport_t r = {0};
    for (int c = 0; c < CPU_CORES; c++) {
        int avg = cpu_load_pct(&w->start, now, c);
        if (avg < 0) return false;
        r.avg[c]  = avg;
        r.peak[c] = (w->peak[c] > avg) ? w->peak[c] : avg;
    }
    r.window_s = (uint32_t)((now->total_us - w->start.total_us) / 1000000u);
    *out = r;
    return true;
}
