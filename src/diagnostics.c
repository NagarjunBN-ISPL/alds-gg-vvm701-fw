#include "diagnostics.h"
#include "cpu_load.h"
#include "di.h"
#include "du_task.h"
#include "store.h"
#include "modem.h"
#include "mqtt.h"
#include "forensics.h"
#include "rs485_bus.h"
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

/* ── CPU load per core (cpu_load.h) ────────────────────────────────────────
 * The window opens at boot and again at every PUBLISHED report; a one-minute
 * sample (diag_cpu_sample, every CPU_LOAD_SAMPLE_CYCLES main cycles) keeps the
 * busiest minute. Main task only — the task table below is static. */
static CpuWindow_t s_cpu;

/* The task table as a snapshot. False when run-time stats are off in sdkconfig
 * (the report then simply has no "cpu" key) or the table outgrew the buffer. */
static bool cpu_snapshot(CpuSnap_t *s) {
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    /* One entry per task: about a dozen exist (main, du_task, di_task, relay,
     * esp_timer, ipc0/1, IDLE0/1, ...), 24 leaves room for anything a future
     * IDF adds. Static: ~1.5 KB that must not live on the stack. */
    static TaskStatus_t tasks[24];
    configRUN_TIME_COUNTER_TYPE total = 0;
    UBaseType_t n = uxTaskGetSystemState(tasks, sizeof(tasks) / sizeof(tasks[0]), &total);
    if (n == 0) return false;
    memset(s, 0, sizeof(*s));
    s->total_us = (uint64_t)total;
    for (UBaseType_t i = 0; i < n; i++)
        cpu_snap_add_task(s, tasks[i].pcTaskName, (uint64_t)tasks[i].ulRunTimeCounter);
    return true;
#else
    (void)s;
    return false;
#endif
}

void diag_cpu_sample(void) {
    CpuSnap_t now = {0};
    if (cpu_snapshot(&now)) cpu_window_sample(&s_cpu, &now);
}

/* ── Init (diag_init + sysmon_init merged) ──────────────────────────────── */
void diag_init(void) {
    memset(&g_diag, 0, sizeof(g_diag));
    g_diag.grace_start_us = esp_timer_get_time();

    s_reset_reason = (int)esp_reset_reason();
    /* One bump per boot — diag_init() is called exactly once from app_main,
     * after nvs_flash_init(). Lets the fleet spot a reboot loop that telemetry
     * gaps alone wouldn't reveal. */
    s_boot_count = nvs_boot_count_bump();

    CpuSnap_t cpu_now = {0};
    if (cpu_snapshot(&cpu_now)) cpu_window_begin(&s_cpu, &cpu_now);
    else ESP_LOGW(TAG, "CPU load not measured — FreeRTOS run-time stats are off in sdkconfig");

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
    g_diag.loop_sum_ms += ms;
    g_diag.loop_count++;
}

/* ── Nozzle bitmask -> JSON array of labels ───────────────────────────────
 * Emits [] for none. Bounded by DU_LABEL_MAX (22), so at most 22 two-digit
 * entries — 88 bytes worst case, which the caller's budget allows for. */
static int absent_json(uint32_t mask, char *buf, size_t size) {
    /* MUST always leave a syntactically valid array in buf. It is interpolated
     * with %s into the report, so an empty string here would emit
     * "nozzles_absent":, — invalid JSON that the backend could not parse at
     * all. Every early exit therefore writes "[]" rather than "". */
    if (!buf || size < 3) { if (buf && size) buf[0] = '\0'; return 0; }

    int pos = 0;
    buf[pos++] = '[';
    bool first = true;
    for (int label = 1; label <= 32; label++) {
        if (!(mask & (1u << (label - 1)))) continue;
        /* +2 reserves room for the closing bracket and the NUL. */
        int n = snprintf(buf + pos, size - pos - 1, "%s%d", first ? "" : ",", label);
        if (n < 0 || n >= (int)(size - pos - 1)) break;  /* keep what fits, close */
        pos += n;
        first = false;
    }
    buf[pos++] = ']';
    buf[pos]   = '\0';
    return pos;
}

/* ── Single combined report ────────────────────────────────────────────────
 * One message under the "diagnostics" key — the cloud routes on it.
 *
 * Grouped rather than flat, because the flat list had grown to 27 keys with no
 * indication of which ones answer the same question. The groups map to the
 * questions a fuel-gateway operator actually asks:
 *
 *   records   am I losing sale data, and how close am I to losing more?
 *   link      is the gateway reachable, and has it been stable?
 *   bus       is the RS485 side healthy, and if not, WHICH unit?
 *   sys       is the hardware healthy?
 *   last_stop why did it stop last time?
 *
 * `health` + `why` are the device's own verdict over all of the above, so a
 * dashboard can show something correct without the backend re-implementing
 * every threshold. Every value below is read from live state — nothing here is
 * a constant dressed up as a measurement. */
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
    /* Acquisition-task health. Without these there is no way to tell from the
     * cloud whether an outage was actually survived: du_spill counts payloads
     * pushed to flash because the link was down (non-zero = buffering worked),
     * du_drop counts payloads lost because both RAM and flash refused (any
     * non-zero value is real data loss and should alert). du_q is the live RAM
     * backlog — persistently non-zero means publishing is not keeping up with
     * polling. Counters are cumulative since boot, NOT windowed like pub_ok. */
    TaskHandle_t du_h       = du_task_handle();
    uint32_t du_stack_hwm   = du_h ? (uint32_t)uxTaskGetStackHighWaterMark(du_h) : 0;
    uint32_t du_q           = (uint32_t)du_task_pending();
    uint32_t du_spill       = du_task_spilled();
    uint32_t du_drop        = du_task_dropped();
    uint32_t store_q        = (uint32_t)store_pending();   /* O(1) — never scan here */

    /* Two states that are otherwise INVISIBLE from the cloud, and both mean
     * the device is quietly not doing its job:
     *
     *   store_ok=false  littlefs never mounted, so there is no offline buffer
     *                   at all — an outage will lose every record instead of
     *                   replaying it, and nothing else in the telemetry would
     *                   ever say so (store_q reads 0 either way).
     *   rtc_synced=false  the clock has never been set from the network, so
     *                   every TS being published is 1970. Records are still
     *                   correctly ORDERED, but their absolute timestamps are
     *                   wrong — the backend needs to be able to see that
     *                   rather than infer it from the timestamps themselves.
     *
     * Both are booleans on the health report, not the telemetry contract, so
     * adding them changes nothing a payload parser depends on. */
    bool     store_ok       = store_ready();
    bool     time_synced    = rtc_is_synced();

    /* Permanent-loss counter for records that were buffered and then evicted
     * when the queue hit its ceiling — distinct from du_drop, which counts
     * records refused at enqueue. This is the loss a long outage actually
     * produces, and before this counter it appeared nowhere in telemetry. */
    uint32_t trimmed        = store_trimmed();
    /* Measured against STORE_MAX_BYTES, the ceiling the trim path itself uses,
     * so this is a true distance-to-data-loss rather than partition usage. */
    size_t   q_bytes        = store_queue_bytes();
    unsigned q_pct          = (unsigned)((uint64_t)q_bytes * 100ULL / (uint64_t)STORE_MAX_BYTES);
    if (q_pct > 100) q_pct = 100;

    uint32_t absent_mask    = du_task_absent_mask();
    uint32_t lock_timeouts  = rs485_bus_lock_timeouts();
    uint32_t connected_s    = mqtt_connected_s();

    /* Worst headroom across the three long-lived tasks. Only the minimum is
     * ever acted on, so three separate HWM fields spent payload on a decision
     * nobody makes. A zero from a task that never started is excluded rather
     * than reported as "no headroom left". */
    uint32_t stack_min = main_stack_hwm;
    if (di_stack_hwm && di_stack_hwm < stack_min) stack_min = di_stack_hwm;
    if (du_stack_hwm && du_stack_hwm < stack_min) stack_min = du_stack_hwm;

    uint32_t cycle_avg_ms = g_diag.loop_count
                          ? (uint32_t)(g_diag.loop_sum_ms / g_diag.loop_count)
                          : 0;

    /* CPU load per core over this report's window (cpu_load.h), in "sys". No
     * "cpu" key at all when it cannot be measured — never a made-up number. */
    CpuSnap_t   cpu_now = {0};
    CpuReport_t cpu     = {0};
    char        cpu_json[112] = "";
    bool        cpu_have = cpu_snapshot(&cpu_now) && cpu_window_report(&s_cpu, &cpu_now, &cpu);
    if (cpu_have) {
        snprintf(cpu_json, sizeof(cpu_json),
                 ",\"cpu\":{\"window_s\":%lu,\"core0_avg\":%d,\"core1_avg\":%d,"
                 "\"core0_peak\":%d,\"core1_peak\":%d}",
                 (unsigned long)cpu.window_s, cpu.avg[0], cpu.avg[1], cpu.peak[0], cpu.peak[1]);
        ESP_LOGI(TAG, "CPU load over the last %lu s: core0 %d%% (busiest minute %d%%), "
                      "core1 %d%% (busiest minute %d%%)",
                 (unsigned long)cpu.window_s, cpu.avg[0], cpu.peak[0], cpu.avg[1], cpu.peak[1]);
    }

    /* ── Health verdict ───────────────────────────────────────────────────
     *   fault    — data is being lost now, or the last stop was a real hang.
     *   degraded — something is wrong that has not cost data yet.
     * `why` carries the specific reasons as stable tokens so the backend
     * alerts on those rather than re-deriving every threshold, and so a new
     * reason can be added here without the dashboard needing a release. */
    char why[192];
    int  why_pos = 0;
    bool fault = false, degraded = false;
    why[0] = '\0';

    #define WHY(tok) do {                                                      \
        int _n = snprintf(why + why_pos, sizeof(why) - why_pos,                \
                          "%s\"" tok "\"", why_pos ? "," : "");                \
        if (_n > 0 && _n < (int)(sizeof(why) - why_pos)) why_pos += _n;        \
    } while (0)

    if (!store_ok)                  { fault = true; WHY("store_unmounted");  }
    if (du_drop)                    { fault = true; WHY("records_dropped");  }
    if (trimmed)                    { fault = true; WHY("records_trimmed");  }
    if (forensics_last_was_fault()) { fault = true; WHY("last_stop_fault");  }

    if (!time_synced)               { degraded = true; WHY("clock_unsynced"); }
    if (absent_mask)                { degraded = true; WHY("nozzle_absent");  }
    if (q_pct > 25)                 { degraded = true; WHY("queue_filling");  }
    if (g_diag.recov_l2 || g_diag.recov_l3)
                                    { degraded = true; WHY("link_rebuilt");   }
    if (g_diag.pub_fail)            { degraded = true; WHY("publish_failures"); }
    if (temp_c > THERMAL_WARM_MAX_C){ degraded = true; WHY("temp_high");      }
    /* Reconnected recently on a device that has been up a while — the
     * signature of a flapping link, which a single report otherwise hides. */
    if (uptime_s > 600 && connected_s < 300)
                                    { degraded = true; WHY("link_unstable");  }
    #undef WHY

    const char *health = fault ? "fault" : (degraded ? "degraded" : "ok");

    char absent[96];
    absent_json(absent_mask, absent, sizeof(absent));

    /* Empty (len 0) on a cold boot, or if it would not fit — in which case the
     * key is omitted entirely rather than emitted broken. */
    char stop[224];
    int  stop_len = forensics_stop_json(stop, sizeof(stop));

    /* Under the "diagnostics" key, on <base>/logs — the cloud identifies a
     * message kind by which top-level key is present. */
    /* Measured, not guessed: 774 bytes for a healthy report, 790 with one
     * issue flagged, and 1180 for an absolute worst case (every `why` token
     * set, a last_stop object present, all 22 nozzles listed absent, and a
     * 23-character operator name). The "cpu" object (2026-10-16) adds at most
     * 96, so ~1276. Sized to the modem's own ceiling, MQTT_MODEM_PAYLOAD_SAFE_MAX
     * (1400), past which the modem itself rejects the publish; the guard below
     * still catches any overrun and degrades to a valid minimal report rather
     * than emitting broken JSON. */
    char buf[MQTT_MODEM_PAYLOAD_SAFE_MAX];
    int written = snprintf(buf, sizeof(buf),
        "{\"HW_ID\":\"%s\",\"diagnostics\":{"
        "\"health\":\"%s\",\"why\":[%s],"
        "\"fw\":\"%s\",\"product\":\"%s\",\"cred_fp\":\"%08lx\","
        "\"uptime_s\":%lu,\"boot_count\":%lu,\"reset_reason\":%d,"
        "%s%s%s"
        "\"records\":{\"q_ram\":%lu,\"q_flash\":%lu,\"q_bytes\":%lu,\"q_pct\":%u,"
                     "\"spilled\":%lu,\"dropped\":%lu,\"trimmed\":%lu,"
                     "\"store_ok\":%s,\"rtc_synced\":%s},"
        "\"link\":{\"rssi\":%d,\"operator\":\"%s\",\"connected_s\":%lu,"
                  "\"recov_l1\":%lu,\"recov_l2\":%lu,\"recov_l3\":%lu,"
                  "\"bearer_distrust\":%lu,\"pub_ok\":%lu,\"pub_fail\":%lu},"
        "\"bus\":{\"nozzles_absent\":%s,\"lock_timeouts\":%lu,"
                 "\"cycle_max_ms\":%lu,\"cycle_avg_ms\":%lu,\"cycles\":%lu},"
        "\"sys\":{\"heap_free\":%lu,\"heap_min_ever\":%lu,\"stack_min\":%lu,"
                 "\"temp_c\":%.1f,\"vbat_mv\":%d%s},"
        "\"subsystems\":{\"du\":%s,\"radar\":%s,\"genset\":%s,\"di\":%s,\"gps\":%s}"
        "},\"TS\":\"%s\"}",
        hw_id,
        health, why,
        FW_VERSION, PRODUCT_ID, (unsigned long)nvs_config_compiled_cred_fp(),
        (unsigned long)uptime_s, (unsigned long)s_boot_count, s_reset_reason,
        stop_len ? "\"last_stop\":" : "", stop_len ? stop : "", stop_len ? "," : "",
        (unsigned long)du_q, (unsigned long)store_q,
        (unsigned long)q_bytes, q_pct,
        (unsigned long)du_spill, (unsigned long)du_drop, (unsigned long)trimmed,
        store_ok    ? "true" : "false",
        time_synced ? "true" : "false",
        csq, operator_name, (unsigned long)connected_s,
        (unsigned long)g_diag.recov_l1, (unsigned long)g_diag.recov_l2,
        (unsigned long)g_diag.recov_l3, (unsigned long)g_diag.bearer_distrust,
        (unsigned long)g_diag.pub_ok, (unsigned long)g_diag.pub_fail,
        absent, (unsigned long)lock_timeouts,
        (unsigned long)g_diag.loop_max_ms, (unsigned long)cycle_avg_ms,
        (unsigned long)g_diag.loop_count,
        (unsigned long)heap_free, (unsigned long)heap_min_ever,
        (unsigned long)stack_min,
        temp_c, vbat_mv, cpu_json,
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
        ESP_LOGI(TAG, "Diagnostics published (health=%s)", health);
        g_diag.pub_ok      = 0;
        g_diag.pub_fail    = 0;
        g_diag.loop_max_ms = 0;
        g_diag.loop_sum_ms = 0;
        g_diag.loop_count  = 0;
        /* Cleared only now, for the same reason the window counters are: a
         * device that dies and comes back into an outage must still report why
         * once the link returns. The ladder counters (recov_*) are deliberately
         * NOT cleared — they are cumulative since boot so the backend can diff
         * consecutive reports, which windowed values would make impossible. */
        forensics_stop_clear();
        /* The next report's CPU window starts here, like the counters above. */
        if (cpu_have) cpu_window_begin(&s_cpu, &cpu_now);
    } else {
        ESP_LOGW(TAG, "Diagnostics publish failed — counters retained for next attempt");
    }
}
