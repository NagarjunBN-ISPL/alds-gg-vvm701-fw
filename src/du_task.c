#include "du_task.h"
#include "forensics.h"
#include "config.h"
#include "nvs_config.h"
#include "dispenser.h"
#include "yenen.h"
#include "rs485_bus.h"
#include "store.h"
#include "diagnostics.h"
#include "modem.h"
#include "mqtt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "du_task";

/* Live config and HW_IDs are owned by main.c and are fixed after boot — a
 * setconfig always reboots, so there is no concurrent writer to guard here. */
extern DeviceConfig g_cfg;
extern char g_hw_du[DU_MAX_DISPENSERS][24];

/* One slot per queued payload. Sized to the dispenser payload buffer so a
 * queue entry can hold any message either driver builds.
 *
 * `seq` identifies the record for the peek/release token in du_task.h. It is
 * assigned once at emit and never reused, so a token can only match the exact
 * payload it was issued for. */
typedef struct {
    uint32_t seq;
    char     json[DU_PAYLOAD_MAX];
} DuMsg_t;

static QueueHandle_t s_q          = NULL;
static TaskHandle_t  s_task       = NULL;
static unsigned      s_spilled    = 0;
static unsigned      s_dropped    = 0;
static uint32_t      s_seq        = 0;   /* next payload's seq; only du_task writes it */

/* Serialises du_task_flush_to_store() against itself. That function drains the
 * queue into a single shared static buffer, and it is called from BOTH cores:
 * du_task on core 1 (when MQTT is down) and the main/FOTA path on core 0. With
 * no guard, two concurrent drains interleave xQueueReceive into the same buffer
 * and one record is torn / duplicated / lost. A bounded take is enough:
 * whoever loses the race simply returns, because the winner is draining the
 * same shared queue and will take those records too — nothing is left behind. */
static SemaphoreHandle_t s_flush_lock = NULL;

/* ── Enqueue path ─────────────────────────────────────────────────────────
 * RAM first, flash only on overflow. In normal running the main task drains
 * every cycle so the queue never fills and flash is never touched — roughly
 * 8,600 writes a day avoided at two nozzles. The queue only backs up when the
 * main task is stuck in the recovery ladder, which is exactly when persistence
 * starts to matter: RAM would not survive the restart that ladder ends in.
 *
 * Note the send is non-blocking (timeout 0). Blocking here would stall
 * acquisition waiting on a consumer that may be minutes away from running
 * again — the precise failure this task exists to avoid. */
static void du_emit(const char *payload)
{
    /* Never queue an empty record: store_enqueue() rejects it, and an empty
     * MQTT publish is worse than useless. Reaching here with one would mean a
     * caller emitted without building a payload — see the poll functions. */
    if (!payload || !payload[0]) {
        ESP_LOGE(TAG, "du_emit called with an empty payload — dropped (bug)");
        return;
    }

    /* static, not stack: DuMsg_t is a full DU_PAYLOAD_MAX buffer, and the two
     * of them here were 2 KB of du_task's 6 KB stack — on the same path that
     * descends into littlefs/fprintf. du_emit() is called only from du_task,
     * sequentially, so static is safe and buys back the headroom. */
    static DuMsg_t msg;
    msg.seq = ++s_seq;
    strncpy(msg.json, payload, sizeof(msg.json) - 1);
    msg.json[sizeof(msg.json) - 1] = '\0';

    /* RULE: flash must always hold records STRICTLY OLDER than RAM, because
     * publish_dispenser_queue() drains flash first. So the moment anything is
     * on flash, everything goes to flash until it is fully drained.
     *
     * Without this the order inverts and the cloud gets records out of
     * sequence: an outage fills RAM first, so RAM would hold the OLDER records
     * while the newer ones spilled to flash — and flash-first draining would
     * then publish newer before older. Since the backend reconstructs each sale
     * by differencing consecutive totalizer readings ordered by TS, an
     * out-of-order replay produces negative deltas and phantom transactions.
     * This ordering is the whole contract, not an optimisation.
     *
     * `du_task_pending() == 0` is the other half of that contract and is NOT
     * redundant. During an outage the RAM queue is normally empty (everything
     * is going straight to flash), but there are two paths that legitimately
     * leave older records sitting in RAM while flash is non-empty:
     *
     *   - the main task requeues a failed publish to the front of the RAM
     *     queue (du_task_requeue_front) while a flash backlog already exists;
     *   - a store_enqueue() failure below falls back to RAM.
     *
     * Appending to flash in either state would file a NEW record behind OLDER
     * ones still in RAM — the exact inversion this rule exists to prevent. So
     * whenever RAM holds anything, the new record joins it there, and the
     * overflow path below is what moves the whole run to flash in order. */
    if (store_ready() && du_task_pending() == 0 && store_has_records()) {
        if (store_enqueue(payload)) { s_spilled++; return; }
        /* Flash refused (full/faulted) — fall through and try RAM rather than
         * drop. Order is already compromised at that point, but a record the
         * cloud can dedupe beats a record that never arrives. */
    }

    if (s_q && xQueueSend(s_q, &msg, 0) == pdTRUE) return;

    /* RAM is full — normally the first spill of an outage, and also the way a
     * short run of records left in RAM by the paths above eventually reaches
     * flash. Move the whole RAM queue to flash IN ORDER before appending this
     * one, so flash ends up holding the complete oldest-first sequence and RAM
     * is left empty. Anything already on flash is older than anything in RAM
     * (that is the invariant), and this payload is newer than all of it, so
     * appending in queue order keeps the whole log oldest-first. It also has a
     * second benefit: the recovery ladder
     * ends in a restart, and anything still in RAM at that moment is lost.
     * Draining RAM to flash here means a restart mid-outage loses nothing.
     *
     * store_enqueue() takes its own lock and must NOT run with the RS485 bus
     * held — a littlefs erase can block >100 ms. Both callers release the bus
     * before reaching here. */
    if (store_ready()) {
        if (s_spilled == 0)
            ESP_LOGW(TAG, "RAM queue full — moving %d record(s) to flash (link down?)",
                     du_task_pending());
        static DuMsg_t old;                 /* static for the same stack reason */
        while (s_q && xQueueReceive(s_q, &old, 0) == pdTRUE) {
            if (store_enqueue(old.json)) s_spilled++;
            else                         s_dropped++;
        }
        if (store_enqueue(payload)) { s_spilled++; return; }
    }

    s_dropped++;
    ESP_LOGE(TAG, "payload dropped — RAM queue full and flash unavailable (total %u)",
             s_dropped);
}

/* ── Poll result ──────────────────────────────────────────────────────────
 * "Was a payload built?" and "is the nozzle healthy?" are DIFFERENT questions
 * and must not share a return value.
 *
 * Both poll functions used to return one bool meaning "healthy", and the caller
 * emitted the payload unconditionally afterwards. On the bus-lock timeout path
 * they return before building anything — so the caller emitted whatever the
 * SHARED payload buffer still held: the previous nozzle's payload, re-queued
 * under a second identity, or on the very first round an all-zero buffer
 * publishing as an empty string.
 *
 * A duplicated payload is not a cosmetic bug here. It carries a stale totalizer
 * with a fresh position in the stream, and the backend derives every sale from
 * the difference between consecutive readings — so a repeat reading of an
 * earlier value shows up as a negative delta, i.e. a phantom transaction.
 *
 * `built` gates the emit; `healthy` only feeds du_ok. `polled`/`responded` were
 * added for the absent-nozzle backoff below and answer a third, separate
 * question — "did we learn anything about this nozzle's existence this round?"
 * — which neither of the others can express: a bus-lock timeout is not evidence
 * of absence, and a busy nozzle is healthy and very much present. */
typedef struct {
    bool built;      /* payload buffer now holds a fresh payload for this nozzle */
    bool healthy;    /* nozzle answered — busy counts (only valid if built)      */
    bool polled;     /* nozzle was actually interrogated (false = bus never won) */
    bool responded;  /* nozzle answered AT ALL — busy counts (only if polled)    */
} DuPollResult_t;

/* ── Absent-nozzle backoff ────────────────────────────────────────────────
 * Reclaims the dead time a silent nozzle burns every round so it stops
 * stretching the cadence of the nozzles that work. See the DU_ABSENT_*
 * rationale in config.h — in particular why the counter must advance on
 * "did not respond" rather than on "unhealthy" (a nozzle mid-sale reports busy,
 * and demoting it then would be the worst possible time to stop watching).
 *
 * Indexed by nozzle label (1-based), sized to the largest label either driver
 * can produce so a config change can never index past the end. */
#define DU_LABEL_MAX (DU_MAX_DISPENSERS * 2)

static uint16_t s_miss[DU_LABEL_MAX + 1];   /* consecutive silent rounds, per label */
static uint32_t s_round;                    /* round counter, drives the Yenen probe phase   */
static uint32_t s_bs_round;                 /* rounds in which BlueSky was polled (its phase) */

/* Should this round skip the nozzle entirely? Only ever true for one that has
 * been silent for DU_ABSENT_AFTER_ROUNDS consecutive rounds, and even then it
 * is still probed every DU_ABSENT_RETRY_EVERY rounds.
 *
 * `round` is the caller's OWN round counter — the count of rounds in which
 * that protocol was actually polled (s_bs_round for BlueSky, s_round for
 * Yenen) — so "every 6 rounds" means every 6 of ITS rounds. Using the shared
 * counter for BlueSky once made the probe phase and the BlueSky phase never
 * coincide, and a demoted BlueSky nozzle was never probed again until reboot. */
static bool du_absent_skip(int label, uint32_t round)
{
    if (label < 1 || label > DU_LABEL_MAX) return false;
    if (s_miss[label] < DU_ABSENT_AFTER_ROUNDS) return false;
    return (round % DU_ABSENT_RETRY_EVERY) != 0;
}

/* Only called for a nozzle that was actually interrogated. Any response resets
 * the counter, restoring full cadence on the very next round. */
static void du_absent_update(int label, bool responded)
{
    if (label < 1 || label > DU_LABEL_MAX) return;
    if (responded) {
        if (s_miss[label] >= DU_ABSENT_AFTER_ROUNDS)
            ESP_LOGI(TAG, "nz%d answered again — restoring full poll cadence", label);
        s_miss[label] = 0;
        return;
    }
    if (s_miss[label] < 0xFFFF) s_miss[label]++;
    if (s_miss[label] == DU_ABSENT_AFTER_ROUNDS)
        ESP_LOGW(TAG, "nz%d silent for %d consecutive rounds — probing every %d rounds "
                      "so its retry budget stops delaying the live nozzles",
                 label, DU_ABSENT_AFTER_ROUNDS, DU_ABSENT_RETRY_EVERY);
}

/* Which nozzles are currently demoted to probe-only, as a bitmask over global
 * labels (bit 0 = label 1). DU_LABEL_MAX is 22, so a uint32_t covers every
 * nozzle a fully-loaded gateway can have.
 *
 * This is the difference between "du_ok:false, drive out and check all ten
 * nozzles" and "nozzle 7 has been silent". The backoff logic already knows
 * exactly which nozzles stopped answering; it simply had no way to say so. */
uint32_t du_task_absent_mask(void)
{
    uint32_t mask = 0;
    for (int label = 1; label <= DU_LABEL_MAX; label++)
        if (s_miss[label] >= DU_ABSENT_AFTER_ROUNDS)
            mask |= (1u << (label - 1));
    return mask;
}

/* ── Timestamp gate ───────────────────────────────────────────────────────
 * A payload is only safe to emit if it carries a REAL wall-clock time.
 *
 * du_task_start() runs before network_init() and before the first
 * modem_time_sync() (main.c boot order), so on every cold boot this task polls
 * for the whole bring-up window — typically 30 s to several minutes, since
 * registration alone can wait 120 s per attempt and the bring-up retries with
 * modem hard resets — with the RTC still unset. rtc_timestamp() reports that by
 * returning false and writing the epoch string, but both call sites used to
 * ignore the return value, so at a 12 s cadence every boot published 2-20+
 * readings PER NOZZLE all stamped "1970-01-01 00:00:00".
 *
 * That is corrupting, not merely useless, because the backend reconstructs each
 * sale by differencing consecutive totalizer readings ORDERED BY TS:
 *   - many records sharing one identical TS have no defined order, so the
 *     differencing runs in arbitrary order and yields negative deltas —
 *     phantom and merged transactions;
 *   - epoch stamps sort before all real history, so the first correctly
 *     stamped reading differences against a 1970 record;
 *   - dedupe on HW_ID + TS + totalizer collapses the identical-TS idle
 *     readings into one, silently discarding readings.
 * Those are exactly the "transaction merge" and "transaction missing"
 * symptoms seen in the field.
 *
 * Skipping the emit costs NOTHING: totalizers are cumulative, so the first
 * correctly stamped reading after sync carries the entire delta accrued during
 * the blind window. A skipped reading loses no volume; an epoch-stamped reading
 * corrupts the stream it lands in.
 *
 * The gate is rtc_timestamp()'s own return value (year >= 2020), NOT
 * rtc_is_synced(). s_time_synced is a static bool that resets to false on every
 * reboot, but the RTC retains time across esp_restart() — so rtc_is_synced()
 * would needlessly suppress a device whose clock is genuinely good after a
 * software restart. The year check is the true test of "is this timestamp
 * usable". Note also that once time is known the gate stays open through a
 * network outage (the RTC free-runs), so outage buffering is unaffected — only
 * a device that has never had time since cold boot is held back. */
static bool du_ts_now(char *ts, size_t ts_size, int label)
{
    if (rtc_timestamp(ts, ts_size)) return true;
    ESP_LOGW(TAG, "nz%d: RTC not set yet — reading NOT published (would carry the "
                  "epoch TS and corrupt cloud-side sale differencing); the next "
                  "reading after time sync carries the full delta", label);
    return false;
}

/* ── One BlueSky nozzle ───────────────────────────────────────────────── */
/* `label` is the global nozzle number, 2N-1 or 2N for dispenser N — and on a
 * BlueSky bus that IS the nozzle's RS485 address, so the label the cloud sees
 * and the address polled are one number by construction. */
static DuPollResult_t poll_bluesky_nozzle(const char *hw_id, int label, bool read_kf, char *payload)
{
    static DuNozzle_t nz;                       /* static: keeps it off the task stack */
    uint8_t addr  = (uint8_t)label;
    char    ts[32];

    /* polled = false: the nozzle was never interrogated, so this round carries NO
     * evidence about whether it is present. Counting it as a miss would let a
     * run of bus contention demote a perfectly good nozzle. */
    if (!rs485_bus_lock(RS485_BUS_LOCK_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "bus busy — skipping BlueSky nz%d this round (NOT published)", label);
        return (DuPollResult_t){ .built = false, .healthy = false, .polled = false };
    }
    rs485_bus_set_bluesky();
    du_poll_bluesky_nozzle(addr, &nz, read_kf);
    rs485_bus_set_8n1(9600);                    /* leave the bus as radar/genset expect */
    rs485_bus_unlock();

    /* stamped at the read, not at publish. `healthy` stays truthful even when
     * the reading is withheld — the pump answered; only the clock is missing.
     * A nozzle mid-sale (busy) is healthy: it answered, it is just not read
     * until the sale ends. Only a nozzle that does not answer clears du_ok —
     * otherwise diagnostics reported a dispenser fault during every sale. */
    if (!du_ts_now(ts, sizeof(ts), label))
        return (DuPollResult_t){ .built = false, .healthy = nz.present,
                                 .polled = true, .responded = nz.present };

    du_build_nozzle_payload(&nz, label, hw_id, ts, payload, DU_PAYLOAD_MAX);
    return (DuPollResult_t){ .built = true, .healthy = nz.present,
                             .polled = true, .responded = nz.present };
}

/* ── One Yenen nozzle ─────────────────────────────────────────────────── */
/* `head` is the dispenser NUMBER (1..DU_YENEN_MAX) from du_number_of(), never
 * the loop index. */
static DuPollResult_t poll_yenen_nozzle(const char *hw_id, int head, int side, int label, char *payload)
{
    static YenenNozzle_t ynz;
    char ts[32];

    if (!rs485_bus_lock(RS485_BUS_LOCK_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "bus busy — skipping Yenen nz%d this round (NOT published)", label);
        return (DuPollResult_t){ .built = false, .healthy = false, .polled = false };
    }
    rs485_bus_set_yenen();
    yn_poll_nozzle((uint8_t)head, (uint8_t)side, &ynz);
    rs485_bus_set_8n1(9600);
    rs485_bus_unlock();

    if (!du_ts_now(ts, sizeof(ts), label))
        return (DuPollResult_t){ .built = false, .healthy = ynz.present,
                                 .polled = true, .responded = ynz.present };

    yn_build_nozzle_payload(&ynz, label, hw_id, ts, payload, DU_PAYLOAD_MAX);
    return (DuPollResult_t){ .built = true, .healthy = ynz.present,
                             .polled = true, .responded = ynz.present };
}

/* ── Task ─────────────────────────────────────────────────────────────── */
static void du_task_run(void *arg)
{
    (void)arg;
    static char payload[DU_PAYLOAD_MAX];

    /* Registered with the task watchdog for the same reason every other loop
     * here is: a task that wedges silently is worse than one that reboots. */
    esp_task_wdt_add(NULL);

    /* K-factor is near-constant, so it is read on its own slow cadence and the
     * last good value is carried forward in between (du_poll_bluesky_nozzle
     * handles the carry). Gated once per round so the tight totalizer cadence
     * is not spent re-reading it. */
    int64_t last_kfactor_us = 0;

    ESP_LOGI(TAG, "started on core %d — dispensers=%u",
             xPortGetCoreID(), g_cfg.du_count);

    /* Start of the last round in which BlueSky nozzles were read — the anchor
     * for BLUESKY_POLL_INTERVAL_MS. 0 = never, so the first round reads them. */
    int64_t last_bs_us = 0;

    for (;;) {
        int64_t round_start = esp_timer_get_time();
        s_round++;                              /* drives the Yenen absent-nozzle probe phase */
        bool read_kf = (last_kfactor_us == 0 ||
                        round_start - last_kfactor_us >= (int64_t)KFACTOR_POLL_INTERVAL_S * 1000000LL);
        if (read_kf) last_kfactor_us = round_start;

        /* BlueSky is due when BLUESKY_POLL_INTERVAL_MS has elapsed since the
         * last BlueSky round STARTED — wall-clock, so the cadence is 20 s
         * whatever the rounds in between did. The 500 ms slack absorbs the
         * tick granularity of the pacing sleep: a round that starts a few ms
         * before the nominal boundary is still "this" boundary, not the next. */
        bool bs_due = (last_bs_us == 0 ||
                       round_start - last_bs_us >= (int64_t)BLUESKY_POLL_INTERVAL_MS * 1000LL - 500000LL);
        if (bs_due) { last_bs_us = round_start; s_bs_round++; }

        /* Snapshot the counts once per round rather than re-reading g_cfg in
         * each loop condition. parse_setconfig() assigns the whole DeviceConfig
         * struct, which is not atomic, and on its nvs_save_failed path it does
         * so WITHOUT the usual reboot — leaving a live window where this task
         * could see a half-updated struct and poll a nozzle that was just
         * deconfigured. Both counts are clamped to their MAX and the g_hw_*
         * arrays are sized to MAX, so a torn read was never memory-unsafe; the
         * snapshot just keeps each round internally consistent. */
        uint8_t n_du = g_cfg.du_count;
        if (n_du > DU_MAX_DISPENSERS) n_du = DU_MAX_DISPENSERS;

        bool all_ok = true;
        bool any_yenen = false;                 /* decides the round pacing below */

        for (int i = 0; i < n_du; i++) {
            /* THE dispenser number, from du_number_of() (nvs_config.c) — the
             * product's one rule for which pump a list entry is. It is the
             * address on the bus and the base of the nozzle labels — never the
             * loop index. nvs_config_load() guarantees it is valid and unique
             * for every entry inside du_count, so the check is a belt. */
            int n = du_number_of(&g_cfg, i);
            if (n < 1 || n > DU_MAX_DISPENSERS) {
                ESP_LOGE(TAG, "DU entry %d resolves to no dispenser number — skipped", i + 1);
                all_ok = false;
                continue;
            }
            if (g_cfg.du_make[i] == DU_MAKE_BLUESKY) {
                /* BlueSky on its own 20 s clock (BLUESKY_POLL_INTERVAL_MS): up to
                 * 7 RS485 reads per nozzle, so it is de-rated rather than the
                 * whole cadence. See config.h. */
                if (!bs_due) continue;
                for (int half = 0; half < 2; half++) {
                    int label = (n - 1) * 2 + half + 1;      /* = the nozzle's RS485 address */
                    /* A nozzle confirmed absent keeps its last known state in du_ok
                     * (all_ok is ANDed with false) — it is still a fault, it just
                     * stops costing a full retry budget every round. */
                    if (du_absent_skip(label, s_bs_round)) { all_ok = false; continue; }

                    forensics_mark(FSLOT_DU, F_POLL_BLUESKY, (uint8_t)label);
                    DuPollResult_t r = poll_bluesky_nozzle(g_hw_du[i], label, read_kf, payload);
                    if (r.built) du_emit(payload);   /* ONLY if this round built one */
                    if (r.polled) du_absent_update(label, r.responded);
                    all_ok &= r.healthy;
                    esp_task_wdt_reset();
                }
            } else if (g_cfg.du_make[i] == DU_MAKE_YENEN) {
                any_yenen = true;
                if (n > DU_YENEN_MAX) { all_ok = false; continue; }   /* head 6 unaddressable; belt */
                for (int side = 0; side < 2; side++) {
                    int label = (n - 1) * 2 + side + 1;
                    if (du_absent_skip(label, s_round)) { all_ok = false; continue; }

                    forensics_mark(FSLOT_DU, F_POLL_YENEN, (uint8_t)label);
                    DuPollResult_t r = poll_yenen_nozzle(g_hw_du[i], n, side, label, payload);
                    if (r.built) du_emit(payload);
                    if (r.polled) du_absent_update(label, r.responded);
                    all_ok &= r.healthy;
                    esp_task_wdt_reset();
                }
            }
            /* Any other make value is clamped away by nvs_config_load(); nothing
             * to do here. */
        }

        /* du_ok now means "every nozzle answered", not "answered and
         * published" — publishing moved to the main task, and its failures are
         * already counted in pub_fail. Splitting them is more useful anyway: a
         * dead pump and a dead link are different faults. */
        if (n_du > 0) g_diag.du_ok = all_ok;

        /* Cap RAM-loss exposure on an UNCLEAN reset (brownout / WDT panic / hard
         * power cut) during a network outage. While the link is down the main
         * task is in the recovery ladder and is NOT draining the RAM queue, so
         * readings accumulate in s_q until DU_QUEUE_SLOTS forces a spill — a span
         * (up to ~2.5 min at a light site) that an unclean reset would lose,
         * because du_task_flush_to_store() otherwise runs only on the CLEAN
         * restart the ladder ends in. Flushing RAM->flash each round while
         * offline bounds that loss to at most one round at any site size.
         *
         * Safe and cheap: it is a no-op with ZERO flash wear while online (link
         * up leaves the RAM-first fast path untouched); it preserves ordering
         * (the flush drains oldest-first, exactly like the reboot path and the
         * du_emit overflow path); and the RS485 bus is released here (every
         * nozzle poll above unlocks it), so the littlefs write cannot stall a
         * bus hold. mqtt_is_connected() is a plain bool read — safe from core 1. */
        if (!mqtt_is_connected() && du_task_pending() > 0)
            du_task_flush_to_store();

        /* Start-to-start pacing: subtract the time the round actually took so
         * the cadence holds regardless of how long the reads ran. A round that
         * overruns simply starts the next immediately rather than compounding
         * the delay. The round interval is PUBLISH_INTERVAL_MS (10 s) when a
         * Yenen dispenser needs that cadence, else BLUESKY_POLL_INTERVAL_MS
         * (20 s) — a BlueSky-only site has nothing to do in the odd rounds, so
         * it does not run them; either way BlueSky is read every 20 s. */
        int64_t interval_ms = any_yenen ? (int64_t)PUBLISH_INTERVAL_MS
                                        : (int64_t)BLUESKY_POLL_INTERVAL_MS;
        int64_t elapsed_ms  = (esp_timer_get_time() - round_start) / 1000;
        int64_t wait_ms     = interval_ms - elapsed_ms;
        if (wait_ms < DU_TASK_MIN_GAP_MS) wait_ms = DU_TASK_MIN_GAP_MS;

        /* Slice the wait so the watchdog is fed while idle. */
        int64_t slept = 0;
        while (slept < wait_ms) {
            int64_t chunk = (wait_ms - slept > 1000) ? 1000 : (wait_ms - slept);
            vTaskDelay(pdMS_TO_TICKS(chunk));
            esp_task_wdt_reset();
            slept += chunk;
        }
    }
}

/* ── Public ───────────────────────────────────────────────────────────── */
bool du_task_start(void)
{
    if (s_task) return true;

    if (g_cfg.du_count == 0) {
        ESP_LOGI(TAG, "no dispensers configured — task not started");
        return true;                       /* not a failure */
    }

    s_q = xQueueCreate(DU_QUEUE_SLOTS, sizeof(DuMsg_t));
    if (!s_q) {
        ESP_LOGE(TAG, "queue alloc failed (%d x %u bytes)",
                 DU_QUEUE_SLOTS, (unsigned)sizeof(DuMsg_t));
        return false;
    }

    /* Guards the cross-core flush drain — see s_flush_lock. Created before the
     * task so it is live the instant du_task_run() can call the flush. If it
     * cannot be created the flush degrades to unguarded (its prior behaviour),
     * which is no worse than before this guard existed. */
    if (!s_flush_lock) s_flush_lock = xSemaphoreCreateMutex();

    /* Core 1, away from the main task: the recovery ladder blocks core 0 for
     * minutes and this task must keep running through it. Priority above the
     * main loop so a poll is not starved by publish work, and matched to the
     * di_task stack (6144) which has the same shape of UART + snprintf frames. */
    if (xTaskCreatePinnedToCore(du_task_run, "du_task", 6144, NULL, 6, &s_task, 1) != pdPASS) {
        ESP_LOGE(TAG, "task create failed (heap OOM?)");
        vQueueDelete(s_q);
        s_q = NULL;
        return false;
    }
    return true;
}

bool du_task_peek(char *buf, size_t buf_size, uint32_t *token)
{
    if (!s_q || !buf || buf_size == 0) return false;
    static DuMsg_t msg;                        /* static: 1 KB, main task only */
    if (xQueuePeek(s_q, &msg, 0) != pdTRUE) return false;
    strncpy(buf, msg.json, buf_size - 1);
    buf[buf_size - 1] = '\0';
    if (token) *token = msg.seq;
    return true;
}

bool du_task_release(uint32_t token)
{
    if (!s_q) return false;

    static DuMsg_t m;                          /* static: 1 KB, main task only */
    if (xQueueReceive(s_q, &m, 0) != pdTRUE) return false;   /* spilled meanwhile */

    if (m.seq == token) return true;           /* discarded the record we published */

    /* Not the record we published: du_emit() drained the queue to flash while
     * the publish was in flight, and this is a newer payload that arrived
     * after. Put it back at the FRONT — the slot we just freed guarantees room
     * — so nothing is lost and nothing is reordered. The record we published is
     * on flash now and will be republished; a duplicate is the safe direction,
     * and the cloud dedupes on HW_ID + TS + totalizer. */
    if (xQueueSendToFront(s_q, &m, 0) != pdTRUE) {
        s_dropped++;
        ESP_LOGE(TAG, "release: could not restore a record after a queue rotation "
                      "(total dropped %u)", s_dropped);
    }
    return false;
}

int du_task_pending(void)
{
    return s_q ? (int)uxQueueMessagesWaiting(s_q) : 0;
}


TaskHandle_t du_task_handle(void) { return s_task; }
unsigned du_task_spilled(void)    { return s_spilled; }
unsigned du_task_dropped(void)    { return s_dropped; }

void du_task_flush_to_store(void) {
    if (!store_ready() || !s_q) return;
    /* Serialise the drain. A bounded wait (not portMAX) so the pre-restart FOTA
     * flush on core 0 actually completes when du_task is mid-drain on core 1,
     * yet no caller can ever block for long: the RAM queue is only DU_QUEUE_SLOTS
     * deep and drains in well under this window. If the wait times out we return
     * rather than drain concurrently — the holder owns the same queue and takes
     * these records too. Lock order is always s_flush_lock -> store's own lock
     * (store_enqueue), never the reverse, so this cannot deadlock. WDT-safe:
     * the timeout is far under WDT_TIMEOUT_MS. */
    if (s_flush_lock && xSemaphoreTake(s_flush_lock, pdMS_TO_TICKS(2000)) != pdTRUE) return;
    static DuMsg_t old;
    while (xQueueReceive(s_q, &old, 0) == pdTRUE) {
        if (store_enqueue(old.json)) s_spilled++;
        else                         s_dropped++;
    }
    if (s_flush_lock) xSemaphoreGive(s_flush_lock);
}

