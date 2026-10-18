#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* ── Post-mortem breadcrumbs — why the gateway died ────────────────────────
 * A diagnostics report published every 15 minutes can never explain a hang
 * that happens after it. The explanation has to survive the reboot, so it
 * lives in RTC_NOINIT memory: retained across esp_restart(), a panic, and
 * every watchdog reset, lost only on true power loss. The same mechanism the
 * relay deadman already uses (relay.c), for the same reason.
 *
 * A core dump would be the fuller answer, but partitions.csv is fully
 * allocated and explicitly NOT OTA-updatable — adding a coredump partition
 * means serial-reflashing every deployed unit with a signed image. This costs
 * 40 bytes of RTC RAM and works on units already in the field.
 *
 * ── What it records ───────────────────────────────────────────────────────
 * One slot per watchdog-registered task. Each task stamps its own slot as it
 * enters a phase — two stores, no lock, safe from any context. On the next
 * boot the slots say:
 *
 *   which task stopped progressing  — the one whose stamp is furthest behind
 *   what it was doing               — that slot's phase + detail
 *
 * Only tasks that actually ran are candidates: an untouched slot means the task
 * was never started (di_task with diEnable 0, relay_task before its first
 * command), which is absence, not a stall. Every task that IS running must mark
 * periodically, or it will be blamed for a stall it did not cause.
 *
 * esp_reset_reason() says HOW it died; this says WHERE. A hang inside
 * store_compact and a hang inside a Yenen read are completely different bugs
 * and are indistinguishable without it.
 *
 * ── Why it reports clean restarts too ─────────────────────────────────────
 * An earlier cut of this only reported abnormal resets, so the presence of the
 * record was itself the alert. That misses the two restarts that most need
 * explaining, because both are deliberate esp_restart() calls: the low-heap
 * guard and the exhausted recovery ladder. Neither announces itself over MQTT
 * (there is no session up when they fire), so both would have been invisible.
 *
 * So the record is emitted whenever there is retained history, and carries an
 * explicit "fault" flag for the abnormal cases. The backend reads severity
 * from that flag rather than from the key's presence, and gets the deliberate
 * restarts — with the phase that caused them — for free. */

/* One slot per long-lived, watchdog-registered task. Keep in sync with
 * f_slot_name[] in forensics.c. */
typedef enum {
    FSLOT_MAIN = 0,
    FSLOT_DU,
    FSLOT_DI,
    FSLOT_RELAY,
    FSLOT_COUNT,
} FSlotId_t;

/* Where the firmware was. Application-specific on purpose: "blocked in a UART
 * read" is not actionable, "blocked reading Yenen head 2 side B" is. Keep in
 * sync with f_phase_name[] in forensics.c. */
typedef enum {
    F_NONE = 0,
    F_BOOT_NVS,        /* NVS init / config load / credential enforcement     */
    F_BOOT_MODEM,      /* modem power-on, PWRKEY sequence                     */
    F_BOOT_NET,        /* SIM, registration, PDP context                      */
    F_BOOT_MQTT,       /* first broker connect + subscribe                    */
    F_BOOT_STORE,      /* littlefs mount                                      */
    F_IDLE,            /* between poll cycles — the healthy resting phase     */
    F_POLL_BLUESKY,    /* detail = global nozzle label                        */
    F_POLL_YENEN,      /* detail = global nozzle label                        */
    F_POLL_RADAR,      /* detail = tank index (1-based)                       */
    F_POLL_GENSET,
    F_POLL_DI,
    F_POLL_GPS,
    F_PUBLISH,         /* inside a modem publish round-trip                   */
    F_STORE_ENQ,       /* littlefs append                                     */
    F_STORE_FLUSH,     /* draining the offline queue                          */
    F_STORE_COMPACT,   /* whole-log rewrite — the long one                    */
    F_SETRSP,          /* retired with setrsp (2026-10-18); value kept        */
    F_PASSTHRU,        /* raw rsp/kfactor tunnel command                      */
    F_MS51,            /* Nuvoton transaction (panel lock / DI)               */
    F_FOTA_DL,         /* HTTPS download                                      */
    F_FOTA_WRITE,      /* flash write of the new image                        */
    F_RECOVER,         /* connectivity ladder; detail = level 1..4            */
    F_DIAG,            /* building/publishing this very report                */
    F_PHASE_COUNT,
} FPhase_t;

/* Call FIRST in app_main, before anything else can mark a phase. Latches
 * whatever the previous boot left behind, then re-arms the slots for this one.
 * Safe on a cold boot: an unset magic simply means "no history". */
void forensics_init(void);

/* Stamp a task's slot. Cheap enough for a per-nozzle call site: two stores and
 * a timer read, no lock, no allocation. `detail` is phase-specific (see the
 * enum above); pass 0 where the phase has no natural parameter. */
void forensics_mark(FSlotId_t slot, FPhase_t phase, uint8_t detail);

/* True if the previous boot left readable state, i.e. this was not a cold
 * power-on and the record has not yet been published and cleared. */
bool forensics_have_history(void);

/* True if that previous stop was abnormal (panic, either watchdog, brownout)
 * rather than a deliberate restart. Feeds the health verdict, and mirrors the
 * "fault" member inside the JSON so the two can never disagree. */
bool forensics_last_was_fault(void);

/* Write the "last_stop" JSON object VALUE (a complete {...}) into buf.
 * Returns bytes written, or 0 if there is nothing to report or it would not
 * fit — in which case the caller must omit the key entirely rather than emit
 * a broken or empty object. */
int forensics_stop_json(char *buf, size_t size);

/* Clear the retained record. Call ONLY after the report carrying it has been
 * confirmed published, so a stop that coincides with an outage is not lost to
 * that same outage. */
void forensics_stop_clear(void);
