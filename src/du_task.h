#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * du_task.h — dispenser acquisition task.
 *
 * WHY DISPENSER POLLING LEFT THE MAIN LOOP
 * ----------------------------------------
 * The connectivity recovery ladder (mqtt_reconnect_if_needed -> network
 * rebuild -> modem power-cycle -> restart) runs on the main task and blocks
 * it for the whole recovery. Measured on the bench 2026-08-19: pulling the
 * SIM produced 290 seconds with not a single dispenser read, ending in a
 * restart. Any store-and-forward buffer bolted onto that main loop would have
 * captured one poll round and then nothing, because nothing was being read.
 *
 * So acquisition moved to its own task on core 1. The ladder is deliberately
 * NOT modified — reconnection behaviour is unchanged, it simply no longer
 * takes the pumps down with it. This is the standard producer/consumer split
 * for telemetry: the acquisition task is never gated on link state, and a
 * queue absorbs the difference.
 *
 * The task runs ALWAYS, not only during outages. An outage-only path would be
 * the least-exercised code in the firmware — first proven in the field, during
 * the exact failure it exists to handle. Running it every cycle means normal
 * operation continuously tests the outage path.
 *
 * ── Ownership ─────────────────────────────────────────────────────────────
 *   du_task (core 1)   polls dispensers, builds payloads, enqueues
 *   main task (core 0) drains the queue and publishes; owns the modem
 *
 * Only the main task ever touches the modem, so there is no modem lock and no
 * contention on it. Radar and genset stay on the main task and share the
 * RS485 bus with this task through rs485_bus_lock().
 */

/* Start the acquisition task. Call after rs485_bus_init(), nvs_config_load()
 * and build_hw_ids(), since the task reads the live config and HW_IDs.
 * Returns false if the queue or task could not be created — the caller should
 * treat that as "dispensers will not be polled" and say so loudly. */
bool du_task_start(void);

/* ── Peek / release — the main task never holds a record outside the queue ──
 * Copy the oldest payload WITHOUT removing it, and hand back a token that
 * identifies it. Returns false if the queue is empty. Never blocks.
 *
 * The record is removed only by du_task_release() after its publish is
 * confirmed, mirroring store_peek/store_pop. That is not symmetry for its own
 * sake — it closes a real hole:
 *
 *   Dequeue-then-publish leaves the record in a local buffer, owned by nobody,
 *   for as long as the publish takes (up to 15 s on a dying link). If du_task
 *   fills the queue in that window, there is nowhere to put the record back:
 *   it is dropped. Reserving a spare slot for the put-back — the obvious fix —
 *   makes it WORSE, because du_emit() then spills the whole RAM queue to flash
 *   one slot early, and the in-flight record (the OLDEST of the run) gets
 *   returned to RAM while everything newer than it is already on flash. Flash
 *   is drained first, so it would publish newest-before-oldest: the exact
 *   inversion that produces negative totalizer deltas and phantom sales.
 *
 * Leaving the record IN the queue removes the hole rather than patching it.
 * There is nothing to put back, nothing to drop, and if du_task spills the
 * queue mid-publish the record goes to flash in order with its neighbours.
 * The cost is a possible duplicate publish in that rare case, which the
 * at-least-once contract already covers. */
bool du_task_peek(char *buf, size_t buf_size, uint32_t *token);

/* Discard the record identified by `token` — call only after its publish is
 * confirmed. Returns false without losing anything if the queue moved under
 * the caller (the record was spilled to flash meanwhile); the caller should
 * simply stop draining for this cycle. */
bool du_task_release(uint32_t token);
void du_task_flush_to_store(void);

/* Number of payloads waiting in the RAM queue. */
int du_task_pending(void);

/* Task handle for stack-high-water reporting in diagnostics; NULL if the
 * task never started. */
TaskHandle_t du_task_handle(void);

/* Counters for the diagnostics report. `spilled` is payloads written to flash
 * because the RAM queue was full — a non-zero value means an outage was
 * survived. `dropped` means both RAM and flash refused, i.e. real data loss. */
unsigned du_task_spilled(void);
unsigned du_task_dropped(void);

/* Nozzles currently demoted to probe-only by the absent-nozzle backoff, as a
 * bitmask over global nozzle labels (bit 0 = label 1, up to label 22).
 * Published as bus.nozzles_absent so a silent nozzle can be named instead of
 * collapsing into a single du_ok:false for the whole site. */
uint32_t du_task_absent_mask(void);
