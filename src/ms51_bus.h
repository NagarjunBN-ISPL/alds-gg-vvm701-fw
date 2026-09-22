#pragma once
#include <stdbool.h>
#include <stdint.h>

/*
 * ms51_bus.h — shared ownership of the Nuvoton MS51 link (UART0).
 *
 * WHY THIS EXISTS
 * ---------------
 * Two independent subsystems talk to the MS51 co-processor over the same
 * UART0:
 *
 *   di.c     polls "GETDATA" every ~2 s for the digital inputs
 *   relay.c  sends "RL1ON"/"RL1OFF" to drive the panel-box lock solenoid
 *
 * In the firmware relay.c was ported from (OTA_Integration) there was no
 * di.c, so relay.c legitimately owned UART0 outright — its own comment says
 * it is "the only place that touches the UART after init". That assumption is
 * false here, and porting it unchanged breaks in two ways:
 *
 *   1. Both modules called uart_driver_install() on UART0. The second call
 *      returns ESP_ERR_INVALID_STATE, and relay.c wraps it in
 *      ESP_ERROR_CHECK() — an abort() at boot.
 *
 *   2. Worse, and silent: di_task (prio 5) and relay_task (prio 10) would
 *      interleave write-then-read transactions on one UART. relay's reply
 *      read can consume di's GETDATA response and vice versa, and a garbled
 *      "RL1OFF" is simply not understood by the MS51 — which leaves the panel
 *      box UNLOCKED. That is a physical-security failure, not a data glitch,
 *      so it is arbitrated properly rather than left to timing luck.
 *
 * This module owns the UART and hands out a transaction lock. Neither caller
 * installs the driver itself any more.
 */

/* Install and configure UART0 for the MS51, and create the transaction lock.
 * Idempotent and safe to call from several places at boot in any order —
 * di_uart_init() and relay_init() both call it and neither needs to know
 * which ran first. */
void ms51_bus_init(void);

/* Take the transaction lock for one complete write-then-read exchange.
 *
 * Returns true if the lock was acquired OR if no lock exists (mutex
 * allocation failed at boot). The no-lock case deliberately lets the caller
 * PROCEED rather than bail: an unarbitrated relay command may race with a DI
 * poll, but a relay command that never runs at all can leave the lock
 * solenoid energized and the panel box open. Racing is recoverable; not
 * sending RL1OFF is not.
 *
 * Returns false only on a genuine timeout, which the caller should treat as
 * "this transaction did not happen". */
bool ms51_bus_lock(uint32_t timeout_ms);

/* Release the transaction lock. Safe to call after ms51_bus_lock() returned
 * true, including the no-lock case. */
void ms51_bus_unlock(void);
