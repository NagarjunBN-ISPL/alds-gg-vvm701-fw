#pragma once
#include <stdbool.h>
#include <stdint.h>

/*
 * rs485_bus.h — shared ownership of the RS485 bus (UART1).
 *
 * WHY THIS EXISTS
 * ---------------
 * Four drivers talk over this one UART, and they no longer all run on the
 * same task:
 *
 *   dispenser.c  BlueSky pumps   8E1 @ 9600      } du_task, core 1
 *   yenen.c      Yenen pumps     8N1 @ 9600      }
 *   radar.c      tank gauges     8N1 @ 9600      } main task, core 0
 *   genset.c     DG controller   8N1 @ configurable baud
 *
 * Dispenser polling moved to its own task so it survives a network outage —
 * the recovery ladder blocks the main task for minutes at a time (measured:
 * 290 s with zero dispenser reads), and a buffer is worthless if nothing is
 * being read to put in it. Radar and genset stayed on the main task because
 * their data is sampled state, not transactions: the reading after recovery
 * is current, so losing the ones during an outage costs nothing.
 *
 * That split means two tasks now drive one UART, so every transaction has to
 * be serialised. Note this is not only about interleaved bytes — each driver
 * also RECONFIGURES the port (parity for BlueSky, baud for genset). A mode
 * change landing between another driver's write and read corrupts the reply
 * just as badly as overlapping traffic would.
 *
 * ── Lock discipline ───────────────────────────────────────────────────────
 * Hold the lock across the WHOLE exchange: mode set, write, and read. The
 * rs485_bus_set_*() helpers deliberately do not take it themselves, because
 * doing so would leave a window between configuring the port and using it.
 *
 * NEVER hold this lock across a flash write or an MQTT publish. Both can
 * block for hundreds of milliseconds and neither needs the bus; holding it
 * there is how a slow publish turns into a missed dispenser poll.
 */

/* Install and configure UART1, and create the bus lock. Idempotent. */
void rs485_bus_init(void);

/* Take the bus for one complete exchange.
 *
 * Always pass a real timeout — never block forever. A timeout means "this
 * transaction did not happen": the caller should skip that read and try again
 * next cycle, which degrades to a single "404" rather than stalling a task
 * and eventually tripping the watchdog. */
bool rs485_bus_lock(uint32_t timeout_ms);

/* Release the bus. Every successful lock must be paired with exactly one of
 * these on EVERY exit path, including error returns. */
void rs485_bus_unlock(void);

/* Failed lock acquisitions since boot, cumulative. Every one of these cost a
 * skipped dispenser round, radar/genset poll, or remote command — see
 * rs485_bus_lock(). Published in the diagnostics report as bus.lock_timeouts. */
uint32_t rs485_bus_lock_timeouts(void);

/* ── Line-mode helpers — call with the bus lock held ──────────────────────
 * Each waits for the TX FIFO to drain before reconfiguring: uart_write_bytes()
 * returns once bytes are queued, not once they have left the pin, so changing
 * parity or baud too early corrupts the tail of the previous frame. */
void rs485_bus_set_8n1(uint32_t baud);   /* radar, genset          */
void rs485_bus_set_bluesky(void);        /* BlueSky: 8E1 @ 9600    */
void rs485_bus_set_yenen(void);          /* Yenen:   8N1 @ YENEN_BAUD */
