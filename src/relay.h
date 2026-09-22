#pragma once
#include <stdbool.h>

/*
 * relay.c — Panel-box solenoid lock driver (VVM701 / Nuvoton MS51FB9AE).
 *
 * RL1 is commanded over UART0 (see config.h) with ASCII RL1ON/RL1OFF —
 * the relay coil state lives in the Nuvoton co-processor, not in ESP32
 * GPIO. Fail-locked / energize-to-unlock: RL1 is OFF (locked) at boot and
 * whenever idle.
 *
 * Lock mechanism: spring-return latch. relay_unlock() energizes RL1 to
 * retract the bolt for RELAY_UNLOCK_WINDOW_MS (the person's window to pull
 * the door open), then de-energizes. Re-locking is MECHANICAL — the
 * spring-loaded bolt snaps back into the strike when the door is next
 * pushed shut, however long that takes. The firmware never holds the door
 * open and never keeps the coil energized beyond the window.
 *
 * Two cloud commands map onto this:
 *   authorize unlock -> relay_unlock()      (8 s window, then auto de-energize)
 *   force lock       -> relay_force_lock()   (immediate RL1OFF: lock early / abort)
 *
 * Because the coil state is held off-chip, a crash on the ESP32 side alone
 * (after RL1ON but before RL1OFF) can't be corrected by GPIO reset — so
 * three independent layers guarantee it can't stay unlocked indefinitely:
 *   1. Window timer  — sends RL1OFF after RELAY_UNLOCK_WINDOW_MS (normal case).
 *   2. Deadman timer — independent of (1); if the window hasn't closed by
 *      RELAY_UNLOCK_DEADMAN_MS, forces esp_restart() rather than waiting on
 *      the shared 120 s task watchdog.
 *   3. Boot-time force-lock — relay_init() sends RL1OFF unconditionally as
 *      its first action on every boot, so any reset path (deadman timer,
 *      task WDT, brownout, power cycle) converges on locked.
 *
 * A mechanical key override on the box is assumed to exist independent of
 * this relay, per deployment design.
 */

void relay_init(void);

/* Authorize an unlock: energize RL1 for RELAY_UNLOCK_WINDOW_MS so the door
 * can be opened, then auto de-energize (spring latch re-locks on next
 * close). Safe to call again mid-window — restarts the window. */
void relay_unlock(void);

/* Force RL1 off immediately and cancel any pending timers. Idempotent.
 * Maps to the cloud "lock" command (lock early / abort an unlock). */
void relay_force_lock(void);

/* Current logical state: true = locked, false = unlock window in progress. */
bool relay_is_locked(void);

/* True if the previous boot was caused by the deadman timer firing (i.e.
 * an unlock window never closed normally). Read-and-clear — call once
 * after relay_init() and wire into a post-boot MQTT alert if desired. */
bool relay_last_boot_was_safety_restart(void);
