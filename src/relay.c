#include "relay.h"
#include "forensics.h"
#include "config.h"
#include "ms51_bus.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define TAG "RELAY"

/* Survives esp_restart() (not power loss) — set right before the deadman
 * timer forces a restart, read once by relay_init() on the next boot. */
RTC_NOINIT_ATTR static uint32_t s_safety_restart_marker;
#define SAFETY_RESTART_MAGIC 0xA17B00CEu

/*
 * Each safety-critical relay command is sent back-to-back this many times.
 * RL1ON/RL1OFF are idempotent, so repeating costs nothing and tolerates a
 * single UART frame dropped by EMI on the isolated link.
 *
 * IMPORTANT — this is an OPEN-LOOP mitigation. The MS51's relay-command ACK
 * format is unconfirmed (the vendor reference firmware issues RL1ON/RL1OFF
 * without reading any reply), so firmware cannot positively confirm the
 * physical relay state from UART alone. Real closed-loop confirmation needs
 * either (a) the MS51's actual ack string, captured on the bench, then gate
 * state on it, or (b) a hardware feedback line — a spare relay aux contact or
 * a current-sense input wired back to an ESP32 GPIO. Until then, treat
 * s_locked as the *intended* state, not a *verified* one.
 */
#define NUVOTON_CMD_REPEATS 2

typedef enum {
    RELAY_CMD_UNLOCK = 0,
    RELAY_CMD_LOCK,
} relay_cmd_t;

static esp_timer_handle_t s_window_timer;
static esp_timer_handle_t s_deadman_timer;
static QueueHandle_t      s_cmd_q;
static volatile bool      s_locked = true;
static bool               s_boot_was_safety_restart;
/* When the CURRENT unlock's deadman was armed. Only meaningful while that
 * timer is active; used to shorten a refreshed window so it lands inside the
 * ceiling (see apply_unlock). Touched only by the relay task. */
static int64_t            s_deadman_armed_us;
/* The lock subsystem's "am I usable" flag is simply s_cmd_q != NULL. relay_init
 * leaves it NULL if any init step fails (see there); relay_unlock/force_lock
 * treat NULL as "disabled, box stays locked". No separate ready flag is kept,
 * so there is exactly one source of truth. */

/* ── Nuvoton link ───────────────────────────────────────────────────────── */

/* Send one command; log any reply for bench diagnostics. Returns true if the
 * MS51 replied with any bytes within read_ms. State is deliberately NOT gated
 * on the reply for relay commands — see NUVOTON_CMD_REPEATS note. Queries like
 * GETDATA/RSTDATA need a longer read_ms (the MS51 answers in ~1 s). */
static bool nuvoton_txn(const char *cmd, int read_ms)
{
    /* PORTING NOTE (alds-sr): this firmware also runs di.c, which polls the
     * same MS51 over UART0 every ~2 s — unlike OTA_Integration, where relay.c
     * owned the UART alone. The whole flush/write/read exchange is therefore
     * taken under the shared bus lock so a DI poll cannot swallow this
     * command's reply, or vice versa. See ms51_bus.h for why a garbled RL1OFF
     * matters more than a garbled GETDATA.
     *
     * Timeout budget: a DI poll holds the bus for at most NUVOTON_TIMEOUT_MS
     * (3 s) when the MS51 is silent, so the wait must exceed that or a relay
     * command could be dropped simply because DI was mid-poll. It still sits
     * well inside RELAY_UNLOCK_WINDOW_MS (8 s) and the deadman (11 s). */
    if (!ms51_bus_lock(MS51_BUS_LOCK_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "%s: MS51 bus busy — command NOT sent", cmd);
        return false;
    }

    uart_flush_input(NUVOTON_UART);

    char line[16];
    int  n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    uart_write_bytes(NUVOTON_UART, line, n);

    uint8_t resp[96];
    int len = uart_read_bytes(NUVOTON_UART, resp, sizeof(resp) - 1, pdMS_TO_TICKS(read_ms));

    ms51_bus_unlock();

    if (len > 0) {
        resp[len] = '\0';
        /* Trim trailing CR/LF for a clean single-line log. */
        while (len > 0 && (resp[len - 1] == '\r' || resp[len - 1] == '\n'))
            resp[--len] = '\0';
        ESP_LOGI(TAG, "%s -> %s", cmd, (char *)resp);
        return true;
    }
    ESP_LOGD(TAG, "%s -> (no reply)", cmd);
    return false;
}

/* Send a relay command redundantly. Returns true if the MS51 replied to at
 * least one attempt. */
static bool nuvoton_relay_cmd(const char *cmd)
{
    bool replied = false;
    for (int i = 0; i < NUVOTON_CMD_REPEATS; i++)
        replied |= nuvoton_txn(cmd, 200);
    return replied;
}

/* ── Relay task — the only place that touches the UART after init, so relay
 * state and the Nuvoton link have single-threaded ownership (no mutex). ──── */

static void apply_lock(void)
{
    if (esp_timer_is_active(s_window_timer))  esp_timer_stop(s_window_timer);
    if (esp_timer_is_active(s_deadman_timer)) esp_timer_stop(s_deadman_timer);
    nuvoton_relay_cmd("RL1OFF");
    s_locked = true;
    ESP_LOGI(TAG, "LOCKED (RL1 de-energized; spring latch re-locks on close)");
}

/* How long a window may run, given how long ago this unlock's deadman was
 * armed: RELAY_UNLOCK_WINDOW_MS for a fresh unlock and progressively less for
 * each repeat, so a refreshed window always ends RELAY_RELOCK_MARGIN_MS before
 * the deadman does. 0 = no room left to open at all. This is what makes
 * pressing the button repeatedly safe: the relock stays in the ordinary path
 * instead of escalating to the deadman's restart. (cng-cpil lock_cmd.c) */
static uint32_t window_ms_clamped(uint32_t elapsed_since_deadman_ms)
{
    if (elapsed_since_deadman_ms >= RELAY_UNLOCK_DEADMAN_MS) return 0;
    const uint32_t remaining = RELAY_UNLOCK_DEADMAN_MS - elapsed_since_deadman_ms;
    if (remaining <= RELAY_RELOCK_MARGIN_MS) return 0;
    const uint32_t usable = remaining - RELAY_RELOCK_MARGIN_MS;
    return (usable < RELAY_UNLOCK_WINDOW_MS) ? usable : RELAY_UNLOCK_WINDOW_MS;
}

static void apply_unlock(void)
{
    const int64_t now = esp_timer_get_time();

    if (esp_timer_is_active(s_window_timer)) esp_timer_stop(s_window_timer);

    /* HIGH-8: the deadman is an ABSOLUTE ceiling. A refreshed window may push
     * the normal relock out, but repeated unlocks must NOT keep resetting the
     * hard limit — only arm it on a fresh unlock, never restart a live one.
     * Armed FIRST, before the window and before energizing, so it is
     * guaranteed live even if the RL1ON transaction is slow or the task later
     * stalls. */
    if (!esp_timer_is_active(s_deadman_timer)) {
        s_deadman_armed_us = now;
        if (esp_timer_start_once(s_deadman_timer, (uint64_t)RELAY_UNLOCK_DEADMAN_MS * 1000ULL) != ESP_OK)
            ESP_LOGE(TAG, "DEADMAN FAILED TO ARM — unlock proceeding with the window "
                          "timer as the only relock");
    }

    /* Clamp the window so it always ends BEFORE the deadman — see
     * RELAY_RELOCK_MARGIN_MS in config.h for why. */
    const uint32_t elapsed_ms = (uint32_t)((now - s_deadman_armed_us) / 1000);
    const uint32_t window_ms  = window_ms_clamped(elapsed_ms);
    if (window_ms == 0) {
        /* No room left to open. Relock now rather than start a window that
         * cannot finish — the box is about to be shut either way, and doing it
         * here keeps it out of the deadman's restart. */
        ESP_LOGW(TAG, "unlock refused — deadman ceiling reached, relocking");
        apply_lock();
        return;
    }
    if (esp_timer_start_once(s_window_timer, (uint64_t)window_ms * 1000ULL) != ESP_OK)
        ESP_LOGE(TAG, "window timer failed to start — relying on the deadman");

    nuvoton_relay_cmd("RL1ON");
    s_locked = false;
    ESP_LOGW(TAG, "UNLOCK window %lums (deadman ceiling in %lums)",
             (unsigned long)window_ms, (unsigned long)(RELAY_UNLOCK_DEADMAN_MS - elapsed_ms));
}

static void relay_task(void *arg)
{
    /* PORTING CHANGE (alds-sr): registered with the task watchdog, which the
     * original was not. This drives a physical lock, and a wedged relay task
     * fails SILENTLY — commands queue up and are never applied, so a "lock"
     * push would be acknowledged over MQTT while the box stayed open. The
     * deadman timer only covers an unlock window that is already in progress;
     * it does nothing for a task stuck while idle.
     *
     * The wait had to become bounded to do this: a WDT-registered task that
     * blocks on portMAX_DELAY trips the watchdog as soon as the bus goes quiet.
     * xQueueReceive still returns the instant a command arrives, so command
     * latency is unchanged — the timeout only exists to give the loop a chance
     * to feed the WDT while idle. */
    esp_task_wdt_add(NULL);

    relay_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(s_cmd_q, &cmd, pdMS_TO_TICKS(RELAY_TASK_TICK_MS)) == pdTRUE) {
            switch (cmd) {
                case RELAY_CMD_UNLOCK: apply_unlock(); break;
                case RELAY_CMD_LOCK:   apply_lock();   break;
            }
        }
        /* One mark per tick (RELAY_TASK_TICK_MS) whether or not a command
         * arrived, so an idle-but-alive relay task is distinguishable from a
         * wedged one. */
        forensics_mark(FSLOT_RELAY, F_MS51, 0);
        esp_task_wdt_reset();
    }
}

/* ── Timer callbacks — minimal work only; post to the relay task (esp_timer
 * callbacks run in a shared task and must never block). ─────────────────── */

static void window_timer_cb(void *arg)
{
    (void)arg;
    relay_cmd_t c = RELAY_CMD_LOCK;
    /* Non-blocking — this runs in the shared esp_timer task and must not
     * stall. If the queue is full of unlock commands the relock would be
     * dropped and the box would stay open to the deadman, so clear them and
     * try once more: every one of them is an unlock we are overriding anyway.
     * xQueueReset does not block. */
    if (xQueueSend(s_cmd_q, &c, 0) != pdTRUE) {
        (void)xQueueReset(s_cmd_q);
        if (xQueueSend(s_cmd_q, &c, 0) != pdTRUE)
            ESP_LOGE(TAG, "RELOCK COULD NOT BE QUEUED — the deadman is now the only "
                          "thing that will close the box");
    }
}

static void deadman_timer_cb(void *arg)
{
    (void)arg;
    /* Independent hard backstop: do NOT route through the relay task — it may
     * be the thing that's stuck. Mark and restart directly; boot re-locks. */
    ESP_LOGE(TAG, "LOCK SAFETY: relock deadline exceeded — forcing restart");
    s_safety_restart_marker = SAFETY_RESTART_MAGIC;
    esp_restart();
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void relay_init(void)
{
    /* PORTING CHANGE (alds-sr): the original installed the UART0 driver here
     * with ESP_ERROR_CHECK. In this firmware di_uart_init() has already
     * installed it unconditionally at boot (main.c), so that call would return
     * ESP_ERR_INVALID_STATE and ESP_ERROR_CHECK would abort() before the boot
     * force-lock below ever ran — i.e. the panic would happen at exactly the
     * moment the lock most needs commanding. ms51_bus_init() is idempotent and
     * owns the port for both callers; the pin/baud configuration is identical. */
    ms51_bus_init();

    /* MS51 init + link health probe. The vendor firmware issues RSTDATA once
     * at startup and reads the reply, both to put the co-processor in a known
     * state and (for us) to confirm the UART0 link is alive BOTH ways. Relay
     * commands (RL1ON/RL1OFF) may not ack, so this is the only positive proof
     * the MS51 is actually reachable — watch this line in the boot log. */
    if (nuvoton_txn("RSTDATA", 1500))
        ESP_LOGI(TAG, "MS51 link OK (RSTDATA replied)");
    else
        ESP_LOGW(TAG, "MS51 link: NO reply to RSTDATA — check UART0 wiring "
                      "(GPIO43 TXD0->MS51 RX, GPIO44 RXD0<-MS51 TX) and MS51 power");

    /* Boot force-lock — the single most important command. Retried, run
     * synchronously in the caller's (main) task where blocking is fine. */
    bool replied = false;
    for (int i = 0; i < 3 && !replied; i++) {
        replied = nuvoton_relay_cmd("RL1OFF");
        if (!replied) vTaskDelay(pdMS_TO_TICKS(100));
    }
    s_locked = true;   /* intended fail-safe state; physical state unverifiable without feedback */
    if (!replied)
        ESP_LOGW(TAG, "Boot RL1OFF got no MS51 reply "
                      "(expected if MS51 does not ack relay cmds — verify on bench)");

    s_boot_was_safety_restart = (s_safety_restart_marker == SAFETY_RESTART_MAGIC);
    s_safety_restart_marker   = 0;
    if (s_boot_was_safety_restart)
        ESP_LOGE(TAG, "PREVIOUS BOOT WAS A LOCK SAFETY RESTART — the box did not relock on its own");

    /* The timing policy is compiled constants, so this can only fail after
     * someone edits them — which is exactly when it should be caught. */
    _Static_assert(RELAY_UNLOCK_DEADMAN_MS > RELAY_UNLOCK_WINDOW_MS,
                   "deadman must sit ABOVE the normal relock or it fires on every unlock");
    _Static_assert(RELAY_UNLOCK_DEADMAN_MS >= RELAY_UNLOCK_WINDOW_MS + RELAY_RELOCK_MARGIN_MS,
                   "a FIRST unlock must get its full window without being clamped");
    _Static_assert(RELAY_TASK_TICK_MS < RELAY_UNLOCK_WINDOW_MS,
                   "relay task must get at least one tick inside the unlock window");
    _Static_assert(RELAY_RELOCK_MARGIN_MS > 0, "relock margin must cover the MS51 exchange");

    /* ── Relay subsystem init — DEGRADE, do not abort() ───────────────────
     * The box has already been force-LOCKED above (the synchronous boot
     * RL1OFF), so by the time we get here the fail-safe state is achieved. If
     * the timers/queue/task cannot be created — realistically only heap
     * exhaustion at boot — the right thing is NOT to abort() the whole gateway:
     * that panics a device whose fuel-metering job is otherwise fine, and if
     * the shortage is persistent it becomes a boot loop at a fuel station.
     *
     * Instead we leave the lock subsystem DISABLED (s_cmd_q left NULL) and keep
     * running. relay_unlock()/relay_force_lock() below no-op in that state,
     * so the box simply stays locked — the safe direction — and the gateway
     * keeps polling and publishing. The condition is surfaced loudly in the log
     * and, via relay_is_locked() staying true, is not mistaken for "unlocked".
     *
     * The panel can still be opened mechanically on site; what is lost is
     * REMOTE unlock, which is strictly less important than the device staying
     * up and metering. */
    const esp_timer_create_args_t window_args  = { .callback = window_timer_cb,  .name = "relay_window"  };
    const esp_timer_create_args_t deadman_args = { .callback = deadman_timer_cb, .name = "relay_deadman" };
    if (esp_timer_create(&window_args,  &s_window_timer)  != ESP_OK ||
        esp_timer_create(&deadman_args, &s_deadman_timer) != ESP_OK) {
        ESP_LOGE(TAG, "relay timer create failed (heap OOM?) — REMOTE LOCK/UNLOCK "
                      "DISABLED, box stays locked, gateway continues");
        s_cmd_q = NULL;
        return;
    }

    s_cmd_q = xQueueCreate(8, sizeof(relay_cmd_t));
    if (!s_cmd_q) {
        ESP_LOGE(TAG, "relay cmd queue alloc failed (heap OOM?) — REMOTE LOCK/UNLOCK "
                      "DISABLED, box stays locked, gateway continues");
        /* Free the two timers created just above — parity with the task-create
         * failure branch below (harmless either way since they are never armed,
         * but it keeps the failure paths consistent). */
        esp_timer_delete(s_window_timer);
        esp_timer_delete(s_deadman_timer);
        return;
    }
    if (xTaskCreate(relay_task, "relay", 3072, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "relay task create failed (heap OOM?) — REMOTE LOCK/UNLOCK "
                      "DISABLED, box stays locked, gateway continues");
        vQueueDelete(s_cmd_q);
        s_cmd_q = NULL;   /* NULL so unlock/lock reject outright, not silently queue */
        esp_timer_delete(s_window_timer);
        esp_timer_delete(s_deadman_timer);
        return;
    }

    ESP_LOGI(TAG, "Lock relay init: UART%d TX=GPIO%d RX=GPIO%d @%dbaud  window=%dms deadman=%dms  (fail-locked)%s",
             NUVOTON_UART, NUVOTON_TX_PIN, NUVOTON_RX_PIN, NUVOTON_BAUD,
             RELAY_UNLOCK_WINDOW_MS, RELAY_UNLOCK_DEADMAN_MS,
             s_boot_was_safety_restart ? "  [PREV BOOT: LOCK SAFETY RESTART]" : "");
}

void relay_unlock(void)
{
    if (!s_cmd_q) {
        ESP_LOGW(TAG, "unlock ignored — relay subsystem disabled (box stays locked)");
        return;
    }
    relay_cmd_t c = RELAY_CMD_UNLOCK;
    if (xQueueSend(s_cmd_q, &c, pdMS_TO_TICKS(100)) != pdTRUE)
        ESP_LOGE(TAG, "unlock command dropped (queue full)");
}

void relay_force_lock(void)
{
    if (!s_cmd_q) {
        /* Subsystem disabled — but the box is already locked (boot RL1OFF and
         * the fail-safe default), so a lock request is already satisfied. */
        ESP_LOGW(TAG, "force-lock: relay subsystem disabled — box already locked");
        return;
    }
    /* xQueueSendToFront, NOT ToBack: a LOCK is the safety-critical direction and
     * must not wait behind pending unlocks. If a rapid-fire or stuck cloud
     * actor has filled the queue with unlock commands, the lock jumps ahead of
     * them so the box is secured first. (The deadman timer is the ultimate
     * backstop if even this is dropped: an unlock window that never closes
     * forces a restart, which re-locks at boot.) */
    relay_cmd_t c = RELAY_CMD_LOCK;
    if (xQueueSendToFront(s_cmd_q, &c, pdMS_TO_TICKS(100)) != pdTRUE) {
        /* A LOCK MUST NEVER BE DROPPED. Eight queued unlocks would otherwise
         * be enough to make the safety-critical direction the one that fails.
         * Discarding them is correct rather than merely expedient: the operator
         * has since asked for the box to be shut, so every pending unlock is
         * already superseded. */
        ESP_LOGW(TAG, "lock queue full — discarding pending unlocks");
        (void)xQueueReset(s_cmd_q);
        if (xQueueSendToFront(s_cmd_q, &c, pdMS_TO_TICKS(100)) != pdTRUE)
            ESP_LOGE(TAG, "LOCK COMMAND DROPPED — relying on the window timer and the deadman");
    }
}

bool relay_is_locked(void)
{
    return s_locked;
}

bool relay_last_boot_was_safety_restart(void)
{
    /* Set once in relay_init() before the relay task exists and read once from
     * app_main after MQTT is up — no concurrent writer, so a plain read-clear
     * is safe here. */
    bool v = s_boot_was_safety_restart;
    s_boot_was_safety_restart = false;
    return v;
}
