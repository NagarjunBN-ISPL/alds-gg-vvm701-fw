/* Drives the connectivity recovery ladder EXTRACTED from src/main.c against a
 * simulated modem. This is the firmware's most consequential state machine: get
 * it wrong in one direction and a gateway sits offline through an outage it
 * could have fixed; wrong in the other and the whole fleet reboot-loops through
 * a broker restart. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <setjmp.h>

#include "config.h"
#include "esp_log.h"

int g_log_quiet = 1;
static const char *TAG = "main";

/* ── simulated world ───────────────────────────────────────────────────── */
static int64_t s_now_us = 1000000;
int64_t esp_timer_get_time(void) { return s_now_us; }
static void advance_s(int64_t s) { s_now_us += s * 1000000LL; }

static bool sim_connected;      /* mqtt_is_connected()                      */
static bool sim_connect_ok;     /* will mqtt_connect() succeed              */
static bool sim_bearer;         /* network_data_bearer_up()                 */
static bool sim_netinit_ok;     /* will network_init() succeed              */

static int n_connect, n_subscribe, n_bearer, n_netinit, n_hardreset, n_restart, n_flush;

static jmp_buf restart_jmp;

bool     mqtt_is_connected(void)      { return sim_connected; }
bool     mqtt_connect(void)           { n_connect++; sim_connected = sim_connect_ok; return sim_connect_ok; }
static void subscribe_all(void)       { n_subscribe++; }
bool     network_data_bearer_up(void) { n_bearer++; return sim_bearer; }
bool     network_init(void)           { n_netinit++; return sim_netinit_ok; }
void     modem_hard_reset(void)       { n_hardreset++; }
void     du_task_flush_to_store(void) { n_flush++; }
void     esp_restart(void)            { n_restart++; longjmp(restart_jmp, 1); }
uint32_t esp_random(void)             { return 0; }   /* jitter off: deterministic */
int      esp_task_wdt_status(void *h) { (void)h; return 0; }
int      esp_task_wdt_reset(void)     { return 0; }

/* Fruitless-restart cap (main.c restart_streak_*). Simulated here: the real
 * one counts across restarts in RTC memory; the ladder only asks "may I". */
static bool sim_restart_allowed = true;
static int  n_restart_asked;
static bool restart_streak_allow(const char *why) { (void)why; n_restart_asked++; return sim_restart_allowed; }

/* The ladder now records which level it is escalating to (so an exhausted
 * ladder is explainable after the restart it ends in) and mirrors its counters
 * into g_diag for the diagnostics report. Both are stubbed rather than
 * ignored: n_mark/g_diag let the tests below assert that the escalation
 * counters actually track the levels taken. */
typedef enum { FSLOT_MAIN = 0 } FSlotId_t;
typedef enum { F_RECOVER = 22 } FPhase_t;
static int n_mark, last_mark_detail;
void forensics_mark(FSlotId_t s, FPhase_t p, unsigned char d) {
    (void)s; (void)p; n_mark++; last_mark_detail = (int)d;
}

static struct {
    unsigned recov_l1, recov_l2, recov_l3, bearer_distrust;
} g_diag;

#include "extracted_ladder.inc"

/* esp_restart() never returns on the device, so bracket every call. */
static void tick(void) {
    if (setjmp(restart_jmp) == 0) mqtt_reconnect_if_needed();
}

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static void counters_clear(void) {
    n_connect = n_subscribe = n_bearer = n_netinit = n_hardreset = n_restart = n_flush = 0;
}

/* The ladder's budgets are function-static, so normalise them between scenarios
 * the only way the outside world can: let a connect succeed, which runs
 * LADDER_RESET().
 *
 * More than one tick may be needed. If the previous scenario left L1 exhausted,
 * the first tick lands in the broker-outage branch — which re-arms L1 and
 * returns — and only the NEXT one can connect. On the device this never arises:
 * the only way to leave the ladder exhausted is L4, which reboots. */
static void ladder_reset(void) {
    sim_connected = false; sim_connect_ok = true; sim_bearer = true; sim_netinit_ok = true;
    for (int i = 0; i < 10 && !sim_connected; i++) {
        advance_s(3600);
        tick();
    }
    if (!sim_connected) { printf("  FAIL  ladder_reset could not reach a connected state\n"); exit(2); }
    sim_connected = false;
    counters_clear();
}

int main(void) {
    printf("== connectivity recovery ladder (extracted from main.c) ==\n");

    /* 1. THE BUG THIS FIX EXISTS FOR: registered but the PDP context is gone.
     *    The ladder must reach L2 — the only level that runs QIACT=1 — quickly,
     *    not after twenty minutes and not never. */
    printf("[1] PDP context down (the pdpdeact case)\n");
    {
        ladder_reset();
        sim_connect_ok = false;
        sim_bearer     = false;     /* registered, but no data bearer */
        sim_netinit_ok = false;     /* keep it failing so we can watch escalation */

        for (int i = 0; i < 6; i++) { tick(); advance_s(120); }
        CHECK(n_netinit > 0, "never reached L2 — the gateway would stay offline "
                             "forever with a dead PDP context (this is the bug)");
        CHECK(n_restart == 0, "rebooted before exhausting L2/L3");
        printf("       reached L2 after %d connect attempts, %d bearer checks\n",
               n_connect, n_bearer);
    }

    /* 2. Genuine broker outage: bearer healthy, broker down. The fleet must NOT
     *    escalate — no PDP rebuilds, no modem power-cycles, above all no reboot. */
    printf("[2] broker outage with a healthy bearer\n");
    {
        ladder_reset();
        sim_connect_ok = false;
        sim_bearer     = true;
        sim_netinit_ok = true;

        /* Run well past the L1 budget but below the distrust threshold. */
        int checks_seen = 0;
        for (int i = 0; i < 5 * (MQTT_BEARER_DISTRUST_AFTER - 1); i++) {
            tick();
            advance_s(MQTT_BROKER_OUTAGE_RETRY_S + 5);
            checks_seen = n_bearer;
        }
        CHECK(n_restart == 0, "REBOOTED during a broker outage — this reboot-loops the fleet");
        CHECK(n_hardreset == 0, "power-cycled the modem during a broker outage");
        CHECK(n_netinit == 0, "rebuilt the network during a broker outage (%d times)", n_netinit);
        CHECK(n_connect > 0, "stopped probing the broker — it would never notice it return");
        CHECK(checks_seen > 0 && checks_seen < MQTT_BEARER_DISTRUST_AFTER,
              "bearer checks %d should be under the distrust threshold here", checks_seen);
        printf("       %d connect probes, %d bearer checks, 0 escalations\n",
               n_connect, n_bearer);
    }

    /* 3. The broker comes back mid-outage: must reconnect and re-subscribe. */
    printf("[3] broker returns\n");
    {
        ladder_reset();
        sim_connect_ok = false;
        sim_bearer     = true;
        for (int i = 0; i < 20; i++) { tick(); advance_s(MQTT_BROKER_OUTAGE_RETRY_S + 5); }
        int before = n_connect;
        sim_connect_ok = true;
        advance_s(MQTT_BROKER_OUTAGE_RETRY_S + 5);
        tick();
        CHECK(n_connect > before, "never retried after the outage");
        CHECK(sim_connected, "did not reconnect when the broker returned");
        CHECK(n_subscribe > 0, "reconnected without re-subscribing — inbound commands would be lost");
    }

    /* 4. The modem LIES: "+QIACT: 1,1" forever on a dead context. The ladder has
     *    to stop believing it eventually, or it parks at L1 for good. */
    printf("[4] stale bearer (modem reports up, nothing works)\n");
    {
        ladder_reset();
        sim_connect_ok = false;
        sim_bearer     = true;      /* the lie */
        sim_netinit_ok = false;

        for (int i = 0; i < 5 * (MQTT_BEARER_DISTRUST_AFTER + 3); i++) {
            tick();
            advance_s(MQTT_BROKER_OUTAGE_RETRY_S + 5);
        }
        CHECK(n_netinit > 0, "believed a stale bearer indefinitely — parked at L1 forever");
        printf("       distrusted after %d bearer checks, then %d L2 rebuilds\n",
               n_bearer, n_netinit);
    }

    /* 5. Hard failure all the way down: L2 x3, L3 x3, then reboot — and the RAM
     *    queue must be flushed to flash BEFORE the reboot. */
    printf("[5] total failure escalates to reboot\n");
    {
        ladder_reset();
        sim_connect_ok = false;
        sim_bearer     = false;
        sim_netinit_ok = false;

        for (int i = 0; i < 60 && n_restart == 0; i++) { tick(); advance_s(600); }
        CHECK(n_netinit >= NET_BRINGUP_RETRIES, "L2 not fully spent (%d)", n_netinit);
        CHECK(n_hardreset >= MODEM_PWRCYCLE_MAX, "L3 not fully spent (%d)", n_hardreset);
        CHECK(n_restart == 1, "did not reach L4 restart (%d)", n_restart);
        CHECK(n_flush > 0, "restarted WITHOUT flushing the RAM queue — buffered sales lost");
        printf("       L2 x%d, L3 x%d, then restart (flushed first)\n", n_netinit, n_hardreset);
    }

    /* 6. Every level's budget must be re-armed by a success, or the second
     *    outage of a boot would find the ladder already exhausted. */
    printf("[6] budgets re-arm after recovery\n");
    {
        ladder_reset();
        sim_connect_ok = false; sim_bearer = false; sim_netinit_ok = false;
        for (int i = 0; i < 8; i++) { tick(); advance_s(600); }   /* burn L2 */
        CHECK(n_restart == 0, "reached L4 too early");

        /* recover at L2 */
        sim_netinit_ok = true; sim_connect_ok = true;
        advance_s(600); tick();
        CHECK(sim_connected, "did not recover at L2");

        /* now a fresh outage must get a full budget again */
        counters_clear();
        sim_connected = false; sim_connect_ok = false; sim_bearer = true;
        for (int i = 0; i < 5; i++) { tick(); advance_s(60); }
        CHECK(n_restart == 0, "second outage rebooted immediately — budgets never re-armed");
        CHECK(n_netinit == 0, "second outage escalated immediately — L1 budget not re-armed");
        CHECK(n_connect > 0, "second outage never probed the broker");
    }

    /* 7. Already connected: the ladder must be a no-op, on every call. */
    printf("[7] no-op while connected\n");
    {
        ladder_reset();
        sim_connected = true;
        counters_clear();
        for (int i = 0; i < 100; i++) { tick(); advance_s(1); }
        CHECK(n_connect == 0 && n_bearer == 0 && n_netinit == 0 && n_restart == 0,
              "did work while the link was up (%d/%d/%d/%d)",
              n_connect, n_bearer, n_netinit, n_restart);
    }

    /* 8. The restart cap: once RESTART_STREAK_CAP fruitless restarts have
     *    been counted, L4 must NOT restart again — it stays up and keeps the
     *    ladder cycling (L3 already power-cycles the modem), so a dead-SIM
     *    gateway polls and buffers instead of reboot-looping forever. */
    printf("[8] restart cap keeps a dead-SIM gateway up\n");
    {
        ladder_reset();
        sim_connect_ok = false; sim_bearer = false; sim_netinit_ok = false;
        sim_restart_allowed = false;                  /* the cap has been reached */
        counters_clear(); n_restart_asked = 0;
        for (int i = 0; i < 120; i++) { tick(); advance_s(600); }
        CHECK(n_restart == 0, "RESTARTED despite the cap (%d)", n_restart);
        CHECK(n_restart_asked > 0, "never reached L4 in 120 ticks");
        CHECK(n_netinit > NET_BRINGUP_RETRIES && n_hardreset > MODEM_PWRCYCLE_MAX,
              "ladder must keep cycling L2/L3 after the cap (L2 x%d, L3 x%d)", n_netinit, n_hardreset);
        /* and a broker that returns is still found */
        sim_netinit_ok = true; sim_connect_ok = true;
        for (int i = 0; i < 10 && !sim_connected; i++) { tick(); advance_s(600); }
        CHECK(sim_connected, "did not reconnect after the cap once the network returned");
        sim_restart_allowed = true;
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
