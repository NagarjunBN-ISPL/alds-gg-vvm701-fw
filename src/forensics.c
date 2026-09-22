#include "forensics.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "forensics";

/* ── RTC-retained state ────────────────────────────────────────────────────
 * RTC_NOINIT_ATTR means the startup code does NOT clear this, so it carries
 * across esp_restart(), a panic and every watchdog reset. It is undefined
 * after true power loss, which is exactly what the magic guards: a cold boot
 * reads whatever the RAM happened to hold, fails the magic check, and is
 * correctly treated as "no history" rather than as a garbage death record. */
#define FORENSICS_MAGIC 0x5F0DE4D1u

typedef struct {
    uint8_t  phase;    /* FPhase_t                                           */
    uint8_t  detail;   /* phase-specific parameter                           */
    uint16_t seq;      /* marks made by this task — proves it was moving     */
    uint32_t at_ms;    /* esp_timer ms at the mark                           */
} FSlot_t;

typedef struct {
    uint32_t magic;
    uint32_t boot_count;          /* filled by the diagnostics layer          */
    FSlot_t  slot[FSLOT_COUNT];
} FState_t;

RTC_NOINIT_ATTR static FState_t s_live;   /* this boot — written continuously */

/* Snapshot of what the PREVIOUS boot left, taken once in forensics_init()
 * before s_live is re-armed. Ordinary RAM: it only has to survive until the
 * first diagnostics publish. */
static FState_t s_prev;
static bool     s_have;            /* previous boot left a readable state     */
static bool     s_fault;           /* ...and it ended abnormally              */
static int      s_reset;           /* esp_reset_reason() for THIS boot        */

static const char *const f_slot_name[FSLOT_COUNT] = {
    "main", "du_task", "di_task", "relay",
};

static const char *const f_phase_name[F_PHASE_COUNT] = {
    "none",
    "boot_nvs", "boot_modem", "boot_network", "boot_mqtt", "boot_store",
    "idle",
    "bluesky_read", "yenen_read", "radar_read", "genset_read", "di_read",
    "gps_read",
    "publish",
    "store_enqueue", "store_flush", "store_compact",
    "setrsp_write", "passthrough",
    "ms51_txn",
    "fota_download", "fota_write",
    "recover",
    "diagnostics",
};

static const char *phase_name(uint8_t p) {
    return (p < F_PHASE_COUNT) ? f_phase_name[p] : "unknown";
}

/* Which resets mean something went wrong. ESP_RST_SW is deliberately NOT here:
 * it covers {"cmd":"restart"} and every internal safety restart, which are
 * deliberate. Those are still REPORTED (see the header) — they just are not
 * flagged as faults, so the backend can rank them below a real hang. */
static bool reset_is_abnormal(int r) {
    switch (r) {
        case ESP_RST_PANIC:      /* crash, or task WDT with PANIC enabled     */
        case ESP_RST_INT_WDT:    /* interrupts disabled / ISR stuck           */
        case ESP_RST_TASK_WDT:   /* a task stopped feeding                    */
        case ESP_RST_WDT:        /* any other watchdog                        */
        case ESP_RST_BROWNOUT:   /* supply dipped — usually a hardware fault  */
            return true;
        default:
            return false;
    }
}

/* The task that stopped progressing is the one whose last mark is furthest
 * behind the newest mark any task made.
 *
 * Only slots that were EVER marked (seq > 0) are candidates. A task that never
 * ran is absent, not stalled — di_task legitimately never starts when diEnable
 * is 0, and relay_task's slot is empty until the first panel command, so
 * treating an untouched slot as the stall would blame the wrong task on every
 * single report. Falls back to FSLOT_MAIN, which always marks (app_main stamps
 * it before anything else can run), so there is always a valid answer.
 *
 * Shared by the boot log and the JSON so the two can never disagree. */
static int worst_slot(void) {
    int worst = -1;
    for (int i = 0; i < FSLOT_COUNT; i++) {
        if (s_prev.slot[i].seq == 0) continue;
        if (worst < 0 || s_prev.slot[i].at_ms < s_prev.slot[worst].at_ms) worst = i;
    }
    return (worst < 0) ? FSLOT_MAIN : worst;
}

void forensics_init(void) {
    s_reset = (int)esp_reset_reason();
    s_have  = (s_live.magic == FORENSICS_MAGIC);

    if (s_have) {
        memcpy(&s_prev, &s_live, sizeof(s_prev));
        s_fault = reset_is_abnormal(s_reset);
    } else {
        memset(&s_prev, 0, sizeof(s_prev));
        s_fault = false;
    }

    /* Re-arm for this boot. Done unconditionally so a corrupt or cold state is
     * replaced by a valid one immediately. */
    memset(&s_live, 0, sizeof(s_live));
    s_live.magic = FORENSICS_MAGIC;

    if (s_have) {
        int w = worst_slot();
        /* Log it locally too — if the modem never comes up, the serial console
         * is the only place this will ever be readable. */
        esp_log_level_t lvl = s_fault ? ESP_LOG_ERROR : ESP_LOG_INFO;
        ESP_LOG_LEVEL(lvl, TAG,
                 "PREVIOUS BOOT STOPPED: reset=%d fault=%d, last task=%s "
                 "phase=%s detail=%u (last mark %lu ms after boot, %u marks)",
                 s_reset, (int)s_fault, f_slot_name[w],
                 phase_name(s_prev.slot[w].phase), s_prev.slot[w].detail,
                 (unsigned long)s_prev.slot[w].at_ms, s_prev.slot[w].seq);
    } else {
        ESP_LOGI(TAG, "no retained state — cold boot (power-on or first flash)");
    }
}

void forensics_mark(FSlotId_t slot, FPhase_t phase, uint8_t detail) {
    if ((unsigned)slot >= FSLOT_COUNT) return;
    FSlot_t *s = &s_live.slot[slot];
    s->phase  = (uint8_t)phase;
    s->detail = detail;
    s->seq++;
    s->at_ms  = (uint32_t)(esp_timer_get_time() / 1000);
}

bool forensics_have_history(void)   { return s_have; }
bool forensics_last_was_fault(void) { return s_have && s_fault; }

int forensics_stop_json(char *buf, size_t size) {
    if (!s_have || !buf || size == 0) return 0;

    int worst = worst_slot();

    uint32_t newest = 0;
    for (int i = 0; i < FSLOT_COUNT; i++)
        if (s_prev.slot[i].at_ms > newest) newest = s_prev.slot[i].at_ms;

    const FSlot_t *w = &s_prev.slot[worst];

    int n = snprintf(buf, size,
        "{\"fault\":%s,\"reset_reason\":%d,\"task\":\"%s\",\"phase\":\"%s\","
        "\"detail\":%u,\"stalled_ms\":%lu,\"marks\":%u,\"ran_ms\":%lu}",
        s_fault ? "true" : "false",
        s_reset, f_slot_name[worst], phase_name(w->phase),
        (unsigned)w->detail,
        (unsigned long)(newest - w->at_ms),
        (unsigned)w->seq,
        (unsigned long)newest);

    if (n < 0 || n >= (int)size) return 0;   /* caller omits the key entirely */
    return n;
}

void forensics_stop_clear(void) { s_have = false; }
