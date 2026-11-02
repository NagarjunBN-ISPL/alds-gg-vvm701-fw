#include "config.h"
#include "modem.h"
#include "network.h"
#include "fota.h"
#include "mqtt.h"
#include "nvs_config.h"
#include "dispenser.h"
#include "yenen.h"
#include "rs485_bus.h"
#include "rsp_tunnel.h"
#include "du_task.h"
#include "store.h"
#include "radar.h"
#include "genset.h"
#include "di.h"
#include "relay.h"
#include "gps.h"
#include "tls_ca.h"
#include "diagnostics.h"
#include "forensics.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_attr.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "main";

/* ── Global state ─────────────────────────────────────────────────────── */
static char        g_imei[20];
/* Not static: du_task.c reads the live config and the dispenser HW_IDs. Both
 * are fixed after boot — a setconfig always reboots — so there is no
 * concurrent writer and no lock is needed on them. */
DeviceConfig g_cfg;

/* Each dispenser protocol and each radar type gets an independent label space
 * even though they all share the RS485 bus. */
char g_hw_du[DU_MAX_DISPENSERS][24];     /* non-static: read by du_task.c */
static char g_hw_ztech[RADAR_MAX][24];
static char g_hw_pantech[RADAR_MAX][24];
static char g_hw_genset[24], g_hw_di[24];

#define TOPIC_MAX 200
static char t_data[TOPIC_MAX];   /* <base>/data — polled device readings only (build_topics) */
static char t_logs[TOPIC_MAX];   /* <base>/logs — everything else (build_topics)            */
static char t_update[TOPIC_MAX];
static char t_fleet_update[TOPIC_MAX];
static char t_setcfg[TOPIC_MAX];
/* Panel-box lock command topic. Per-IMEI like setconfig/update — a lock
 * command must reach exactly one gateway, never the fleet. The ack and every
 * lock event ride t_lock_out (<base>/<imei>/lock/out), never <base>/data and
 * never back onto the command topic (its "unlocked" text would re-trigger the
 * parser) — the cng-cpil lock contract. */
static char t_lock[TOPIC_MAX];
static char t_lock_out[TOPIC_MAX];
/* The cloud's raw command tunnel, per dispenser (rsp_tunnel.h): the cloud
 * frames the command, the gateway puts it on the bus and answers .../out with
 * the pump's reply as received, or "404". rsp for every make; kfactor for
 * BlueSky only — Yenen has no k-factor, so it gets no kfactor topic. "" = not
 * served (an OFF slot, or kfactor on a Yenen slot): never subscribed. */
static char t_rsp_in[DU_MAX_DISPENSERS][TOPIC_MAX], t_rsp_out[DU_MAX_DISPENSERS][TOPIC_MAX];
static char t_kf_in[DU_MAX_DISPENSERS][TOPIC_MAX],  t_kf_out[DU_MAX_DISPENSERS][TOPIC_MAX];

static int g_diag_cycle = 0;

/* ── LED ──────────────────────────────────────────────────────────────── */
static void led_on(void)  { gpio_set_level(LED_PIN, 1); }
static void led_off(void) { gpio_set_level(LED_PIN, 0); }

/* Blocking status blink. Used only at milestone events (MQTT connect, boot
 * complete, FOTA start) as fast bursts, plus one short pulse per cloud push
 * (led_activity_pulse, wired into mqtt_publish). No idle heartbeat: the LED
 * is quiet unless something is actually happening. */
static void led_blink(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        led_off();
        if (i < count - 1) vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

/* ── Solid-on resting state (unconfigured) ────────────────────────────────
 * The LED's resting level is normally OFF, so the per-publish pulse reads as a
 * flash against a dark LED. While the gateway is UNCONFIGURED — announcing
 * status "unconfigured" and waiting for a setconfig — the resting level is ON
 * instead, giving an installer a state they can read across the room without
 * a laptop: fast blink = booting, steady on = up but not provisioned, dark
 * with a flash per push = provisioned and reporting.
 *
 * Everything that leaves the LED at rest has to consult this, which is why the
 * pulse and the FOTA/boot blink stop below both route through it rather than
 * calling led_off() unconditionally. */
static bool s_led_solid = false;

static void led_solid_set(bool on) {
    if (on == s_led_solid) return;      /* idempotent — called every poll cycle */
    s_led_solid = on;
    if (on) led_on(); else led_off();
}

/* One short flash — registered with mqtt_set_activity_cb() so it fires on
 * every publish. This is the steady-state indicator: a blink = data pushed.
 * When the LED is being held on (unconfigured), the flash is INVERTED — a
 * brief gap rather than a brief light — so a push is still visible without
 * destroying the solid-on state the gap sits in. */
static void led_activity_pulse(void) {
    if (s_led_solid) {
        led_off();
        vTaskDelay(pdMS_TO_TICKS(40));
        led_on();
    } else {
        led_blink(1, 40, 0);
    }
}

/* ── Continuous background blink (boot + FOTA) ────────────────────────────
 * A periodic esp_timer toggles the LED on its own task, so the LED keeps
 * blinking fast even while the main task is blocked inside a long modem /
 * network / FOTA-download call. Started at power-on and during FOTA; stopped
 * once the device is connected / FOTA is done, after which the per-publish
 * pulse takes over. These two never overlap: the timer is always stopped
 * before any publish that would fire led_activity_pulse. */
static esp_timer_handle_t s_blink_timer = NULL;

static void blink_timer_cb(void *arg) {
    (void)arg;
    static int s = 0;
    s = !s;
    gpio_set_level(LED_PIN, s);
}

static void led_blink_start(void) {
    if (!s_blink_timer) {
        const esp_timer_create_args_t args = { .callback = blink_timer_cb,
                                                .name     = "led_blink" };
        if (esp_timer_create(&args, &s_blink_timer) != ESP_OK) return;
    }
    esp_timer_stop(s_blink_timer);                 /* no-op if not running   */
    esp_timer_start_periodic(s_blink_timer, 60000); /* toggle every 60 ms ~8 Hz */
}

static void led_blink_stop(void) {
    if (s_blink_timer) esp_timer_stop(s_blink_timer);
    /* Back to the resting level rather than blindly off: an UNCONFIGURED
     * gateway that takes a FOTA would otherwise come out of the download with
     * a dark LED and no way back to solid until the next state change. */
    if (s_led_solid) led_on(); else led_off();
}

/* ── RS485 helpers ──────────────────────────────────────────────────────
 * The UART setup and the line-mode switches moved to rs485_bus.c when
 * dispenser polling moved to du_task: both tasks now drive UART1 and both
 * need the same mode helpers, and every transaction has to be serialised
 * through the bus lock. Behaviour is unchanged — the implementations were
 * moved verbatim. Radar and genset below are the main task's remaining users
 * of the bus. */

/* ── Build identifiers ────────────────────────────────────────────────── */
static void build_hw_ids(void) {
    /* HW_ID suffixes are all two DIGITS and every one of them is also the
     * unit's address on its bus (an explicit sfx* from setconfig overrides the
     * auto value, but the equivalence holds either way):
     *   dispensers   = the dispenser NUMBER (du_number_of: the slot's suffix;
     *                  an OFF slot has none) — BlueSky nozzles 2N-1/2N, Yenen head N
     *   Ztech tanks  = Modbus slave, auto ZTECH_SLAVE_BASE + tank index (04..)
     *   Pantech      = Modbus slave, auto continues after the last Ztech tank
     *   genset       = gensetSlave (default 10)
     *   DI           11 (no bus address)
     * The dispenser and radar ranges DO overlap once a site has 4+ dispensers
     * and a radar (DU4 = 04 = Ztech tank 1): validate_suffixes() computes these
     * same values and rejects the plan, and the operator assigns explicit
     * suffixes — a loud refusal beats two units silently publishing under one
     * HW_ID. validate_suffixes() MUST stay in sync with this function. */
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
        int n = du_number_of(&g_cfg, i);
        if (n < 1 || n > 99) n = i + 1;         /* slot OFF: never published or subscribed; belt */
        snprintf(g_hw_du[i], sizeof(g_hw_du[i]), "%s%c%c", g_imei,
                 (char)('0' + n / 10), (char)('0' + n % 10));
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        if (g_cfg.sfx_ztech[i][0])
            snprintf(g_hw_ztech[i], sizeof(g_hw_ztech[i]), "%s%s", g_imei, g_cfg.sfx_ztech[i]);
        else
            snprintf(g_hw_ztech[i], sizeof(g_hw_ztech[i]), "%s%02d", g_imei, 4 + i);
        if (g_cfg.sfx_pantech[i][0])
            snprintf(g_hw_pantech[i], sizeof(g_hw_pantech[i]), "%s%s", g_imei, g_cfg.sfx_pantech[i]);
        else
            snprintf(g_hw_pantech[i], sizeof(g_hw_pantech[i]), "%s%02d", g_imei, 4 + g_cfg.ztech_count + i);
    }
    if (g_cfg.sfx_genset[0])
        snprintf(g_hw_genset, sizeof(g_hw_genset), "%s%s", g_imei, g_cfg.sfx_genset);
    else if (g_cfg.genset_slave <= 99)
        snprintf(g_hw_genset, sizeof(g_hw_genset), "%s%02d", g_imei, g_cfg.genset_slave);   /* = slave, default 10 */
    else
        snprintf(g_hw_genset, sizeof(g_hw_genset), "%s10", g_imei);                         /* slave >99 can't be a 2-char suffix */
    if (g_cfg.sfx_di[0])
        snprintf(g_hw_di, sizeof(g_hw_di), "%s%s", g_imei, g_cfg.sfx_di);
    else
        snprintf(g_hw_di, sizeof(g_hw_di), "%s11", g_imei);                                 /* 11 (not on the Modbus bus) */
}

static void build_topics(void) {
    /* Two flat, IMEI-less outbound topics. Every payload carries "HW_ID" (bare
     * IMEI, or IMEI+suffix for per-equipment telemetry) so the cloud attributes
     * it to a device by that field.
     *   <base>/data (e.g. "gogas/data") — the readings this gateway POLLS and
     *     nothing else: dispenser nozzles (live and the flash backlog), radar
     *     tanks, the DG's register/Alarms/status frames, DI changes and GPS
     *     fixes. The record stream the cloud stores.
     *   <base>/logs (e.g. "gogas/logs") — everything else: the diagnostics
     *     report, every lifecycle "status" (ready/unconfigured, restarting,
     *     erasing_config, factory_resetting, connectivity_rollback, firmware_credentials_
     *     applied), every setconfig reply (config_received/_successful/
     *     _failed, config_report) and every FOTA status. Split 2026-10-16
     *     (btpl's layout) so /data carries device readings only. The broker
     *     ACL must allow publishing to both.
     * Per-IMEI reply topics are not touched by the split: lock events stay on
     * <base>/<imei>/lock/out, and the rsp/kfactor tunnel replies on their own
     * .../out topics — each is the answer a waiting command listens on.
     * Inbound command topics (setconfig/update/rsp/kfactor below) stay
     * per-IMEI — they must target one device, and flattening them would make
     * every device act on every push. */
    snprintf(t_data,        TOPIC_MAX, "%s/data",          g_cfg.mqtt_base_topic);
    snprintf(t_logs,        TOPIC_MAX, "%s/logs",          g_cfg.mqtt_base_topic);
    snprintf(t_update,      TOPIC_MAX, "%s/%s/update",     g_cfg.mqtt_base_topic, g_imei);
    /* Fleet FOTA carries the PRODUCT NAME: GG and GGY share one base topic in
     * production, and this is what keeps a GG fleet push off the GGY units. */
    snprintf(t_fleet_update,TOPIC_MAX, "%s/fleet/gg/update",  g_cfg.mqtt_base_topic);
    snprintf(t_setcfg,      TOPIC_MAX, "%s/%s/setconfig",  g_cfg.mqtt_base_topic, g_imei);
    snprintf(t_lock,        TOPIC_MAX, "%s/%s/lock",       g_cfg.mqtt_base_topic, g_imei);
    snprintf(t_lock_out,    TOPIC_MAX, "%s/%s/lock/out",   g_cfg.mqtt_base_topic, g_imei);
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
        t_rsp_in[i][0] = t_rsp_out[i][0] = '\0';
        t_kf_in[i][0]  = t_kf_out[i][0]  = '\0';
        if (!du_slot_on(&g_cfg, i)) continue;   /* an OFF slot gets no command topics */
        const bool bluesky = (g_cfg.du_make[i] == DU_MAKE_BLUESKY);
        if (!bluesky && g_cfg.du_make[i] != DU_MAKE_YENEN) continue;
        snprintf(t_rsp_in[i],  TOPIC_MAX, "%s/rsp/imei/%s/in",  g_cfg.mqtt_base_topic, g_hw_du[i]);
        snprintf(t_rsp_out[i], TOPIC_MAX, "%s/rsp/imei/%s/out", g_cfg.mqtt_base_topic, g_hw_du[i]);
        if (bluesky) {                          /* Yenen has no k-factor */
            snprintf(t_kf_in[i],  TOPIC_MAX, "%s/kfactor/imei/%s/in",  g_cfg.mqtt_base_topic, g_hw_du[i]);
            snprintf(t_kf_out[i], TOPIC_MAX, "%s/kfactor/imei/%s/out", g_cfg.mqtt_base_topic, g_hw_du[i]);
        }
    }
}

/* Forward decls — defined later in the file. */
static void mqtt_reconnect_if_needed(void);
static void poll_incoming_mqtt(void);
static bool ota_is_pending_verify(void);
static void ota_confirm_before_restart(const char *why);
static void fota_owed_answers_drop(void);
static void fota_failed_owe(const char *json);
static void publish_fota_failed_owed(void);

/* ── Fruitless-restart cap ─────────────────────────────────────────────────
 * Every self-restart this firmware performs for a NETWORK reason — the boot
 * path's "network never came up" / "broker unreachable and network down", and
 * the recovery ladder's L4 — is a bet that a fresh modem bring-up will help.
 * With a dead SIM, no coverage or a decommissioned APN it never does, and the
 * gateway would spend its life in a boot→3×registration-wait→reboot loop:
 * ~10 min per lap, forever, each lap tearing down the acquisition task that
 * is otherwise happily reading pumps and buffering to flash.
 *
 * So the streak is counted across restarts (RTC memory survives esp_restart,
 * not a power cut — a power cut is a genuine fresh start) and once it reaches
 * RESTART_STREAK_CAP the firmware STOPS restarting for that reason and stays
 * up: the boot path falls into the poll loop offline, and the ladder keeps
 * cycling L1–L3 (which already includes a modem PWRKEY power-cycle — the
 * only thing a restart could have added). Any broker session zeroes the
 * streak, so it is only ever reached by a fault a restart cannot fix. Ported
 * from cng-cpil (s_l4_streak), extended to the boot path here because that
 * is where a dead-SIM gateway actually loops. */
#define RESTART_STREAK_CAP    6
#define RESTART_STREAK_MAGIC  0x52535443u   /* "RSTC" */
RTC_NOINIT_ATTR static uint32_t s_restart_streak_magic;
RTC_NOINIT_ATTR static uint32_t s_restart_streak;

static void restart_streak_init(void) {
    if (s_restart_streak_magic != RESTART_STREAK_MAGIC) {   /* power-on / garbage */
        s_restart_streak_magic = RESTART_STREAK_MAGIC;
        s_restart_streak       = 0;
    }
    if (s_restart_streak)
        ESP_LOGW(TAG, "%lu consecutive network-recovery restart(s) so far without a "
                      "broker session (cap %d)", (unsigned long)s_restart_streak, RESTART_STREAK_CAP);
}

/* True = go ahead and restart (and the streak is counted). False = the cap is
 * reached; the caller must stay up instead. */
static bool restart_streak_allow(const char *why) {
    if (s_restart_streak >= RESTART_STREAK_CAP) {
        ESP_LOGE(TAG, "%s — but %d consecutive restarts have not produced a broker "
                      "session, so NOT restarting again: staying up, buffering to "
                      "flash, the recovery ladder keeps trying", why, RESTART_STREAK_CAP);
        return false;
    }
    s_restart_streak++;
    return true;
}

static void restart_streak_clear(void) { s_restart_streak = 0; }


/* When the current subscriptions were made — the anchor for the retain-grace
 * window (CMD_RETAIN_GRACE_MS, config.h). A broker delivers retained messages
 * right after SUBSCRIBE, so "arrived within a few seconds of subscribing" is
 * the signature of a retained command. Set here, read by cmd_in_retain_grace(). */
static int64_t s_subscribed_at_us = 0;

static bool cmd_in_retain_grace(const char *what) {
    if (s_subscribed_at_us == 0) return false;
    int64_t age_ms = (esp_timer_get_time() - s_subscribed_at_us) / 1000;
    if (age_ms >= CMD_RETAIN_GRACE_MS) return false;
    ESP_LOGW(TAG, "%s arrived %lld ms after subscribing — treated as a RETAINED "
                  "message and ignored (publish commands with retain=false)",
             what, (long long)age_ms);
    return true;
}

static void subscribe_all(void) {
    s_subscribed_at_us = esp_timer_get_time();
    /* A LIVE broker session is what proves the network is usable; this is
     * also called on the boot path when the connect failed (the subscribes
     * then simply error), and that must not count as success. */
    if (mqtt_is_connected()) restart_streak_clear();
    mqtt_subscribe(t_update);
    mqtt_subscribe(t_fleet_update);
    mqtt_subscribe(t_setcfg);
    mqtt_subscribe(t_lock);
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {   /* "" = not served (build_topics) */
        if (t_rsp_in[i][0]) mqtt_subscribe(t_rsp_in[i]);
        if (t_kf_in[i][0])  mqtt_subscribe(t_kf_in[i]);
    }
}

/* ── Tiny JSON helpers ────────────────────────────────────────────────── */
static bool json_get_str(const char *json, const char *key,
                          char *out, size_t out_size) {
    if (out_size == 0) return false;
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    if (*p != '"') return false;
    p++;
    /* Walk to the CLOSING quote honoring backslash escapes, decoding them as we
     * copy — a plain strchr(p,'"') stops at the first quote even when it is an
     * escaped \" inside the value, truncating (and leaving a stray backslash in)
     * a credential that legitimately contains a quote or backslash. This is now
     * the only way string config arrives (setconfig), so a broker/APN password
     * with a " or \\ must survive intact. Values without a backslash take the
     * identical path they always did, so normal configs are unaffected. */
    size_t n = 0;
    while (*p && *p != '"') {
        char c;
        if (*p == '\\' && p[1]) {
            p++;
            switch (*p) {
                case 'n':  c = '\n'; break;
                case 't':  c = '\t'; break;
                case 'r':  c = '\r'; break;
                case 'b':  c = '\b'; break;
                case 'f':  c = '\f'; break;
                case '"':  c = '"';  break;
                case '\\': c = '\\'; break;
                case '/':  c = '/';  break;
                case 'u': {
                    /* \uXXXX, which Go, .NET and HTML-safe JSON encoders write
                     * for '&', '<', '>', '=' and '+' — "\u0026" between the
                     * parameters of a presigned link. Decoded only when it is
                     * printable ASCII; anything else (a NUL, a control byte, a
                     * non-ASCII character) keeps the old result, the 'u'
                     * literally. Each hex digit is read only after the one
                     * before it was one, so this never reads past the string. */
                    unsigned v = 0;
                    int k = 1;
                    for (; k <= 4; k++) {
                        const char h = p[k];
                        unsigned d;
                        if      (h >= '0' && h <= '9') d = (unsigned)(h - '0');
                        else if (h >= 'a' && h <= 'f') d = (unsigned)(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F') d = (unsigned)(h - 'A' + 10);
                        else break;
                        v = v * 16u + d;
                    }
                    if (k == 5 && v >= 0x20 && v <= 0x7E) {
                        c = (char)v;
                        p += 4;                   /* the p++ below steps past the last digit */
                    } else {
                        c = 'u';
                    }
                    break;
                }
                default:   c = *p;   break;   /* unknown escape: keep the char literal */
            }
        } else {
            c = *p;
        }
        if (n < out_size - 1) out[n++] = c;   /* clamp like before; keep scanning for the close */
        p++;
    }
    if (*p != '"') return false;              /* string never closed — treat as absent */
    out[n] = '\0';
    return true;
}

static bool json_has_key(const char *json, const char *key) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    return strstr(json, search) != NULL;
}

/* ── setconfig field readers (pure: json in, staged config out) ──────────
 * Each refuses a value that would otherwise be quietly turned into a
 * different, valid setting — the device would then answer config_successful
 * while polling an address, or running a switch, that nobody chose. */

/* A suffix key into its char[3] field. json_get_str() CLAMPS a longer value,
 * so "045" would arrive as "04" (another valid address) and a number sent
 * without quotes would read as absent: both are refused here, naming the key.
 * The format itself (digits, 01..99) is judged by validate_suffixes() on the
 * merged plan. Key absent: the field is left untouched. */
static bool setcfg_get_sfx(const char *json, const char *key, char *out, size_t out_size,
                           char *err, size_t err_size) {
    if (!json_has_key(json, key)) return true;
    char wide[16];
    if (!json_get_str(json, key, wide, sizeof(wide)) || strlen(wide) > out_size - 1) {
        snprintf(err, err_size, "%s must be 2 digits as a string, e.g. 05", key);
        return false;
    }
    memcpy(out, wide, strlen(wide) + 1);
    return true;
}

/* The text right after "key": (and any whitespace) — the start of its value —
 * or NULL when the key is absent. */
static const char *setcfg_value(const char *json, const char *key) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return NULL;
    p += strlen(search);
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return p;
}

/* True when nothing but whitespace stands between p and the end of the value
 * (the ',' before the next member, or the object's closing '}'). */
static bool setcfg_value_ends(const char *p) {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return *p == ',' || *p == '}';
}

/* Consumes `word` (lower case) at *pp, ignoring case. */
static bool setcfg_word(const char **pp, const char *word) {
    const char *p = *pp;
    for (; *word; word++, p++) {
        char c = (*p >= 'A' && *p <= 'Z') ? (char)(*p + 32) : *p;
        if (c != *word) return false;
    }
    *pp = p;
    return true;
}

/* An int key. The value must be a whole number — bare or quoted ("2" is 2) —
 * and inside lo..hi, or the push is REFUSED with the key named. Nothing is
 * clamped, defaulted or ignored any more: "duCount":9 used to become 6,
 * "duCount":-1 became 0 (every dispenser gone), "duCount":"two" and
 * "gensetModel":12 were quietly replaced, and each was acked
 * config_successful — on a gateway that only setconfig can reach. Key
 * absent: *io is left as is. */
static bool setcfg_int_in_range(const char *json, const char *key, int lo, int hi,
                                int *io, char *err, size_t err_size) {
    const char *p = setcfg_value(json, key);
    if (!p) return true;
    bool quoted = (*p == '"');
    if (quoted) p++;
    bool neg = (*p == '-');
    if (neg) p++;
    /* At most 9 digits are read (fits a 32-bit long with no overflow); a 10th
     * is left in place and fails the end-of-value check below. */
    long v      = 0;
    int  digits = 0;
    while (*p >= '0' && *p <= '9' && digits < 9) {
        v = v * 10 + (*p - '0');
        digits++;
        p++;
    }
    bool shape_ok = digits >= 1;
    if (shape_ok && quoted) {
        if (*p == '"') p++;
        else           shape_ok = false;
    }
    if (!shape_ok || !setcfg_value_ends(p)) {
        snprintf(err, err_size, "%s must be a whole number %d..%d", key, lo, hi);
        return false;
    }
    if (neg) v = -v;
    if (v < lo || v > hi) {
        snprintf(err, err_size, "%s must be %d..%d", key, lo, hi);
        return false;
    }
    *io = (int)v;
    return true;
}

/* A switch key: true/false or 1/0, bare or quoted, any case. Anything else is
 * REFUSED — "gpsEnable":"true" used to be ignored (only the bare form was
 * read) and "yes" too, both acked config_successful. Key absent: *io is left
 * as is. */
static bool setcfg_get_bool(const char *json, const char *key, bool *io,
                            char *err, size_t err_size) {
    const char *p = setcfg_value(json, key);
    if (!p) return true;
    bool quoted = (*p == '"');
    if (quoted) p++;
    bool v, ok = true;
    if      (setcfg_word(&p, "true"))  v = true;
    else if (setcfg_word(&p, "false")) v = false;
    else if (*p == '1')                { v = true;  p++; }
    else if (*p == '0')                { v = false; p++; }
    else                               { v = false; ok = false; }
    if (ok && quoted) {
        if (*p == '"') p++;
        else           ok = false;
    }
    if (!ok || !setcfg_value_ends(p)) {
        snprintf(err, err_size, "%s must be true or false", key);
        return false;
    }
    *io = v;
    return true;
}

/* A string key into a fixed field. Present but not a string, or longer than
 * the field, is REFUSED: json_get_str() would read the first as absent and
 * silently CUT the second — a broker password or APN one character short is a
 * gateway that never reconnects. *present says whether the key was sent. */
static bool setcfg_get_text(const char *json, const char *key, char *out, size_t out_size,
                            bool *present, char *err, size_t err_size) {
    *present = false;
    if (!json_has_key(json, key)) return true;
    char wide[160];
    if (!json_get_str(json, key, wide, sizeof(wide))) {
        snprintf(err, err_size, "%s must be a string", key);
        return false;
    }
    if (strlen(wide) > out_size - 1) {
        snprintf(err, err_size, "%s is longer than %u characters", key, (unsigned)(out_size - 1));
        return false;
    }
    memcpy(out, wide, strlen(wide) + 1);
    *present = true;
    return true;
}

/* sfxGenset and gensetSlave are ONE value — the genset's suffix is its Modbus
 * address — so they are synced by what THIS push carries, never by what the
 * staged copy inherited from NVS. The old sync tested the staged suffix: once
 * the first genset setconfig had stored "10", a later {"gensetSlave":4} was
 * overwritten back to 10 and acked config_successful, and the DG sat on the
 * wrong address reading 404.
 *   sfxGenset "NN" -> slave NN; a gensetSlave in the same push must agree
 *   gensetSlave N  -> suffix "%02u" (none above 99: the HW_ID then uses "10")
 *   sfxGenset ""   -> back to the slave's own number
 *   neither        -> the stored pair, untouched
 * A slave outside 1..247 is refused: an address is never guessed. */
static bool setcfg_genset_addr(const char *json, DeviceConfig *nc, char *err, size_t err_size) {
    bool slave_present = json_has_key(json, "gensetSlave");
    int  slave         = nc->genset_slave;
    if (!setcfg_int_in_range(json, "gensetSlave", 1, 247, &slave, err, err_size)) return false;
    bool sfx_present = json_has_key(json, "sfxGenset");
    char sfx[sizeof(nc->sfx_genset)] = "";
    if (!setcfg_get_sfx(json, "sfxGenset", sfx, sizeof(sfx), err, err_size)) return false;

    if (sfx_present && sfx[0]) {
        memcpy(nc->sfx_genset, sfx, sizeof(sfx));
        if (isdigit((unsigned char)sfx[0]) && isdigit((unsigned char)sfx[1])) {
            int gs = atoi(sfx);
            if (slave_present && slave != gs) {
                snprintf(err, err_size, "sfxGenset %s and gensetSlave %d disagree, send one of them",
                         sfx, slave);
                return false;
            }
            if (gs > 0) nc->genset_slave = (uint8_t)gs;   /* "00": validate_suffixes() refuses */
        }
        return true;   /* not 2 digits: validate_suffixes() refuses it with its own reason */
    }
    if (slave_present || sfx_present) {
        nc->genset_slave = (uint8_t)slave;
        if (nc->genset_slave <= 99)
            snprintf(nc->sfx_genset, sizeof(nc->sfx_genset), "%02u", (unsigned)(nc->genset_slave % 100));
        else
            nc->sfx_genset[0] = '\0';
    }
    return true;
}

/* True if a setconfig payload carries any connectivity credential (broker or
 * APN user/pass). Used ONLY to suppress the plaintext payload log below so a
 * pushed secret never reaches the serial console — the payload handed to
 * parse_setconfig() is untouched, so parsing behaviour is identical. */
static bool setcfg_has_secret(const char *json) {
    return json_has_key(json, "mqttUser") || json_has_key(json, "mqttPass") ||
           json_has_key(json, "apnUser")  || json_has_key(json, "apnPass");
}

/* Parse a JSON boolean. Accepts true/false and 1/0 — an atoi()-style reader
 * can't be used here because atoi("true") is 0, i.e. it can't tell true from
 * false — bare or quoted: some cloud libraries write "true" as a string, and
 * the only readers (FOTA's "force" and "handover") would otherwise take a
 * deliberate swap for a refusal. */
static bool json_get_bool(const char *json, const char *key, bool def) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return def;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p == '"') p++;
    if (*p == 't' || *p == 'T' || *p == '1') return true;
    if (*p == 'f' || *p == 'F' || *p == '0') return false;
    return def;
}

/* ── setconfig ────────────────────────────────────────────────────────── */
/* This is only the coarse "is this payload a recognized setconfig at all"
 * gate — it requires at least ONE key below to be present, then the actual
 * per-field parsing loops (du%dMake/du%dSfx/sfxZtech%d/... further down)
 * run over DU_MAX_DISPENSERS/RADAR_MAX dynamically regardless of this list. A
 * key missing here only matters for a payload that contains *nothing else*
 * recognized (e.g. a bare {"du4Sfx":"04"} or {"sfxDI":"12"}), which would
 * otherwise be rejected as bad_payload — so every key is listed explicitly to
 * avoid that trap. Keep this in sync with DU_MAX_DISPENSERS/RADAR_MAX in
 * config.h when either cap changes. */
static const char * const s_setcfg_known_keys[] = {
    "duCount",
    "ztechCount","pantechCount","radarType","tankCount",
    "gensetType","gensetModel","gensetSlave","gensetBaud","diEnable","gpsEnable",
    "sfxGenset","sfxDI",
    "du1Make","du2Make","du3Make","du4Make","du5Make","du6Make",
    "du1Sfx","du2Sfx","du3Sfx","du4Sfx","du5Sfx","du6Sfx",
    "sfxZtech1","sfxZtech2","sfxZtech3","sfxZtech4",
    "sfxPantech1","sfxPantech2","sfxPantech3","sfxPantech4",
    "apn","apnUser","apnPass",
    "mqttHost","mqttPort","mqttUser","mqttPass","mqttBaseTopic","mqttTls",
};
/* Keys the member check accepts that are not config keys and set nothing:
 *  - the read-only keys of a config_report. A report can be copied, edited and
 *    sent back as a setconfig (passwords are reported only as *Set booleans
 *    for exactly that reason), so these are accepted and IGNORED;
 *  - factory_reset's arguments, "confirm" and "force". parse_setconfig()
 *    reads them for that command only; with any other payload it refuses them
 *    as unknown_key_confirm / unknown_key_force, as before they existed. */
static const char * const s_setcfg_extra_keys[] = {
    "HW_ID", "status", "fw", "apnPassSet", "mqttPassSet", "connPending",
    "confirm", "force",
};
#define SETCFG_KEYS_MAX 64
_Static_assert(sizeof(s_setcfg_known_keys) / sizeof(s_setcfg_known_keys[0]) +
               sizeof(s_setcfg_extra_keys) / sizeof(s_setcfg_extra_keys[0]) < SETCFG_KEYS_MAX,
               "setcfg_check_members() tracks at most SETCFG_KEYS_MAX - 1 keys");

/* Walks the top level of a setconfig payload — already proven ONE complete
 * JSON object by payload_is_one_json_object() — and checks every member: a key
 * from `keys` or `extra` (or "cmd"), named only once, holding a plain value (a
 * string, or a bare number / true / false). The typed readers then judge each
 * value; `extra` keys set no configuration (s_setcfg_extra_keys).
 *
 * Refusing an unknown key is the point: the readers look keys up by name, so a
 * typo ("du1make", "gpsenable") used to be skipped without a word and the push
 * acked config_successful with that setting never applied — on a gateway that
 * nobody can visit. A key sent twice is refused because only the first copy
 * would be read. `err` names the key; non-alphanumerics in it become '_'. */
static bool setcfg_check_members(const char *json, const char *const *keys, size_t n_keys,
                                 const char *const *extra, size_t n_extra,
                                 char *err, size_t err_size) {
    bool seen[SETCFG_KEYS_MAX] = {false};   /* keys, then extra, then "cmd" */
    if (n_keys > SETCFG_KEYS_MAX - 1) n_keys = SETCFG_KEYS_MAX - 1;
    if (n_extra > SETCFG_KEYS_MAX - 1 - n_keys) n_extra = SETCFG_KEYS_MAX - 1 - n_keys;
    const char *p = strchr(json, '{');
    if (!p) { snprintf(err, err_size, "bad_payload"); return false; }
    p++;
    for (;;) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p == '}') return true;                    /* end of the object */
        if (*p != '"') { snprintf(err, err_size, "bad_payload"); return false; }
        const char *k = ++p;
        while (*p && *p != '"' && *p != '\\') p++;     /* no key of ours has an escape */
        if (*p != '"') { snprintf(err, err_size, "bad_payload"); return false; }
        size_t klen = (size_t)(p - k);
        p++;

        int idx = -1;
        if (klen == 3 && strncmp(k, "cmd", 3) == 0) idx = (int)(n_keys + n_extra);
        for (size_t i = 0; i < n_keys && idx < 0; i++)
            if (strlen(keys[i]) == klen && strncmp(k, keys[i], klen) == 0) idx = (int)i;
        for (size_t i = 0; i < n_extra && idx < 0; i++)
            if (strlen(extra[i]) == klen && strncmp(k, extra[i], klen) == 0) idx = (int)(n_keys + i);
        if (idx < 0 || seen[idx]) {
            char name[25];
            size_t n = klen < sizeof(name) - 1 ? klen : sizeof(name) - 1;
            for (size_t i = 0; i < n; i++) name[i] = isalnum((unsigned char)k[i]) ? k[i] : '_';
            name[n] = '\0';
            snprintf(err, err_size, "%s_key_%s", idx < 0 ? "unknown" : "duplicate", name);
            return false;
        }
        seen[idx] = true;

        /* The ':' must follow the key directly — every JSON serialiser writes
         * it so, and the readers look keys up as "key": exactly. With a space
         * in between, a key passed here would then read as ABSENT. */
        if (*p != ':') { snprintf(err, err_size, "bad_payload"); return false; }
        p++;
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p == '"') {                                /* a string, escapes honoured */
            p++;
            while (*p && *p != '"') { if (*p == '\\' && p[1]) p++; p++; }
            if (*p != '"') { snprintf(err, err_size, "bad_payload"); return false; }
            p++;
        } else {                                        /* a bare number / true / false */
            const char *v = p;
            while (isalnum((unsigned char)*p) || *p == '-' || *p == '+' || *p == '.') p++;
            if (p == v) {                               /* an object, an array, nothing */
                snprintf(err, err_size, "%.*s must be a plain value", (int)(klen < 24 ? klen : 24), k);
                return false;
            }
        }
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p == ',') { p++; continue; }
        if (*p == '}') return true;
        snprintf(err, err_size, "bad_payload");
        return false;
    }
}

/* Every active piece of equipment's *effective* HW_ID suffix (custom if set,
 * else the numeric auto value from build_hw_ids()) must be unique, or two
 * pieces of equipment publish under the same HW_ID and become
 * indistinguishable downstream. */
static bool validate_suffixes(const DeviceConfig *c, char *err, size_t err_size) {
    /* Static rather than stack-local: this trio is ~770 bytes, and this
     * function sits at the bottom of a deep call chain (app_main -> ... ->
     * parse_setconfig -> validate_suffixes) that already carries several
     * hundred bytes of its own locals per frame. validate_suffixes() is only
     * ever called sequentially from the single main task, never re-entrantly
     * or concurrently, so static is safe here and trades BSS (abundant) for
     * stack headroom (the scarcer resource on this call path). */
    static const char *sfx[32];
    static char        label[32][12];
    static char        auto_sfx[32][8];
    int         n = 0;

    /* Auto values mirror build_hw_ids() EXACTLY — see the table there. Unlike
     * the earlier fixed 21../31../41.. scheme, the auto ranges CAN overlap
     * (dispenser 04 vs Ztech tank 1 at slave 4), which is precisely why every
     * active unit's effective suffix is computed here and checked: the plan is
     * refused with a reason naming both units, and the operator assigns an
     * explicit suffix. It also catches a CUSTOM suffix colliding with another
     * unit's auto value (a dispenser set to "10" while a genset is active).
     *
     * The 32-entry arrays above bound the total: 6 dispensers + 4 Ztech +
     * 4 Pantech + genset + DI = 16 at full capacity. Raising any *_MAX in
     * config.h past that headroom means growing them too. */
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
        /* Slot OFF (no suffix): not polled, publishes nothing, claims no
         * suffix — there is no position fallback any more. */
        if (!du_slot_on(c, i)) continue;
        snprintf(label[n], sizeof(label[n]), "DU%d", i + 1);
        /* A dispenser's suffix IS its number IS its bus address, so it has to
         * be one the bus can reach before anything else is checked: 01..06
         * (a BlueSky nozzle address is 2N, and the bus carries 12), Yenen
         * 01..05 (head 6 has no nozzle code — DU_YENEN_MAX). */
        sfx[n] = c->du_sfx[i];
        if (strlen(sfx[n]) != 2 || !isdigit((unsigned char)sfx[n][0]) ||
            !isdigit((unsigned char)sfx[n][1])) {
            snprintf(err, err_size, "DU%d suffix must be 2 digits (the dispenser number, 01..0%d)",
                     i + 1, DU_MAX_DISPENSERS);
            return false;
        }
        int num   = du_number_of(c, i);
        int max_n = (c->du_make[i] == DU_MAKE_YENEN) ? DU_YENEN_MAX : DU_MAX_DISPENSERS;
        if (num < 1 || num > max_n) {
            snprintf(err, err_size, "DU%d names dispenser %02d, but a %s dispenser must be 01..%02d",
                     i + 1, num, c->du_make[i] == DU_MAKE_YENEN ? "Yenen" : "BlueSky", max_n);
            return false;
        }
        n++;
    }
    for (int i = 0; i < c->ztech_count; i++) {
        snprintf(label[n], sizeof(label[n]), "Ztech%d", i + 1);
        if (c->sfx_ztech[i][0]) sfx[n] = c->sfx_ztech[i];
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "%02d", 4 + i); sfx[n] = auto_sfx[n]; }
        n++;
    }
    for (int i = 0; i < c->pantech_count; i++) {
        snprintf(label[n], sizeof(label[n]), "Pantech%d", i + 1);
        if (c->sfx_pantech[i][0]) sfx[n] = c->sfx_pantech[i];
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "%02d", 4 + c->ztech_count + i); sfx[n] = auto_sfx[n]; }
        n++;
    }
    if (c->genset_type != GENSET_OFF) {
        snprintf(label[n], sizeof(label[n]), "Genset");
        if (c->sfx_genset[0]) sfx[n] = c->sfx_genset;
        else if (c->genset_slave <= 99) { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "%02d", c->genset_slave); sfx[n] = auto_sfx[n]; }
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "10"); sfx[n] = auto_sfx[n]; }
        n++;
    }
    if (c->di_enable) {
        snprintf(label[n], sizeof(label[n]), "DI");
        if (c->sfx_di[0]) sfx[n] = c->sfx_di;
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "11"); sfx[n] = auto_sfx[n]; }
        n++;
    }

    /* Format: a non-empty custom suffix must be exactly 2 DIGITS, 01..99.
     * Digits only, because in this product every suffix doubles as an address
     * (a radar tank's Modbus slave, the genset's slave, a dispenser's number)
     * and a lettered one would keep its HW_ID while polling some fallback
     * address the operator never chose. Length alone isn't enough either: the
     * suffix is embedded verbatim into every JSON payload
     * ("HW_ID":"<imei><sfx>") and into the rsp/kfactor MQTT topic strings — a
     * quote corrupts every telemetry message from that unit into unparseable
     * JSON, and a space/wildcard breaks the topic matching. */
    for (int i = 0; i < n; i++) {
        if (sfx[i][0] && (strlen(sfx[i]) != 2 ||
                          !isdigit((unsigned char)sfx[i][0]) ||
                          !isdigit((unsigned char)sfx[i][1]) ||
                          (sfx[i][0] == '0' && sfx[i][1] == '0'))) {
            snprintf(err, err_size, "%s suffix must be exactly 2 digits (01..99)", label[i]);
            return false;
        }
    }

    /* Defensive: every equipment type now has a numeric auto-fallback (above),
     * so no effective suffix should be empty. This still guards the case of a
     * future equipment type added without a fallback — two empties would both
     * resolve to the bare IMEI and collide with the device-level messages. */
    int empty_count = 0;
    for (int i = 0; i < n; i++) if (!sfx[i][0]) empty_count++;
    if (empty_count > 1) {
        snprintf(err, err_size, "more than one equipment has no suffix assigned (bare IMEI)");
        return false;
    }
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (sfx[i][0] && strcmp(sfx[i], sfx[j]) == 0) {
                snprintf(err, err_size, "%s and %s have duplicate suffix %s",
                         label[i], label[j], sfx[i]);
                return false;
            }
        }
    }
    return true;
}

/* Resolve one radar tank's Modbus slave the way the poll path will
 * (get_radar_slave, further down): a numeric suffix IS the address, anything
 * else falls back to the field convention. Shared by the validator so the two
 * can never disagree. */
static uint8_t radar_slave_for(const char *sfx, int fallback) {
    if (sfx && sfx[0] >= '0' && sfx[0] <= '9') {
        int v = atoi(sfx);
        if (v > 0 && v <= 247) return (uint8_t)v;
    }
    return (uint8_t)fallback;
}

/* Every Modbus unit this plan would poll — each radar tank and the genset —
 * must resolve to a DIFFERENT slave address. Their read requests are
 * byte-identical apart from the slave byte (and, for some types, the function
 * code), so two units on one address means one physical device answering both
 * queries, its registers decoded once as tank level and once as genset state,
 * with every slave/FC/CRC echo check passing. The suffix check above catches
 * most of this because the suffix is the address — this covers the rest: a
 * lettered suffix falling back to an auto address that a numeric suffix
 * already claims, and a genset whose slave is above 99 (no 2-char suffix). */
static bool validate_bus_addresses(const DeviceConfig *c, char *err, size_t err_size) {
    uint8_t     slave[2 * RADAR_MAX + 1];
    char        label[2 * RADAR_MAX + 1][12];
    int         n = 0;

    for (int i = 0; i < c->ztech_count; i++) {
        snprintf(label[n], sizeof(label[n]), "Ztech%d", i + 1);
        slave[n++] = radar_slave_for(c->sfx_ztech[i], ZTECH_SLAVE_BASE + i);
    }
    for (int i = 0; i < c->pantech_count; i++) {
        snprintf(label[n], sizeof(label[n]), "Pantech%d", i + 1);
        slave[n++] = radar_slave_for(c->sfx_pantech[i], ZTECH_SLAVE_BASE + c->ztech_count + i);
    }
    if (c->genset_type != GENSET_OFF) {
        snprintf(label[n], sizeof(label[n]), "Genset");
        slave[n++] = c->genset_slave;
    }
    for (int i = 0; i < n; i++) {
        if (slave[i] < 1 || slave[i] > 247) {
            snprintf(err, err_size, "%s resolves to Modbus slave %u (must be 1..247)", label[i], slave[i]);
            return false;
        }
        for (int j = i + 1; j < n; j++) {
            if (slave[i] == slave[j]) {
                snprintf(err, err_size, "%s and %s both resolve to Modbus slave %u",
                         label[i], label[j], slave[i]);
                return false;
            }
        }
    }
    return true;
}

/* Case-insensitive string equality — the old config portal produced radar
 * types as "Ztech"/"Pantech" (capitalised), so accept any casing. */
static bool streq_ci(const char *a, const char *b) {
    while (*a && *b) {
        char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
        if (ca != cb) return false;
        a++; b++;
    }
    return *a == *b;
}

/* Connectivity strings (APN + broker fields) are embedded verbatim inside
 * quoted AT command arguments — AT+QICSGP (network.c), AT+QMTOPEN/AT+QMTCONN
 * (mqtt.c). A double quote in any of them breaks the AT syntax outright, and
 * an embedded CR/LF terminates the command line early — which would let a
 * setconfig payload inject arbitrary AT commands after it. The connectivity
 * trial would eventually roll such a push back, but reject it up front with
 * a clear reason instead of burning a reboot + trial on a value that can
 * never work (and to keep even a transient injection window closed). */
static bool conn_str_ok(const char *s) {
    for (; *s; s++)
        if (*s == '"' || *s == '\\' || (unsigned char)*s < 0x20) return false;
    return true;
}

/* Dispenser slots (the btpl meter rule). Slot N (du1..du6) is ON exactly when
 * du<N>Sfx is set, and that suffix is the dispenser number = bus address =
 * HW_ID suffix; "" turns the slot OFF. A station with three BlueSky
 * dispensers whose first is out of service sends du1Sfx "" with du2Sfx "02"
 * and du3Sfx "03": dispensers 2 and 3 are polled and publish as ...02 and
 * ...03, and the pumps are never re-addressed. du<N>Make sets the slot's
 * protocol (bluesky / yenen), as before.
 *
 * duCount is a cross-check, not a switch: when sent it must equal the number
 * of slots ON in the merged plan (this push over what is stored), else the
 * push is refused with both numbers — a config page that ticked a dispenser
 * but sent no suffix for it would otherwise be acked and poll nothing. Absent,
 * the count simply follows the suffixes. Format, range and uniqueness of the
 * suffixes are judged by validate_suffixes() on the merged plan. */
static bool setcfg_du_slots(const char *json, DeviceConfig *nc, char *err, size_t err_size) {
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
        char key_m[32], key_s[32], make_str[32];
        bool make_present = false;
        snprintf(key_m, sizeof(key_m), "du%dMake", i + 1);
        snprintf(key_s, sizeof(key_s), "du%dSfx", i + 1);
        if (!setcfg_get_text(json, key_m, make_str, sizeof(make_str), &make_present, err, err_size))
            return false;
        if (make_present) {
            if (streq_ci(make_str, "bluesky")) {
                nc->du_make[i] = DU_MAKE_BLUESKY;
            } else if (streq_ci(make_str, "yenen")) {
                nc->du_make[i] = DU_MAKE_YENEN;
            } else {
                /* Refuse rather than ignore: a typo here would leave the slot
                 * on whatever make it had before (BlueSky on a fresh unit) and
                 * the operator would see "config_successful". */
                snprintf(err, err_size, "du_make_invalid_use_bluesky_or_yenen");
                return false;
            }
        }
        if (!setcfg_get_sfx(json, key_s, nc->du_sfx[i], sizeof(nc->du_sfx[i]), err, err_size))
            return false;
    }

    int on = 0;
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) if (nc->du_sfx[i][0]) on++;
    int cnt = on;
    if (!setcfg_int_in_range(json, "duCount", 0, DU_MAX_DISPENSERS, &cnt, err, err_size))
        return false;
    if (cnt != on) {
        snprintf(err, err_size, "duCount %d but %d dispenser suffixes set (a dispenser slot is ON "
                                "only with its duNSfx)", cnt, on);
        return false;
    }
    nc->du_count = (uint8_t)on;
    nc->du_plan  = DU_PLAN_SLOTS;
    return true;
}

/* Publish a config_failed status with a machine-readable reason. */
static void setcfg_fail(const char *reason) {
    /* Sized for the longest reason a reader or validator writes (verr, 128)
     * plus the envelope and IMEI, so the JSON is never cut mid-string. */
    char err[256];
    snprintf(err, sizeof(err),
             "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"%s\"}",
             g_imei, reason);
    mqtt_publish(t_logs, err);
}

/* factory_reset's two checks, because nothing it erases can be undone. The
 * refusal reason, or NULL when the reset may go ahead:
 *   confirm  must be this gateway's own IMEI — a reset published to the wrong
 *            unit's topic, or copied from another unit's page, wipes nothing;
 *   unsent   readings still queued are sales not yet delivered: refused, with
 *            the count, unless the push says "force":true to discard them.
 * `why` holds the counted reason. Pure, so the host tests run it. */
static const char *factory_reset_refusal(const char *confirm, const char *imei, int unsent,
                                         bool force, char *why, size_t why_size) {
    if (!confirm || !confirm[0])             return "confirm_required";
    if (!imei || strcmp(confirm, imei) != 0) return "confirm_mismatch";
    if (unsent > 0 && !force) {
        snprintf(why, why_size, "unsent_readings_%d", unsent);
        return why;
    }
    return NULL;
}

static const char *du_make_str(uint8_t make) {
    if (make == DU_MAKE_BLUESKY) return "bluesky";
    if (make == DU_MAKE_YENEN)   return "yenen";
    return "unknown";
}

/* Read side of the read/write pair the old local web portal gave for free.
 * Publishes the LIVE DeviceConfig on the logs topic so the fleet is auditable
 * ("what is station X set to / did my change take?") without a site visit.
 * Secrets are never echoed: passwords are reported only as *Set booleans, so
 * a report can be copied, tweaked and re-sent without accidentally writing a
 * literal "***" over the real password (omit the pass key to keep it). */
static void publish_config_report(void) {
    char buf[MQTT_MODEM_PAYLOAD_SAFE_MAX];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"HW_ID\":\"%s\",\"status\":\"config_report\",\"fw\":\"%s\","
        "\"duCount\":%u,"
        "\"ztechCount\":%u,\"pantechCount\":%u,"
        "\"gensetType\":%u,\"gensetModel\":%u,\"gensetSlave\":%u,\"gensetBaud\":%lu,"
        "\"diEnable\":%u,\"gpsEnable\":%s,"
        "\"sfxGenset\":\"%s\",\"sfxDI\":\"%s\","
        "\"du1Make\":\"%s\",\"du1Sfx\":\"%s\","
        "\"du2Make\":\"%s\",\"du2Sfx\":\"%s\","
        "\"du3Make\":\"%s\",\"du3Sfx\":\"%s\","
        "\"du4Make\":\"%s\",\"du4Sfx\":\"%s\","
        "\"du5Make\":\"%s\",\"du5Sfx\":\"%s\","
        "\"du6Make\":\"%s\",\"du6Sfx\":\"%s\","
        "\"sfxZtech1\":\"%s\",\"sfxZtech2\":\"%s\",\"sfxZtech3\":\"%s\",\"sfxZtech4\":\"%s\","
        "\"sfxPantech1\":\"%s\",\"sfxPantech2\":\"%s\",\"sfxPantech3\":\"%s\",\"sfxPantech4\":\"%s\","
        "\"apn\":\"%s\",\"apnUser\":\"%s\",\"apnPassSet\":%s,"
        "\"mqttHost\":\"%s\",\"mqttPort\":%u,\"mqttUser\":\"%s\","
        "\"mqttPassSet\":%s,\"mqttBaseTopic\":\"%s\",\"mqttTls\":%s,"
        "\"connPending\":%s"
        "}",
        g_imei, FW_VERSION,
        (unsigned)g_cfg.du_count,
        (unsigned)g_cfg.ztech_count, (unsigned)g_cfg.pantech_count,
        (unsigned)g_cfg.genset_type, (unsigned)g_cfg.genset_model, (unsigned)g_cfg.genset_slave,
        (unsigned long)g_cfg.genset_baud,
        (unsigned)g_cfg.di_enable, g_cfg.gps_enable ? "true" : "false",
        g_cfg.sfx_genset, g_cfg.sfx_di,
        du_make_str(g_cfg.du_make[0]), g_cfg.du_sfx[0],
        du_make_str(g_cfg.du_make[1]), g_cfg.du_sfx[1],
        du_make_str(g_cfg.du_make[2]), g_cfg.du_sfx[2],
        du_make_str(g_cfg.du_make[3]), g_cfg.du_sfx[3],
        du_make_str(g_cfg.du_make[4]), g_cfg.du_sfx[4],
        du_make_str(g_cfg.du_make[5]), g_cfg.du_sfx[5],
        g_cfg.sfx_ztech[0], g_cfg.sfx_ztech[1], g_cfg.sfx_ztech[2], g_cfg.sfx_ztech[3],
        g_cfg.sfx_pantech[0], g_cfg.sfx_pantech[1], g_cfg.sfx_pantech[2], g_cfg.sfx_pantech[3],
        g_cfg.apn, g_cfg.apn_user, g_cfg.apn_pass[0] ? "true" : "false",
        g_cfg.mqtt_host, (unsigned)g_cfg.mqtt_port, g_cfg.mqtt_user,
        g_cfg.mqtt_pass[0] ? "true" : "false", g_cfg.mqtt_base_topic,
        g_cfg.mqtt_tls ? "true" : "false",
        g_cfg.conn_pending ? "true" : "false");
    if (n >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "config_report truncated (%d/%zu) — sending minimal", n, sizeof(buf));
        snprintf(buf, sizeof(buf),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_report\",\"error\":\"truncated\"}", g_imei);
    }
    mqtt_publish(t_logs, buf);
}

/* Defined below, with the replay guard. */
static bool payload_is_one_json_object(const char *s);

static void parse_setconfig(const char *json) {
    char verr[128];   /* refusal reason from the field readers and validators */

    /* ── The payload as a whole, before any field is read ──────────────────
     * ONE complete JSON object (a push cut short — by the publisher, or by the
     * 2 KB inbound clamp in mqtt.c — would otherwise apply the fields that
     * happened to arrive), in which every key is one this firmware knows and
     * appears once. See setcfg_check_members(). */
    if (!payload_is_one_json_object(json)) {
        ESP_LOGW(TAG, "setconfig: not one complete JSON object — ignored (no reboot)");
        setcfg_fail("bad_payload");
        return;
    }
    if (!setcfg_check_members(json, s_setcfg_known_keys,
                              sizeof(s_setcfg_known_keys) / sizeof(s_setcfg_known_keys[0]),
                              s_setcfg_extra_keys,
                              sizeof(s_setcfg_extra_keys) / sizeof(s_setcfg_extra_keys[0]),
                              verr, sizeof(verr))) {
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        setcfg_fail(verr);
        return;
    }
    bool known_key = false;
    {
        char search[24];
        for (size_t i = 0; i < sizeof(s_setcfg_known_keys) / sizeof(s_setcfg_known_keys[0]); i++) {
            snprintf(search, sizeof(search), "\"%s\":", s_setcfg_known_keys[i]);
            if (strstr(json, search)) { known_key = true; break; }
        }
    }

    /* Remote commands — parsed with json_get_str so both compact
     * ("cmd":"restart") and pretty-printed ("cmd": "restart") JSON work. A
     * command travels ALONE: {"cmd":"restart","gpsEnable":true} used to
     * restart and drop the setting without a word. */
    char cmdbuf[24] = "";
    bool cmd_present = false;
    if (!setcfg_get_text(json, "cmd", cmdbuf, sizeof(cmdbuf), &cmd_present, verr, sizeof(verr))) {
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        setcfg_fail(verr);
        return;
    }
    if (cmd_present && known_key) {
        ESP_LOGW(TAG, "setconfig: rejected — cmd '%s' sent together with config keys", cmdbuf);
        setcfg_fail("cmd_must_be_sent_alone");
        return;
    }
    /* "confirm" and "force" belong to factory_reset alone (s_setcfg_extra_keys
     * lets them through the member check for it). Anywhere else they are
     * refused as the unknown keys they were before. */
    if (!(cmd_present && strcmp(cmdbuf, "factory_reset") == 0)) {
        const char *stray = json_has_key(json, "confirm") ? "unknown_key_confirm"
                          : json_has_key(json, "force")   ? "unknown_key_force"
                          : NULL;
        if (stray) {
            ESP_LOGW(TAG, "setconfig: rejected — %s", stray);
            setcfg_fail(stray);
            return;
        }
    }
    if (cmd_present) {
        /* One-shot commands: a RETAINED copy would re-run them on every
         * reconnect (a reboot loop, a wipe on every boot). Dropped inside the
         * grace window with an ack that says why — see CMD_RETAIN_GRACE_MS. */
        if ((strcmp(cmdbuf, "restart") == 0 || strcmp(cmdbuf, "erase_config") == 0 ||
             strcmp(cmdbuf, "factory_reset") == 0) &&
            cmd_in_retain_grace(cmdbuf)) {
            char ack[160];
            snprintf(ack, sizeof(ack),
                     "{\"HW_ID\":\"%s\",\"status\":\"cmd_ignored_retained\",\"cmd\":\"%s\"}",
                     g_imei, cmdbuf);
            mqtt_publish(t_logs, ack);
            return;
        }
        if (strcmp(cmdbuf, "restart") == 0) {
            ESP_LOGI(TAG, "Remote restart");
            /* ACK before we vanish — mqtt_publish is synchronous (waits for the
             * QMTPUBEX result), so the cloud sees the command was accepted
             * rather than just watching the device drop off and reappear. */
            char ack[128];
            snprintf(ack, sizeof(ack),
                     "{\"HW_ID\":\"%s\",\"status\":\"restarting\"}", g_imei);
            mqtt_publish(t_logs, ack);
            vTaskDelay(pdMS_TO_TICKS(500));
            ota_confirm_before_restart("remote restart");
            du_task_flush_to_store();
            esp_restart();
        }
        if (strcmp(cmdbuf, "erase_config") == 0) {
            /* The station's equipment configuration only — dispenser slots,
             * radar tanks, genset, DI, GPS and their suffixes, all back to OFF
             * (nvs_config_clear_equipment). The APN and broker are kept, so the
             * gateway comes back on the network and broker it is on now,
             * "unconfigured", ready for the station's new setconfig. Also kept:
             * the unsent readings (the RAM ones go to flash first, as on a
             * restart, and all of them are delivered after it), the FOTA URL
             * memory and the boot counter. Saved BEFORE the ack, as a setconfig
             * is, so "erasing_config" is only ever said for a stored erase. */
            DeviceConfig nc;
            memcpy(&nc, &g_cfg, sizeof(nc));
            nvs_config_clear_equipment(&nc);
            if (!nvs_config_save(&nc)) {
                ESP_LOGE(TAG, "erase_config: NVS save failed — configuration left as it was");
                setcfg_fail("nvs_save_failed");
                return;
            }
            ESP_LOGW(TAG, "Remote erase_config — equipment configuration erased, restarting");
            char ack[128];
            snprintf(ack, sizeof(ack),
                     "{\"HW_ID\":\"%s\",\"status\":\"erasing_config\"}", g_imei);
            mqtt_publish(t_logs, ack);
            vTaskDelay(pdMS_TO_TICKS(500));
            ota_confirm_before_restart("erase config");
            du_task_flush_to_store();
            esp_restart();
        }
        if (strcmp(cmdbuf, "factory_reset") == 0) {
            /* Everything this gateway has stored goes: the configuration, the
             * unsent readings (flash and RAM), the FOTA URL memory and the boot
             * counter. Kept: the installed firmware and the compiled-in
             * APN/broker it comes back on, "unconfigured", as a freshly flashed
             * unit does. Only past factory_reset_refusal()'s two checks. */
            char confirm[24] = "";
            bool confirm_present = false, force = false;
            if (!setcfg_get_text(json, "confirm", confirm, sizeof(confirm), &confirm_present,
                                 verr, sizeof(verr)) ||
                !setcfg_get_bool(json, "force", &force, verr, sizeof(verr))) {
                ESP_LOGW(TAG, "factory_reset: rejected — %s", verr);
                setcfg_fail(verr);
                return;
            }
            int  unsent = store_pending();
            char why[40];
            const char *refusal = factory_reset_refusal(confirm, g_imei, unsent, force,
                                                        why, sizeof(why));
            if (refusal) {
                ESP_LOGW(TAG, "factory_reset: rejected — %s", refusal);
                setcfg_fail(refusal);
                return;
            }
            /* The queue first, the configuration last, and no RAM spill. A
             * queue that cannot be emptied stops the reset here, with nothing
             * erased; cut short after it, the gateway comes back still
             * configured and says "ready" — visibly not reset, so the command
             * is sent again — and never "unconfigured" while still holding the
             * old station's readings. */
            int discarded = unsent + du_task_pending();
            if (store_ready() && !store_purge()) {
                ESP_LOGE(TAG, "factory_reset: the unsent readings could not be erased — nothing erased");
                setcfg_fail("store_purge_failed");
                return;
            }
            ESP_LOGW(TAG, "Remote factory_reset — erasing everything, %d unsent reading(s) discarded",
                     discarded);
            /* ACK before wiping NVS + rebooting, same reasoning as restart. */
            char ack[160];
            snprintf(ack, sizeof(ack),
                     "{\"HW_ID\":\"%s\",\"status\":\"factory_resetting\",\"discarded\":%d}",
                     g_imei, discarded);
            mqtt_publish(t_logs, ack);
            vTaskDelay(pdMS_TO_TICKS(500));
            ota_confirm_before_restart("factory reset");
            nvs_config_factory_reset();
            esp_restart();
        }
        /* Pure read — publishes current config and returns WITHOUT touching
         * NVS or rebooting. Handled here (before the known-key gate below) so
         * a bare {"cmd":"getconfig"} isn't rejected as an unknown payload. */
        if (strcmp(cmdbuf, "getconfig") == 0) {
            ESP_LOGI(TAG, "Remote getconfig");
            publish_config_report();
            return;
        }
        /* Pure read too: the diagnostics report (guide 4.2) now instead of at
         * the end of its window. It IS that report — the window counters clear
         * on a confirmed publish and the next periodic one follows a full
         * DIAG_CYCLE_INTERVAL later — so every report still covers exactly the
         * time since the one before it. */
        if (strcmp(cmdbuf, "diagnostics") == 0) {
            ESP_LOGI(TAG, "Remote diagnostics");
            char ts[32];
            rtc_timestamp(ts, sizeof(ts));
            forensics_mark(FSLOT_MAIN, F_DIAG, 0);
            diag_publish(g_imei, ts, t_logs);
            g_diag_cycle = 0;
            return;
        }
        ESP_LOGW(TAG, "setconfig: rejected — unknown cmd '%s'", cmdbuf);
        setcfg_fail("unknown_cmd");
        return;
    }

    /* Reject a payload naming nothing to set (e.g. {}). A stray message
     * published to the setconfig topic (seen in the field: a FOTA URL sent to
     * the wrong topic) previously fell through all the field defaults,
     * "applied" nothing — and still saved NVS and REBOOTED the device. The
     * shape and every key were already checked at the top. */
    if (!known_key) {
        ESP_LOGW(TAG, "setconfig: unrecognised payload — ignored (no reboot)");
        char err[128];
        snprintf(err, sizeof(err),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"bad_payload\"}",
                 g_imei);
        mqtt_publish(t_logs, err);
        return;
    }

    char st[128];
    snprintf(st, sizeof(st),
             "{\"HW_ID\":\"%s\",\"status\":\"config_received\"}", g_imei);
    mqtt_publish(t_logs, st);

    /* Stage into a copy so a validation failure below leaves the live
     * g_cfg (and the running device) completely untouched.
     *
     * memcpy, not `DeviceConfig nc = g_cfg;`. The duplicate-config guard
     * further down compares the two with memcmp, which reads PADDING bytes as
     * well as members. Struct assignment is not required by the C standard to
     * copy padding — GCC happens to, but relying on that makes the guard a
     * property of the compiler rather than of this code. memcpy is defined to
     * copy every byte, so the comparison is exact by construction.
     *
     * The failure direction is benign either way (a missed match just means an
     * unnecessary save+reboot), but the guard is what stops a RETAINED
     * setconfig from reboot-looping a gateway forever, so it should not rest
     * on an implementation detail. */
    DeviceConfig nc;
    memcpy(&nc, &g_cfg, sizeof(nc));

    /* Dispenser slots — see setcfg_du_slots(): a slot is ON only with its
     * suffix, and duCount must agree with the slots ON. Which dispenser each
     * slot names (its number = its address), whether a Yenen is numbered
     * beyond head 5, and whether two slots name the same dispenser are all
     * checked on the MERGED plan by validate_suffixes() below — a make may
     * come from NVS and a suffix from this push. */
    if (!setcfg_du_slots(json, &nc, verr, sizeof(verr))) {
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        setcfg_fail(verr);
        return;
    }

    /* ── Radar — two input styles accepted, never mixed ────────────────────
     * (a) single-type style : radarType ("ztech"/"pantech"/"none") +
     *                         tankCount. Selecting one type turns the other
     *                         OFF, so switching a site's gauge vendor is one
     *                         push.
     * (b) explicit style    : ztechCount / pantechCount (lets a site run both
     *                         types at once).
     * Anything ambiguous is REJECTED loudly rather than silently guessed — a
     * stray "tankCount" with no "radarType" used to be dropped with no hint
     * that the radar config never applied. Per-tank slaves are the suffixes;
     * validate_bus_addresses() below refuses any two units on one address. */
    bool has_rtype  = json_has_key(json, "radarType");
    bool has_tcount = json_has_key(json, "tankCount");
    bool has_zt     = json_has_key(json, "ztechCount");
    bool has_pt     = json_has_key(json, "pantechCount");

    if (has_rtype || has_tcount) {
        if (has_zt || has_pt) {
            ESP_LOGW(TAG, "setconfig: rejected — mixed radar styles");
            setcfg_fail("radar_mixed_style_use_radarType+tankCount_OR_ztechCount/pantechCount");
            return;
        }
        if (!has_rtype || !has_tcount) {
            ESP_LOGW(TAG, "setconfig: rejected — radarType/tankCount not paired");
            setcfg_fail("radar_incomplete_send_radarType_and_tankCount_together");
            return;
        }
        char rtype[16] = "";
        bool rtype_present = false;
        int  tc = 0;
        if (!setcfg_get_text(json, "radarType", rtype, sizeof(rtype), &rtype_present, verr, sizeof(verr)) ||
            !setcfg_int_in_range(json, "tankCount", 0, RADAR_MAX, &tc, verr, sizeof(verr))) {
            ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
            setcfg_fail(verr);
            return;
        }
        /* The single-type styles each zero the OTHER, so switching a site
         * from one radar vendor to another with one push can't leave the old
         * type still being polled. */
        if (streq_ci(rtype, "ztech")) {
            nc.ztech_count = (uint8_t)tc; nc.pantech_count = 0;
        } else if (streq_ci(rtype, "pantech")) {
            nc.pantech_count = (uint8_t)tc; nc.ztech_count = 0;
        } else if (streq_ci(rtype, "none") || streq_ci(rtype, "off")) {
            nc.ztech_count = 0; nc.pantech_count = 0;
        } else {
            ESP_LOGW(TAG, "setconfig: rejected — bad radarType '%s'", rtype);
            setcfg_fail("radar_type_invalid_use_ztech_pantech_or_none");
            return;
        }
    } else {
        int ztech = nc.ztech_count, pantech = nc.pantech_count;
        if (!setcfg_int_in_range(json, "ztechCount",   0, RADAR_MAX, &ztech,   verr, sizeof(verr)) ||
            !setcfg_int_in_range(json, "pantechCount", 0, RADAR_MAX, &pantech, verr, sizeof(verr))) {
            ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
            setcfg_fail(verr);
            return;
        }
        nc.ztech_count   = (uint8_t)ztech;
        nc.pantech_count = (uint8_t)pantech;
    }

    /* Every genset field and switch is range-checked and REFUSED out of range
     * (setcfg_int_in_range / setcfg_get_bool): gensetModel 12 used to become
     * model 0 and gensetBaud 2400 became 9600, both acked config_successful.
     * The genset address pair is its own reader. */
    int gtype = nc.genset_type, di_mask = nc.di_enable, gmodel = nc.genset_model;
    int gbaud = (int)nc.genset_baud;
    bool gps_on = nc.gps_enable;
    if (!setcfg_int_in_range(json, "gensetType", GENSET_OFF, GENSET_SEDEMAC, &gtype, verr, sizeof(verr)) ||
        !setcfg_int_in_range(json, "diEnable", 0, 0x0F, &di_mask, verr, sizeof(verr)) ||
        !setcfg_int_in_range(json, "gensetModel", 0, GENSET_MODEL_MAX, &gmodel, verr, sizeof(verr)) ||
        !setcfg_int_in_range(json, "gensetBaud", 1, 1000000, &gbaud, verr, sizeof(verr)) ||
        !setcfg_get_bool(json, "gpsEnable", &gps_on, verr, sizeof(verr)) ||
        !setcfg_genset_addr(json, &nc, verr, sizeof(verr))) {
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        setcfg_fail(verr);
        return;
    }
    if (json_has_key(json, "gensetBaud") && gbaud != 4800 && gbaud != 9600 && gbaud != 19200) {
        ESP_LOGW(TAG, "setconfig: rejected — gensetBaud %d", gbaud);
        setcfg_fail("gensetBaud must be 4800, 9600 or 19200");
        return;
    }
    nc.genset_type  = (uint8_t)gtype;
    nc.di_enable    = (uint8_t)di_mask;
    nc.genset_model = (uint8_t)gmodel;
    nc.genset_baud  = (uint32_t)gbaud;
    nc.gps_enable   = gps_on;

    /* Baud whitelist, kept as a belt for a STORED value (a pushed one was
     * refused above) — an out-of-range baud (0 in particular) reaches the UART
     * clock-divider division in rs485_bus_set_8n1() and panics the chip into a
     * reboot loop. */
    if (nc.genset_baud != 4800 && nc.genset_baud != 9600 && nc.genset_baud != 19200)
        nc.genset_baud = 9600;

    char keybuf[16];
    bool sfx_ok = setcfg_get_sfx(json, "sfxDI", nc.sfx_di, sizeof(nc.sfx_di), verr, sizeof(verr));
    for (int i = 0; i < RADAR_MAX && sfx_ok; i++) {
        snprintf(keybuf, sizeof(keybuf), "sfxZtech%d", i + 1);
        sfx_ok = setcfg_get_sfx(json, keybuf, nc.sfx_ztech[i], sizeof(nc.sfx_ztech[i]), verr, sizeof(verr));
    }
    for (int i = 0; i < RADAR_MAX && sfx_ok; i++) {
        snprintf(keybuf, sizeof(keybuf), "sfxPantech%d", i + 1);
        sfx_ok = setcfg_get_sfx(json, keybuf, nc.sfx_pantech[i], sizeof(nc.sfx_pantech[i]), verr, sizeof(verr));
    }
    if (!sfx_ok) {
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        setcfg_fail(verr);
        return;
    }

    /* ── Connectivity: APN + MQTT broker (optional — only touched if an
     * apn/mqtt key is present) ───────────────────────────────────────────
     * g_cfg's primary apn and mqtt fields are always the last connectivity
     * config that PROVED it reached a live broker session (app_main's
     * pending-trial logic enforces this on every boot), so they're exactly
     * what a bad push should roll back to. Stage them into nc's _bak fields
     * from g_cfg (still untouched here) before nc's primary fields change. */
    /* Every connectivity value is read strictly (setcfg_get_text and friends):
     * a value that is not a string, or that does not FIT its field, is refused
     * here — json_get_str() used to cut it to size, and a truncated password
     * only surfaced as a failed trial and a rollback. */
    bool apn_present = false, auser_present = false, apass_present = false;
    bool host_present = false, user_present = false, pass_present = false, topic_present = false;
    bool port_present  = json_has_key(json, "mqttPort");
    int  port_val      = nc.mqtt_port;
    bool tls_present   = json_has_key(json, "mqttTls");
    bool tls_val       = nc.mqtt_tls;
    if (!setcfg_get_text(json, "apn",           nc.apn,             sizeof(nc.apn),             &apn_present,   verr, sizeof(verr)) ||
        !setcfg_get_text(json, "apnUser",       nc.apn_user,        sizeof(nc.apn_user),        &auser_present, verr, sizeof(verr)) ||
        !setcfg_get_text(json, "apnPass",       nc.apn_pass,        sizeof(nc.apn_pass),        &apass_present, verr, sizeof(verr)) ||
        !setcfg_get_text(json, "mqttHost",      nc.mqtt_host,       sizeof(nc.mqtt_host),       &host_present,  verr, sizeof(verr)) ||
        !setcfg_get_text(json, "mqttUser",      nc.mqtt_user,       sizeof(nc.mqtt_user),       &user_present,  verr, sizeof(verr)) ||
        !setcfg_get_text(json, "mqttPass",      nc.mqtt_pass,       sizeof(nc.mqtt_pass),       &pass_present,  verr, sizeof(verr)) ||
        !setcfg_get_text(json, "mqttBaseTopic", nc.mqtt_base_topic, sizeof(nc.mqtt_base_topic), &topic_present, verr, sizeof(verr)) ||
        !setcfg_int_in_range(json, "mqttPort", 1, 65535, &port_val, verr, sizeof(verr)) ||
        !setcfg_get_bool(json, "mqttTls", &tls_val, verr, sizeof(verr))) {
        /* Never echo the value: it may be a password. The reason names the key. */
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        setcfg_fail(verr);
        return;
    }

    /* Resolve mqttPort/mqttTls into nc now — port_val/tls_val already default to
     * the current values when their keys are absent, so this is a no-op unless
     * they were actually sent. Doing it here (rather than only inside the
     * connectivity block below) lets the duplicate-config check see the fully
     * resolved effective config. */
    nc.mqtt_port = (uint16_t)port_val;
    nc.mqtt_tls  = tls_val;

    /* ── Retained / duplicate setconfig guard ────────────────────────────────
     * If the fully-parsed config is byte-identical to the live one, this is
     * almost certainly a RETAINED setconfig the broker re-delivers on every
     * reconnect (the FOTA path guards the same hazard via s_fota_blocked).
     * Saving + rebooting on it would loop the device forever: reboot →
     * reconnect → broker re-delivers the retained message → reboot. Acknowledge
     * it (so the cloud still sees success) but do NOT save or reboot.
     *
     * memcmp is exact and fail-safe here: a genuinely different value always
     * makes it non-zero, so a real change can NEVER be skipped; at worst struct
     * padding makes it miss this optimization and fall through to the normal
     * save+reboot path. nc's _bak/conn_pending/conn_rollback_notify are still
     * copied verbatim from g_cfg at this point (the connectivity block below is
     * the first thing to touch them), so they compare equal and don't taint the
     * result. */
    if (memcmp(&nc, &g_cfg, sizeof(nc)) == 0) {
        ESP_LOGI(TAG, "setconfig: no effective change vs. live config — "
                      "ack without save/reboot (retained or duplicate?)");
        char ok[128];
        snprintf(ok, sizeof(ok),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_successful\",\"fw\":\"%s\"}",
                 g_imei, FW_VERSION);
        mqtt_publish(t_logs, ok);
        return;
    }

    bool apn_changed  = apn_present || auser_present || apass_present;
    bool mqtt_changed = host_present || user_present || pass_present ||
                        topic_present || port_present || tls_present;

    if (apn_changed || mqtt_changed) {
        /* Validate the resulting fields (an omitted key leaves nc holding the
         * current value, which is already known-good). */
        if (nc.apn[0] == '\0') {
            ESP_LOGW(TAG, "setconfig: rejected — empty APN");
            char err[128];
            snprintf(err, sizeof(err),
                     "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"empty_apn\"}",
                     g_imei);
            mqtt_publish(t_logs, err);
            return;
        }
        if (nc.mqtt_host[0] == '\0' || nc.mqtt_base_topic[0] == '\0' ||
            port_val < 1 || port_val > 65535 ||
            strchr(nc.mqtt_base_topic, '#') || strchr(nc.mqtt_base_topic, '+')) {
            ESP_LOGW(TAG, "setconfig: rejected — invalid MQTT broker fields");
            char err[128];
            snprintf(err, sizeof(err),
                     "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"bad_mqtt_broker_fields\"}",
                     g_imei);
            mqtt_publish(t_logs, err);
            return;
        }
        if (!conn_str_ok(nc.apn)       || !conn_str_ok(nc.apn_user)  ||
            !conn_str_ok(nc.apn_pass)  || !conn_str_ok(nc.mqtt_host) ||
            !conn_str_ok(nc.mqtt_user) || !conn_str_ok(nc.mqtt_pass) ||
            !conn_str_ok(nc.mqtt_base_topic)) {
            ESP_LOGW(TAG, "setconfig: rejected — quote/control char in a connectivity field");
            setcfg_fail("invalid_chars_in_connectivity_fields");
            return;
        }
        /* mqtt_port/mqtt_tls are already resolved into nc above (before the
         * duplicate-config guard), so they don't need re-applying here. */
        /* Stage the whole current connectivity set as the rollback target —
         * APN and broker share one pending flag, so a trial failure reverts
         * both together (see nvs_config.h). */
        strncpy(nc.apn_bak,             g_cfg.apn,             sizeof(nc.apn_bak) - 1);
        strncpy(nc.apn_user_bak,        g_cfg.apn_user,        sizeof(nc.apn_user_bak) - 1);
        strncpy(nc.apn_pass_bak,        g_cfg.apn_pass,        sizeof(nc.apn_pass_bak) - 1);
        strncpy(nc.mqtt_host_bak,       g_cfg.mqtt_host,       sizeof(nc.mqtt_host_bak) - 1);
        strncpy(nc.mqtt_user_bak,       g_cfg.mqtt_user,       sizeof(nc.mqtt_user_bak) - 1);
        strncpy(nc.mqtt_pass_bak,       g_cfg.mqtt_pass,       sizeof(nc.mqtt_pass_bak) - 1);
        strncpy(nc.mqtt_base_topic_bak, g_cfg.mqtt_base_topic, sizeof(nc.mqtt_base_topic_bak) - 1);
        nc.mqtt_port_bak = g_cfg.mqtt_port;
        nc.mqtt_tls_bak  = g_cfg.mqtt_tls;
        nc.conn_pending  = true;
        ESP_LOGI(TAG, "setconfig: connectivity change staged (apn=%d mqtt=%d) — trial on reboot",
                 apn_changed, mqtt_changed);
    }

    if (!validate_suffixes(&nc, verr, sizeof(verr)) ||
        !validate_bus_addresses(&nc, verr, sizeof(verr))) {
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        char err[320];
        snprintf(err, sizeof(err),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"%s\"}",
                 g_imei, verr);
        mqtt_publish(t_logs, err);
        return;
    }

    /* Turn the GNSS engine off cleanly on a live disable rather than leaving
     * it running until the modem's next power-cycle — cheap given the reboot
     * that follows a few lines down anyway. Compared against the OLD g_cfg
     * (not yet overwritten), so this only fires on an actual on->off edge. */
    if (g_cfg.gps_enable && !nc.gps_enable) gps_end();

    /* ── Save FIRST, apply to the live g_cfg SECOND ───────────────────────
     * The old order was `g_cfg = nc; if (!save) { ...keep running on the
     * in-RAM config... }`. That did two bad things:
     *
     *   1. On the save-failure path it left the NEW config live WITHOUT a
     *      reboot — the one code path where g_cfg changes and the device keeps
     *      running. du_task on core 1 reads g_cfg concurrently, so that path
     *      was the only lasting window in which it could observe a torn,
     *      half-updated struct (a whole-struct assignment is not atomic).
     *   2. It advertised the config as live while NVS still held the old one,
     *      so any later reboot silently reverted it.
     *
     * Persisting to NVS before touching the live struct removes both. Now
     * g_cfg is assigned ONLY after the save is confirmed, and every successful
     * save is immediately followed by esp_restart() below — so the live struct
     * changes at most ~500 ms before a reboot, never as a lasting concurrent
     * state. A save failure leaves the OLD config running, untouched. */
    if (!nvs_config_save(&nc)) {
        /* Not persisted (e.g. NVS partition full). Leave the live config
         * exactly as it was — do NOT apply nc — and report the failure, so a
         * restart cannot later reload a config we never actually stored. */
        char err[128];
        snprintf(err, sizeof(err),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"nvs_save_failed\"}",
                 g_imei);
        mqtt_publish(t_logs, err);
        return;
    }
    g_cfg = nc;   /* persisted — now safe to go live; reboot follows immediately */

    char ack[128];
    snprintf(ack, sizeof(ack),
             "{\"HW_ID\":\"%s\",\"status\":\"config_successful\",\"fw\":\"%s\"}",
             g_imei, FW_VERSION);
    mqtt_publish(t_logs, ack);
    vTaskDelay(pdMS_TO_TICKS(500));
    ota_confirm_before_restart("setconfig applied");
    du_task_flush_to_store();
    esp_restart();
}

/* ── rsp / kfactor tunnel (rsp_tunnel.h) ─────────────────────────────────
 * The cloud frames the command; the gateway only carries it. Here the payload
 * becomes bytes and joins the queue — the bus work is left to
 * execute_pending(), so this MQTT drain never blocks on a pump. A payload that
 * is not a frame, or a full queue, is answered "404" on the command's own
 * .../out at once: every command gets exactly one reply there. */
static void tunnel_accept(int unit, uint8_t kind, const char *payload) {
    const char *what = (kind == RSP_TUNNEL_KFACTOR) ? "kfactor" : "rsp";
    const char *out  = (kind == RSP_TUNNEL_KFACTOR) ? t_kf_out[unit] : t_rsp_out[unit];
    ESP_LOGI(TAG, "%s tunnel (%s) in: %.160s", what, g_hw_du[unit], payload);

    uint8_t frame[RSP_TUNNEL_REQ_MAX];
    size_t  len = rsp_tunnel_decode(payload, frame, sizeof(frame));
    if (len == 0) {
        ESP_LOGW(TAG, "%s tunnel (%s): not a hex frame of 1..%d bytes — nothing sent, 404",
                 what, g_hw_du[unit], RSP_TUNNEL_REQ_MAX);
        mqtt_publish(out, "404");
        return;
    }
    if (!rsp_tunnel_push((uint8_t)unit, kind, frame, len)) {
        ESP_LOGW(TAG, "%s tunnel (%s): %d commands already waiting — nothing sent, 404",
                 what, g_hw_du[unit], RSP_TUNNEL_QUEUE_MAX);
        mqtt_publish(out, "404");
    }
}

/* The running image's build id — the app version the build stamps in
 * (git describe, e.g. "4ab54b7" or "v1.0.1"; "-dirty" when built from
 * uncommitted code). Sent beside "fw" so the cloud can tell two builds apart
 * even when FW_VERSION was not bumped. Never contains a quote. */
static const char *running_build(void) {
    const esp_app_desc_t *d = esp_app_get_description();
    return (d && d->version[0]) ? d->version : "unknown";
}

/* An update refused without being started: its answer on /logs. Not sent
 * while the running image is still on trial — the push in flight then is the
 * update that brought it, whose one answer is that update's own
 * (check_post_ota, or a rollback), and a refusal of a stray copy (retained,
 * re-delivered) would be read as that push's answer. */
static void fota_refusal_publish(const char *json) {
    if (ota_is_pending_verify()) {
        ESP_LOGW(TAG, "FOTA: refusal not sent — the new image is on trial and its update "
                      "is the push in flight: %s", json);
        return;
    }
    mqtt_publish(t_logs, json);
}

/* ── MQTT message router ──────────────────────────────────────────────── */
static void handle_msg(const char *topic, const char *payload) {
    /* An OFF dispenser slot's command topics are "" (build_topics), so an empty
     * topic would match them. MQTT has no zero-length topic names; belt only. */
    if (!topic || !topic[0] || !payload) return;
    if (strcmp(topic, t_update) == 0 || strcmp(topic, t_fleet_update) == 0) {
        const bool fleet = (strcmp(topic, t_fleet_update) == 0);
        /* One byte past the cap: json_get_str() clamps, so a URL longer than
         * FOTA_URL_MAX arrives here one character too long and is refused as
         * url_too_long below, instead of being cut and downloaded as a
         * different, wrong URL. */
        char url_buf[FOTA_URL_MAX + 2] = {0};
        const char *url_to_use = payload;
        bool force = false;
        bool handover_ok = false;
        char fota_msg[224];

        /* ── What the cloud hears back ───────────────────────────────────────
         * The cloud stops waiting on a push at the first message whose status
         * is fota_successful or fota_failed, so every push this unit takes gets
         * exactly one of those two on /logs, with the detail in "reason";
         * fota_initiated is the only progress message. fota_successful comes
         * from the new image once it has confirmed itself after the reboot
         * (check_post_ota), never before it: a rollback must not follow a
         * success. A refusal is answered through fota_refusal_publish(), which
         * stays quiet while the new image is on trial. The checks of what a URL
         * already did (backstop #1) run BEFORE the retained-message window
         * (backstop #2), so a copy of an update this gateway knows gets that
         * update's answer rather than "retained". Every check only skips, so
         * their order changes the answer, never what the unit does. */

        /* Two payload forms: a bare URL, or JSON —
         *   {"url":"https://...","target":"gg","force":true,"handover":true}
         * target: a fleet message meant for another product is ignored. GG
         *         and GGY share one base topic in production, so with a
         *         mistyped fleet topic this belt is what keeps a GGY image off
         *         a GG unit. An absent target is accepted (bare URL contract).
         * force:  re-apply a URL this gateway has already applied, one whose
         *         image was rolled back, or one that failed earlier this boot
         *         (all three guards below). Operator-only.
         * handover: the link may be ANOTHER product's firmware (a deliberate
         *         swap). Without it such an image is refused as
         *         not_this_product (fota.c), so a wrong file cannot convert a
         *         station. A bare URL never allows it. */
        if (payload[0] == '{') {
            char target[32] = "";
            if (json_get_str(payload, "target", target, sizeof(target)) && target[0]) {
                if (!streq_ci(target, "gg") && !streq_ci(target, "alds-gg") &&
                    !streq_ci(target, "alds-gg-vvm701") &&
                    !streq_ci(target, "alds-gg-vvm701-fw")) {
                    ESP_LOGI(TAG, "FOTA target '%s' is not this product (gg) — ignored", target);
                    /* On this unit's own topic it is a push the cloud waits on;
                     * a fleet message for another product is not ours to answer. */
                    if (!fleet) {
                        snprintf(fota_msg, sizeof(fota_msg),
                                 "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"wrong_target\"}",
                                 g_imei);
                        fota_refusal_publish(fota_msg);
                    }
                    return;
                }
            }
            if (!json_get_str(payload, "url", url_buf, sizeof(url_buf))) {
                ESP_LOGW(TAG, "FOTA JSON payload missing 'url'");
                snprintf(fota_msg, sizeof(fota_msg),
                         "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"no_url\"}", g_imei);
                fota_refusal_publish(fota_msg);
                return;
            }
            url_to_use  = url_buf;
            force       = json_get_bool(payload, "force", false);
            handover_ok = json_get_bool(payload, "handover", false);
        }

        /* ── Retained-message backstop #1: what this URL already did ─────────
         * The grace window is a heuristic; this is the memory. Every URL that
         * SUCCEEDED is recorded in NVS before the restart it triggers, so the
         * image that boots next — same NVS — recognises its own URL and skips
         * it: without this, a retained good URL is a perpetual reflash loop
         * (boot -> resubscribe -> redeliver -> flash the other slot -> boot)
         * on every gateway that receives it. A URL whose image was later
         * ROLLED BACK by the bootloader is remembered too (marked at boot,
         * see app_main), or a retained URL would loop download -> boot ->
         * rollback instead. A different URL always clears the record;
         * "force":true overrides it for an operator who knows. */
        fota_url_state_t st = FOTA_URL_NONE;
        uint32_t         st_fp = 0, this_fp = nvs_fota_url_fp(url_to_use);
        st = nvs_fota_url_get(&st_fp);
        if (!force && st != FOTA_URL_NONE && st_fp == this_fp) {
            ESP_LOGW(TAG, "FOTA: this URL was already %s on this gateway — skipping "
                          "(publish a new URL, or the same one with \"force\":true)",
                     st == FOTA_URL_APPLIED ? "applied" : "applied and rolled back");
            /* Already installed is the outcome the push asked for. A copy of
             * the update still on trial gets no answer here (see
             * fota_refusal_publish): check_post_ota() gives that push's. */
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\",\"fw\":\"%s\","
                     "\"build\":\"%.32s\"}", g_imei,
                     st == FOTA_URL_APPLIED ? "fota_successful" : "fota_failed",
                     st == FOTA_URL_APPLIED ? "fota_skip_already_applied" : "fota_skip_reverted",
                     FW_VERSION, running_build());
            fota_refusal_publish(fota_msg);
            return;
        }

        /* Guard against a retained (or repeatedly re-published) update
         * message re-triggering a download that already FAILED this boot: a
         * RETAINED bad URL is re-delivered on EVERY reconnect, and each
         * attempt tears down MQTT for minutes — without this the device does
         * nothing but loop FOTA attempts. A different URL clears the block.
         * Compares the extracted URL, never the raw payload — the JSON form
         * would otherwise never match and the guard would be dead for exactly
         * the fleet messages it matters most for. "force" overrides this one
         * too (the operator's explicit retry of a URL that failed on, say, a
         * transient S3 error must not be answered fota_skip_known_bad). */
        static char s_fota_blocked[FOTA_URL_MAX + 1] = "";

        size_t url_len = strlen(url_to_use);
        if (url_len == 0 || url_len > FOTA_URL_MAX) {
            ESP_LOGE(TAG, "FOTA: URL is %u bytes (max %d) — rejected without downloading",
                     (unsigned)url_len, FOTA_URL_MAX);
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"url_too_long\"}",
                     g_imei);
            fota_refusal_publish(fota_msg);
            return;
        }
        /* Not an https:// link: refused HERE, as alds-sr does. fota_perform()
         * checks the same, but only after the fleet stagger, the confirm of a
         * pending image, fota_initiated and the MQTT teardown below — all of it,
         * and a reconnect with every subscription after, spent on text that
         * could never download. */
        if (!fota_url_acceptable(url_to_use)) {
            ESP_LOGE(TAG, "FOTA: not an https:// URL — rejected, MQTT left up");
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"non_https_url\"}",
                     g_imei);
            fota_refusal_publish(fota_msg);
            return;
        }
        if (!force && s_fota_blocked[0] && strcmp(url_to_use, s_fota_blocked) == 0) {
            ESP_LOGW(TAG, "FOTA: identical URL already failed this boot — skipping "
                          "(clear the retained message, publish a new URL, or force)");
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"fota_skip_known_bad\"}",
                     g_imei);
            fota_refusal_publish(fota_msg);
            return;
        }

        /* ── Retained-message backstop #2: the grace window ──────────────────
         * A retained URL is re-delivered on every reconnect, and the FOTA path
         * tears MQTT down for minutes each time. Dropped, with an answer that
         * names the cause, if it lands right after subscribing. After
         * backstop #1, so a copy of a URL this gateway already knows has had
         * that URL's own answer above. */
        if (cmd_in_retain_grace(fleet ? "fleet update" : "update")) {
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"fota_ignored_retained\"}",
                     g_imei);
            fota_refusal_publish(fota_msg);
            return;
        }

        /* Fleet trigger: stagger the start 0..29 s by the IMEI's last two
         * digits so twenty gateways do not open twenty S3 downloads on the same
         * cell in the same second. After every guard above, so a message that
         * will be skipped costs no wait; the WDT is fed through the sleep. */
        if (fleet) {
            size_t   imei_len = strlen(g_imei);
            uint32_t jitter_s = 0;
            if (imei_len >= 2 && isdigit((unsigned char)g_imei[imei_len - 2]) &&
                isdigit((unsigned char)g_imei[imei_len - 1]))
                jitter_s = (uint32_t)((g_imei[imei_len - 2] - '0') * 10 + (g_imei[imei_len - 1] - '0')) % 30u;
            ESP_LOGI(TAG, "fleet FOTA: staggering start by %lu s", (unsigned long)jitter_s);
            for (uint32_t w = 0; w < jitter_s; w++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_task_wdt_reset();
            }
        }

        /* esp_ota_begin() REFUSES an update while the running image is still
         * PENDING_VERIFY (ESP_ERR_OTA_ROLLBACK_INVALID_STATE). Receiving and
         * acting on this command proves the image alive as surely as the
         * poll cycle that normally confirms it — same argument as the other
         * deliberate-restart paths. */
        ota_confirm_before_restart("fota requested");
        /* This push supersedes any answer still owed to an EARLIER one (the
         * confirm just above creates one when the image was on trial): the
         * messages carry no push id, so one sent from here on would be read as
         * THIS push's answer. Dropped, not sent. */
        fota_owed_answers_drop();

        /* Last message before MQTT is torn down for the HTTP download — the
         * device goes radio-silent until this push's answer: fota_failed from
         * here, or fota_successful from the new image once it has confirmed
         * itself after the reboot. */
        snprintf(fota_msg, sizeof(fota_msg),
                 "{\"HW_ID\":\"%s\",\"status\":\"fota_initiated\"}", g_imei);
        mqtt_publish(t_logs, fota_msg);
        vTaskDelay(pdMS_TO_TICKS(500));

        /* FOTA start — keep blinking continuously through the whole download
         * + flash (MQTT is down, so there are no per-push blinks meanwhile). */
        led_blink_start();

        /* Free the modem's data session for HTTP — MQTT and QHTTP can't
         * share it cleanly. Reconnect afterwards if the download fails. */
        mqtt_disconnect();
        vTaskDelay(pdMS_TO_TICKS(1000));

        char fota_reason[48] = "unknown";
        bool fota_ok = fota_perform(url_to_use, t_logs, fota_reason, sizeof(fota_reason),
                                    handover_ok);

        led_blink_stop();   /* FOTA end — back to per-push blinks / reboot */

        if (fota_ok) {
            const bool foreign = fota_last_image_foreign();

            /* Remember the URL BEFORE the restart — the next boot is what
             * has to recognise it (backstop #1 above). Recorded for a
             * cross-product handover too: the incoming firmware never reads
             * it, but if the bootloader throws that image away and falls
             * back to us, it is what answers a retained copy of the URL with
             * fota_skip_reverted instead of another download-and-rollback. */
            nvs_fota_url_set(this_fp, FOTA_URL_APPLIED);

            /* Boot partition already set. An update of this product is answered
             * by the new image — fota_successful once it has confirmed itself
             * (check_post_ota), or fota_failed from this one if the bootloader
             * rolls it back — so nothing is sent now. A cross-product handover
             * is the exception: the incoming firmware reports on its own
             * product's topics, never here, so this is the last chance for this
             * push to be answered — reconnect just for that, then restart
             * regardless of publish success. Try twice: the first QMTOPEN right
             * after the HTTP session teardown can transiently fail (seen in the
             * field: 0,1). */
            if (foreign) {
                for (int i = 0; i < 2; i++) {
                    if (mqtt_connect()) {
                        snprintf(fota_msg, sizeof(fota_msg),
                                 "{\"HW_ID\":\"%s\",\"status\":\"fota_successful\","
                                 "\"reason\":\"handover\",\"handover\":true}", g_imei);
                        mqtt_publish(t_logs, fota_msg);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }

                /* Leave the MODEM neutral for the incoming product. With
                 * gpsEnable on, this firmware saved autogps/AGPS on in the
                 * modem's own NVRAM, and those switches outlive the app: a
                 * product that does not manage GNSS (cng-cpil) would otherwise
                 * run the GNSS engine and fetch AGPS data over the SIM for the
                 * rest of the module's life. Products that want GPS switch it
                 * back on at their boot. Same step as hsd-snf. Bounded AT
                 * queries only (gps.c), WDT-fed inside modem_at(). */
                gps_end();

                /* The incoming firmware is another product's. Spilling RAM to
                 * flash here would hand it this station's readings to publish
                 * as its own; and du_task may have written a round since
                 * fota_flash() purged. Empty the queue once more, right now,
                 * and go. Records produced in these last seconds are lost by
                 * design — they belong to a deployment this gateway is leaving. */
                store_purge();
            } else {
                du_task_flush_to_store();
            }
            esp_restart();
        }

        /* Download failed — remember this exact URL so a retained copy of it
         * can't re-trigger the loop, then bring MQTT back up. */
        strncpy(s_fota_blocked, url_to_use, sizeof(s_fota_blocked) - 1);
        s_fota_blocked[sizeof(s_fota_blocked) - 1] = '\0';

        /* This push's answer. Kept until a publish goes through: if this
         * reconnect fails, the poll loop sends it once the ladder has the
         * broker back, rather than the cloud never hearing why. Sent before
         * the subscriptions, so it precedes the answer to any retained copy
         * they bring back. */
        snprintf(fota_msg, sizeof(fota_msg),
                 "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"%s\"}",
                 g_imei, fota_reason);
        fota_failed_owe(fota_msg);
        /* The CA file before the reconnect: an attempt that power-cycled the
         * modem uploaded it again mid-update, and one cut short there would
         * leave every TLS connect failing until the next boot (tls_ca.c).
         * fota_perform() has already deleted its image file, which is what
         * makes room for it. */
        tls_ca_ensure();
        if (mqtt_connect()) {
            publish_fota_failed_owed();
            subscribe_all();
        } else {
            mqtt_reconnect_if_needed();
        }
        return;
    }
    if (strcmp(topic, t_setcfg) == 0) {
        if (setcfg_has_secret(payload))
            ESP_LOGI(TAG, "setconfig payload: [redacted — contains credentials]");
        else
            ESP_LOGI(TAG, "setconfig payload: [%s]", payload);
        parse_setconfig(payload);
        return;
    }
    if (strcmp(topic, t_lock) == 0) {
        /* ── Panel-box solenoid lock (RL1 via the MS51) ──
         * "unlock" contains "lock", so the more specific verb MUST be tested
         * first — reversing these two branches would turn every unlock command
         * into a force-lock. Kept in this order deliberately. */
        ESP_LOGI(TAG, "lock cmd: %s", payload);
        char msg[128];
        if (strstr(payload, "unlock")) {
            /* A retained unlock would open the panel box unattended on every
             * reconnect. "lock" is safe to repeat and is not gated. The lock has
             * its OWN LOCK_RETAIN_GRACE_MS (5 s) rather than the shared 15 s, to
             * match cng-cpil's lock contract exactly. */
            int64_t age_ms = (s_subscribed_at_us != 0)
                             ? (esp_timer_get_time() - s_subscribed_at_us) / 1000
                             : LOCK_RETAIN_GRACE_MS;
            if (s_subscribed_at_us != 0 && age_ms < LOCK_RETAIN_GRACE_MS) {
                ESP_LOGW(TAG, "unlock ignored — arrived %lld ms after subscribing, "
                              "treated as a RETAINED message", (long long)age_ms);
                snprintf(msg, sizeof(msg),
                         "{\"HW_ID\":\"%s\",\"event\":\"ignored_retained\"}", g_imei);
                mqtt_publish(t_lock_out, msg);
                return;
            }
            relay_unlock();
            snprintf(msg, sizeof(msg),
                     "{\"HW_ID\":\"%s\",\"event\":\"unlocked\",\"window_ms\":%d}",
                     g_imei, RELAY_UNLOCK_WINDOW_MS);
        } else if (strstr(payload, "lock")) {
            relay_force_lock();
            snprintf(msg, sizeof(msg),
                     "{\"HW_ID\":\"%s\",\"event\":\"locked\"}", g_imei);
        } else {
            snprintf(msg, sizeof(msg),
                     "{\"HW_ID\":\"%s\",\"event\":\"bad_command\"}", g_imei);
        }
        /* /lock/out, NEVER the command topic or <base>/data — see t_lock_out. */
        mqtt_publish(t_lock_out, msg);
        return;
    }
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
        if (strcmp(topic, t_rsp_in[i]) == 0) { tunnel_accept(i, RSP_TUNNEL_RSP, payload);     return; }
        if (strcmp(topic, t_kf_in[i])  == 0) { tunnel_accept(i, RSP_TUNNEL_KFACTOR, payload); return; }
    }
}

/* ── Execute queued tunnel commands ─────────────────────────────────────
 * These drive the same RS485 bus du_task polls on, so each exchange takes the
 * bus lock — per command, so du_task gets the bus between two of them. The
 * mqtt_publish() of the reply is deliberately OUTSIDE the lock: a publish is a
 * modem round-trip that can block for a second or more and does not need the
 * bus, so holding it there would stall dispenser polling for no reason.
 *
 * At most TUNNEL_BATCH commands per call — each costs ~2 s, the pilot's quiet
 * window after the reply — so a burst (a whole-station price change) cannot
 * keep the main task off its own duties, and a panel unlock arriving behind
 * it waits seconds, not half a minute. This runs every ~1 s and after every
 * poll step, so the rest follow straight away. */
#define TUNNEL_BATCH 4
static void execute_pending(void) {
    for (int n = 0; n < TUNNEL_BATCH; n++) {
        RspTunnelCmd_t c;
        if (!rsp_tunnel_pop(&c)) return;
        /* One command can wait RS485_BUS_WAIT_MAIN_MS (9 s) for the bus and then
         * hold it up to 3 x RS485_RECV_TIMEOUT_MS, so feed the 120 s task WDT
         * between commands. Guarded because this runs on whichever task called
         * poll_incoming_mqtt. */
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        if (c.unit >= DU_MAX_DISPENSERS) continue;              /* belt only */

        const bool  kf   = (c.kind == RSP_TUNNEL_KFACTOR);
        const char *what = kf ? "kfactor" : "rsp";
        const char *out  = kf ? t_kf_out[c.unit] : t_rsp_out[c.unit];
        const bool  bs   = (g_cfg.du_make[c.unit] == DU_MAKE_BLUESKY);
        /* Static, not stack: the deepest call chain on the main task
         * (poll_incoming_mqtt -> execute_pending), one command at a time. */
        static char reply[RSP_TUNNEL_REPLY_TEXT_MAX];
        snprintf(reply, sizeof(reply), "404");

        if (!rs485_bus_lock(RS485_BUS_WAIT_MAIN_MS)) {
            ESP_LOGW(TAG, "%s tunnel (%s): RS485 bus busy for %d ms — nothing sent, 404",
                     what, g_hw_du[c.unit], RS485_BUS_WAIT_MAIN_MS);
            mqtt_publish(out, reply);
            return;                 /* a poll round has the bus; the rest wait */
        }
        if (bs) rs485_bus_set_bluesky();
        else    rs485_bus_set_yenen();
        rsp_tunnel_exchange(c.frame, c.len, reply, sizeof(reply));
        /* Any tunnel frame may have been a k-factor write: the next nozzle
         * messages re-read it instead of carrying the cached value. */
        if (bs) du_kfactor_mark_stale();
        rs485_bus_set_8n1(9600);    /* leave the bus as radar/genset expect */
        rs485_bus_unlock();

        /* The /out contract: the pump's reply as received, or 404. */
        mqtt_publish(out, reply);
    }
}

/* ── Post-OTA check ───────────────────────────────────────────────────── */
/* True while this boot runs an OTA-delivered image that has not confirmed
 * itself yet. False on a serial-flashed image and on one already confirmed. */
static bool ota_is_pending_verify(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t   state;
    return esp_ota_get_state_partition(running, &state) == ESP_OK
        && state == ESP_OTA_IMG_PENDING_VERIFY;
}

/* ── Did the previous update stay? ──────────────────────────────────────────
 * The bootloader writes ABORTED into the other slot's otadata entry when an
 * image reached PENDING_VERIFY and reset without confirming itself — that is
 * the only trace a silent rollback leaves. Read once at boot, surfaced to the
 * cloud as fota_failed with reason fota_reverted (cng-cpil reports it too),
 * and used to flip the remembered URL to REVERTED so a retained copy cannot
 * loop the download.
 * esp_ota_get_last_invalid_partition() keeps answering until that slot is
 * written again, so the URL state transition is what makes this fire once. */
static char s_reverted_image[36] = "";

static void note_reverted_image_at_boot(void) {
    const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
    if (!bad) return;
    uint32_t fp = 0;
    if (nvs_fota_url_get(&fp) != FOTA_URL_APPLIED) return;      /* already noted, or nothing recorded */
    nvs_fota_url_set(fp, FOTA_URL_REVERTED);
    esp_app_desc_t desc;
    if (esp_ota_get_partition_description(bad, &desc) == ESP_OK && desc.project_name[0])
        snprintf(s_reverted_image, sizeof(s_reverted_image), "%.*s",
                 (int)sizeof(desc.project_name), desc.project_name);
    else
        snprintf(s_reverted_image, sizeof(s_reverted_image), "%s", bad->label);
    ESP_LOGE(TAG, "the last FOTA image (%s, slot %s) did not confirm itself and was ROLLED BACK "
                  "— its URL is now refused unless re-sent with force", s_reverted_image, bad->label);
}

/* The final fota_successful is the cloud's only proof that an update took.
 * The image confirms itself on its first poll cycle whether or not the link is
 * up (a weak site must not roll a good update back), so the answer can find no
 * broker at that moment — it is then kept and re-sent each cycle until a
 * publish is confirmed. It is also owed when ota_confirm_before_restart()
 * confirms the image instead, and the restart right after it would swallow
 * it: a setconfig pushed straight after an update, and the cloud never hears
 * that the update took. So the debt is kept in RTC memory too, which survives
 * esp_restart() (not a power cut), and the next boot of the same image —
 * confirmed by then — pays it (app_main). Same as alds-sr. */
#define FOTA_OWED_MAGIC 0x464F5441u   /* "FOTA" */
RTC_NOINIT_ATTR static uint32_t s_fota_owed;
static bool s_fota_completed_unsent = false;

static void fota_completed_owe(bool owed) {
    s_fota_completed_unsent = owed;
    s_fota_owed             = owed ? FOTA_OWED_MAGIC : 0;
}

/* Every DELIBERATE restart this firmware performs on a cloud command
 * (setconfig save, restart, erase_config, factory_reset, a FOTA it just applied) goes
 * through here first. An OTA image that has not confirmed itself yet is
 * thrown away by the bootloader on the next reset — so a setconfig pushed the
 * moment a freshly swapped gateway announced "unconfigured", before its first
 * poll cycle had run, would have rebooted it straight back into the OTHER
 * product's firmware. Receiving, parsing and acting on a cloud command is at
 * least as strong a proof of a working image as the poll cycle that normally
 * confirms it, so confirm here rather than let the restart undo the update. */
static void ota_confirm_before_restart(const char *why) {
    if (!ota_is_pending_verify()) return;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not confirm the pending OTA image before a deliberate restart "
                      "(%s): %s — that restart will roll the update back", why, esp_err_to_name(err));
        return;
    }
    fota_completed_owe(true);
    ESP_LOGW(TAG, "confirmed the pending OTA image before a deliberate restart (%s) — "
                  "a restart of an unconfirmed image would roll it back", why);
}

static void publish_fota_completed(void) {
    if (!s_fota_completed_unsent || !mqtt_is_connected()) return;
    char msg[192];
    snprintf(msg, sizeof(msg),
             "{\"HW_ID\":\"%s\",\"status\":\"fota_successful\",\"reason\":\"update_confirmed\","
             "\"fw\":\"%s\",\"build\":\"%.32s\"}", g_imei, FW_VERSION, running_build());
    if (mqtt_publish(t_logs, msg)) fota_completed_owe(false);
}

/* A final fota_failed that found no broker: why a download failed (when the
 * reconnect after it failed too), or a rollback noticed at boot. Kept, like
 * the success above, until a publish goes through. RAM only: a restart before
 * then loses it, and the cloud's own timeout is what answers that push. */
static char s_fota_failed_unsent[200] = "";

static void fota_failed_owe(const char *json) {
    snprintf(s_fota_failed_unsent, sizeof(s_fota_failed_unsent), "%s", json ? json : "");
}

static void publish_fota_failed_owed(void) {
    if (!s_fota_failed_unsent[0] || !mqtt_is_connected()) return;
    if (mqtt_publish(t_logs, s_fota_failed_unsent)) s_fota_failed_unsent[0] = '\0';
}

/* A new push supersedes every answer still owed to earlier ones (handle_msg). */
static void fota_owed_answers_drop(void) {
    fota_completed_owe(false);
    s_fota_failed_unsent[0] = '\0';
}

/* Confirms a freshly flashed image. True when there is nothing left to do —
 * the image was not pending, or the mark took — so the caller stops calling.
 * A failed mark is retried next cycle rather than announced: fota_successful
 * for an image the bootloader would still roll back is a lie (alds-sr). */
static bool check_post_ota(void) {
    if (!ota_is_pending_verify()) return true;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not confirm the new image (%s) — retrying next cycle",
                 esp_err_to_name(err));
        return false;
    }
    fota_completed_owe(true);
    publish_fota_completed();
    return true;
}

/* ── Publish dispenser data ────────────────────────────────────────
 * Acquisition moved to du_task on core 1 — see du_task.h for the measurement
 * that forced it (290 s with zero dispenser reads while the recovery ladder
 * held this task). What remains here is publishing: drain what the acquisition
 * task produced and push it to the broker.
 *
 * ORDER IS THE CONTRACT. Flash is drained BEFORE RAM, and du_emit() guarantees
 * flash always holds records strictly older than RAM (it moves the whole RAM
 * queue to flash on the first spill, and sends everything to flash while any
 * remains). Together those two rules make the published stream globally
 * oldest-first across an outage.
 *
 * This matters because the backend reconstructs each sale by differencing
 * consecutive totalizer readings ordered by TS. Replay a newer reading before
 * an older one and the difference goes negative, which surfaces as a phantom
 * transaction rather than as an obvious error. If either rule is ever changed,
 * they must be changed together.
 *
 * The RAM queue is NOT drained while flash still has records: doing so would
 * publish a newer record mid-backlog and break exactly that ordering.
 *
 * The flash backlog is paced at STORE_FLUSH_PER_CYCLE per cycle rather than
 * emptied in one go: a backlog dumped all at once would hammer a link that has
 * only just come back, and every publish is a modem AT round-trip that blocks
 * this task. Live data keeps flowing while the backlog clears behind it.
 *
 * Records are popped only AFTER the publish is confirmed. A reboot between the
 * two re-sends rather than loses — at-least-once by design, because the cloud
 * can dedupe on HW_ID + TS + totalizer but cannot invent a sale that was
 * dropped. */
/* ── Replay guard: is this exactly one well-formed JSON object? ────────────
 * Records the LIVE builders produce are always well-formed (they fall back to
 * a 404 shape on any truncation). A record replayed from FLASH is not: littlefs
 * is power-safe for its own metadata, but a payload appended in the instant
 * before a power cut can commit as a truncated tail ("{...\"TS\":\"2026-" with
 * no close) or, if a second record was appended after reboot onto the same
 * unterminated line, as two objects merged into one ("{...}{...}").
 *
 * Publishing either straight to the broker sends malformed JSON to the cloud
 * parser on the replay path — precisely where a single bad record can stall a
 * backend batch job. So every FLASH record is checked here before it is put on
 * the wire; a record that fails is discarded rather than published. It is at
 * most one power-loss-corrupted tail record, and dropping it is strictly safer
 * than emitting a payload the server cannot parse.
 *
 * String-aware (braces or quotes inside a JSON string do not count) so it never
 * false-rejects a real payload. Rejects: not starting with '{', an unterminated
 * string, an unclosed object, or anything but whitespace after the top-level
 * object closes. */
static bool payload_is_one_json_object(const char *s) {
    if (!s) return false;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    if (*s != '{') return false;
    int  depth  = 0;
    bool in_str = false, esc = false, closed = false;
    for (; *s; s++) {
        char c = *s;
        if (closed) {                         /* top-level object already ended */
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
            return false;                     /* junk / a second object follows  */
        }
        if (in_str) {
            if (esc)            esc = false;
            else if (c == '\\') esc = true;
            else if (c == '"')  in_str = false;
            continue;
        }
        if      (c == '"') in_str = true;
        else if (c == '{') depth++;
        else if (c == '}') {
            if (--depth == 0)      closed = true;
            else if (depth <  0)   return false;
        }
    }
    return closed && !in_str;                  /* fully closed, no dangling string */
}

static void publish_dispenser_queue(void) {
    static char payload[DU_PAYLOAD_MAX];

    /* Backlog first. The per-cycle allowance is the number of payloads one
     * poll round produces PLUS STORE_FLUSH_PER_CYCLE, so the drain always gains
     * on the backlog no matter how many nozzles are configured — a flat count
     * would fall permanently behind on a large site and the queue would grow
     * without bound. A wall-clock budget caps it either way, so a long backlog
     * clears over several cycles rather than stalling radar/genset/DI in one. */
    int per_round  = g_cfg.du_count * 2;
    int flush_max  = per_round + STORE_FLUSH_PER_CYCLE;
    int64_t deadline = esp_timer_get_time() + (int64_t)STORE_FLUSH_BUDGET_MS * 1000;

    for (int i = 0; i < flush_max && store_ready(); i++) {
        uint32_t token = 0;
        if (esp_timer_get_time() >= deadline) break;
        if (!store_peek(payload, sizeof(payload), &token)) break;

        /* Never put a malformed replayed record on the wire — see
         * payload_is_one_json_object(). Discard it (it is at most one
         * power-loss-corrupted tail) and keep draining the valid records
         * behind it rather than stalling the whole backlog on one bad line. */
        if (!payload_is_one_json_object(payload)) {
            ESP_LOGE(TAG, "replay: dropping a malformed flash record (corrupted by a "
                          "power cut mid-append?) — not published");
            store_pop(token);
            continue;
        }

        if (!mqtt_publish(t_data, payload)) {
            g_diag.pub_fail++;
            break;              /* link still down — leave it queued for next cycle */
        }
        g_diag.pub_ok++;
        /* Discards ONLY if this is still the record we published. The store
         * lock is released across the publish above, so du_task can trim the
         * queue in that window; popping blind would discard an unpublished
         * record. A refused pop just means the head changed — stop draining
         * and pick it up next cycle. */
        if (!store_pop(token)) break;
        esp_task_wdt_reset();
        poll_incoming_mqtt();
    }

    /* Live data — ONLY once the flash backlog is fully drained. Publishing a
     * RAM record while older records remain on flash would invert the order
     * (see the note above). While a backlog exists, du_emit() is routing new
     * payloads to flash anyway, so the RAM queue stays empty and this loop is
     * a no-op until the backlog clears.
     *
     * Bounded by the depth sampled up front so a producer running faster than
     * this loop cannot keep it spinning and starve the rest of the cycle. */
    if (store_ready() && store_has_records()) return;

    int live = du_task_pending();
    while (live-- > 0) {
        uint32_t token = 0;
        /* Peek, publish, then release — the record stays in the queue for the
         * whole publish. On failure there is literally nothing to do: it is
         * still at the head, in order, and gets retried next cycle. If the
         * outage runs long enough to fill RAM, du_task spills it to flash with
         * its neighbours, still in order. See du_task.h for why holding it
         * outside the queue could neither be made lossless nor kept ordered. */
        if (!du_task_peek(payload, sizeof(payload), &token)) break;
        if (!mqtt_publish(t_data, payload)) {
            g_diag.pub_fail++;
            break;
        }
        g_diag.pub_ok++;
        if (!du_task_release(token)) break;   /* queue moved under us — stop here */
        esp_task_wdt_reset();
        poll_incoming_mqtt();
    }
}

/* ── Poll radar ───────────────────────────────────────────────────────── */
/* Returns this tank's health (read valid AND publish accepted) so the caller
 * can AND it across all tanks — same rule du_task applies to du_ok. */
/* An enum rather than a bool selector, so a third radar type can be added
 * without "the third state of a bool" silently reading the wrong g_hw_* array. */
typedef enum {
    RADAR_T_ZTECH,
    RADAR_T_PANTECH
} RadarType_t;

/* The Modbus slave a radar tank is polled at: its HW_ID suffix when numeric,
 * else the field-convention fallback (Ztech from ZTECH_SLAVE_BASE, Pantech
 * after the Ztech tanks). ONE resolver — radar_slave_for() — shared with
 * validate_bus_addresses() so the plan the validator accepted is the plan the
 * bus is asked for. */
static uint8_t get_radar_slave(RadarType_t type, int tank_idx) {
    switch (type) {
        case RADAR_T_PANTECH:
            return radar_slave_for(g_cfg.sfx_pantech[tank_idx], ZTECH_SLAVE_BASE + g_cfg.ztech_count + tank_idx);
        case RADAR_T_ZTECH:
        default:
            return radar_slave_for(g_cfg.sfx_ztech[tank_idx], ZTECH_SLAVE_BASE + tank_idx);
    }
}

static bool poll_one_tank(int tank_idx, RadarType_t type, const char *ts) {
    /* Local, not static: 576 bytes easily fits this shallow frame, and static
     * storage shared between the per-type calls in the same poll cycle is fine
     * only because they're called sequentially today — local removes that
     * assumption at zero runtime cost. */
    uint8_t rx[64];
    char    payload[512];
    int  rx_len = 0;
    bool ok = false;
    uint8_t slave = get_radar_slave(type, tank_idx);

    /* Bus lock covers the mode set and all retries, then is released before the
     * payload is built and published — mqtt_publish() can block for a second or
     * more and does not need the bus. A lock timeout leaves ok=false and the
     * tank publishes "404" for this cycle, the same as a dead sensor. */
    if (rs485_bus_lock(RS485_BUS_WAIT_MAIN_MS)) {
        rs485_bus_set_8n1(9600);
        for (int attempt = 1; attempt <= RADAR_READ_RETRIES && !ok; attempt++) {
            if (attempt > 1) vTaskDelay(pdMS_TO_TICKS(MODBUS_RETRY_GAP_MS));
            switch (type) {
                case RADAR_T_PANTECH: ok = radar_pantech_read(slave, tank_idx, rx, &rx_len); break;
                case RADAR_T_ZTECH:
                default:              ok = radar_ztech_read  (slave, tank_idx, rx, &rx_len); break;
            }
        }
        rs485_bus_unlock();
    } else {
        ESP_LOGW(TAG, "radar tank%d (slave %u): RS485 bus busy — skipped", tank_idx + 1, slave);
        rx_len = 0;
    }
    const char *hw = (type == RADAR_T_PANTECH) ? g_hw_pantech[tank_idx] : g_hw_ztech[tank_idx];

    if (!ok || rx_len == 0) {
        snprintf(payload, sizeof(payload),
                 "{\"HW_ID\":\"%s\",\"tanks\":{\"%d\":{\"data\":\"404\"}},\"TS\":\"%s\"}",
                 hw, tank_idx + 1, ts);
    } else {
        char hex[256] = {0};
        int  pos = 0;
        /* rx_len is bounded to 64 by radar_xfer (resp_max=64); worst-case hex
         * output is 64*"xx," ≈ 191 bytes, well under hex[256], so pos can't
         * reach the buffer end and sizeof(hex)-pos can't underflow. */
        for (int i = 0; i < rx_len; i++) {
            if (i > 0) hex[pos++] = ',';
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x", rx[i]);
        }
        snprintf(payload, sizeof(payload),
                 "{\"HW_ID\":\"%s\",\"tanks\":{\"%d\":{\"data\":\"%s\"}},\"TS\":\"%s\"}",
                 hw, tank_idx + 1, hex, ts);
    }
    bool pub = mqtt_publish(t_data, payload);
    if (pub) g_diag.pub_ok++; else g_diag.pub_fail++;
    return ok && pub;
}

/* Tank selection for poll_radar(): bit i = Ztech tank i, bit (RADAR_MAX + i) =
 * Pantech tank i. RADAR_MAX is 4, so eight bits cover every tank. */
#define RADAR_BIT_ZTECH(i)   (1u << (i))
#define RADAR_BIT_PANTECH(i) (1u << (RADAR_MAX + (i)))
_Static_assert(2 * RADAR_MAX <= 8, "radar tank mask must fit a uint8_t");

static uint8_t radar_all_mask(void) {
    uint8_t m = 0;
    for (int i = 0; i < g_cfg.ztech_count;   i++) m |= RADAR_BIT_ZTECH(i);
    for (int i = 0; i < g_cfg.pantech_count; i++) m |= RADAR_BIT_PANTECH(i);
    return m;
}

/* Polls the tanks selected by `mask` and returns the mask of those that did
 * NOT yield a published reading — the set the fast re-poll (run_poll_cycle)
 * tries again in EQUIP_RETRY_INTERVAL_S. */
static uint8_t poll_radar(const char *ts, uint8_t mask) {
    /* Mode is now set per tank inside poll_one_tank, under the bus lock —
     * setting it out here would race with du_task reconfiguring the port for
     * a dispenser read. */
    /* poll_incoming_mqtt() after every tank, not just at the end of the round.
     * A tank that has to time out costs ~4 s (RADAR_READ_RETRIES x the 1 s reply
     * window, 500 ms apart — config.h), and with
     * several configured this loop could hold an inbound command — a panel
     * unlock in particular — for seconds. The lock is the one command where
     * latency is felt by a person standing at the box, so it gets serviced at
     * every opportunity rather than once per cycle. */
    uint8_t failed = 0;
    for (int i = 0; i < g_cfg.ztech_count; i++) {
        if (!(mask & RADAR_BIT_ZTECH(i))) continue;
        if (!poll_one_tank(i, RADAR_T_ZTECH, ts)) failed |= RADAR_BIT_ZTECH(i);
        poll_incoming_mqtt(); execute_pending();
    }
    for (int i = 0; i < g_cfg.pantech_count; i++) {
        if (!(mask & RADAR_BIT_PANTECH(i))) continue;
        if (!poll_one_tank(i, RADAR_T_PANTECH, ts)) failed |= RADAR_BIT_PANTECH(i);
        poll_incoming_mqtt(); execute_pending();
    }
    /* AND across every tank polled this call rather than last-one-wins — a
     * dead tank hidden behind a healthy one would otherwise report ok. A
     * re-poll of only the failed tanks that succeeds flips this back to true,
     * which is right: every tank has now been read this interval. */
    g_diag.radar_ok = (failed == 0);
    return failed;
}

/* ── Poll genset ──────────────────────────────────────────────────────── */
/* Returns true when every read the model defines succeeded AND every message
 * was published — false is what the fast re-poll acts on. */
static bool poll_genset(const char *ts) {
    static GensetData_t gd;
    /* Raw-hex genset payloads are small — the largest frame (SGC120: 161 bytes
     * → 482 hex chars) is well under 600 bytes — but the buffer is left at 4096
     * so the builder's own ceiling check (MQTT_MODEM_PAYLOAD_SAFE_MAX), not the
     * buffer, is what bounds the payload, and a future wider read block can't
     * silently truncate here. */
    static char payload[4096];
    /* Up to GENSET_READ_RETRIES attempts before giving up this cycle.
     * genset_poll() memsets gd on entry, so a failed attempt leaves it cleared
     * and a later success fully repopulates it. It returns true only when EVERY
     * read the model defines succeeded; a partial result still publishes what it
     * got, with "404" for the read that failed.
     *
     * Genset is the one user that changes BAUD rather than just parity, so the
     * bus must be held across the whole of one attempt — a dispenser read
     * landing mid-sequence would run at the DG's baud and return garbage.
     *
     * The lock is taken PER ATTEMPT, not across all three. Holding it for the
     * full retry burst meant an unreachable DG — which is normal, a DG is often
     * powered down — pinned the bus for ~3.3 s every 5 minutes, long enough to
     * push a dispenser round past its own wait. Each attempt re-asserts the
     * baud under the lock, so nothing about correctness changes; the worst-case
     * hold just drops to one genset_poll (~3.1 s for a silent Mahindra's three
     * reads, ~2 s for the two-read models, at the 1 s reply window in config.h).
     * A lock timeout mid-burst
     * abandons the remaining attempts and publishes 404 for this cycle, exactly
     * as a dead DG would. */
    bool ok = false;
    bool got_bus = false;
    /* Clear up front, not just inside genset_poll(). gd is static, so if every
     * attempt below fails to get the bus, genset_poll() never runs and the
     * builder would serialise the PREVIOUS cycle's registers under this
     * cycle's timestamp — stale DG data presented as a live reading. Clearing
     * here makes a skipped poll publish 404s, which is what it is — ALL of them:
     * genset_expect() sets which messages this model sends, which the cleared
     * struct would otherwise say is none (only the registers 404 went out). */
    memset(&gd, 0, sizeof(gd));
    genset_expect(g_cfg.genset_model, &gd);
    for (int attempt = 1; attempt <= GENSET_READ_RETRIES && !ok; attempt++) {
        if (attempt > 1) vTaskDelay(pdMS_TO_TICKS(MODBUS_RETRY_GAP_MS));
        if (!rs485_bus_lock(RS485_BUS_WAIT_MAIN_MS)) {
            ESP_LOGW(TAG, "genset: RS485 bus busy — attempt %d/%d skipped",
                     attempt, GENSET_READ_RETRIES);
            continue;
        }
        got_bus = true;
        rs485_bus_set_8n1(g_cfg.genset_baud);
        ok = genset_poll(g_cfg.genset_slave, g_cfg.genset_model, &gd);
        /* Leave the port at the common 8N1/9600 rather than the DG's baud, so
         * the resting state is the one most drivers expect. Every user still
         * sets its own mode under the lock, so this is hygiene, not a
         * correctness requirement. */
        rs485_bus_set_8n1(9600);
        rs485_bus_unlock();
    }
    if (!got_bus)
        ESP_LOGW(TAG, "genset: RS485 bus busy for every attempt — skipped this cycle");

    /* Message 1 of up to 3: the measurement block. Each Modbus read is published
     * as its own message per the DG integration spec, in the spec's order, all
     * under the genset HW_ID and sharing this cycle's timestamp. */
    genset_build_registers_payload(&gd, g_hw_genset, ts, payload, sizeof(payload));
    bool pub = mqtt_publish(t_data, payload);
    if (pub) g_diag.pub_ok++; else g_diag.pub_fail++;

    /* Message 2: the alarm registers, Mahindra GC1211/GC1111 only. has_alarms
     * is set from the model table even when the reads failed, so a dead DG
     * still reports a "404" Alarms message rather than silently sending
     * nothing — same rule as the status block below. */
    if (gd.has_alarms) {
        genset_build_alarms_payload(&gd, g_hw_genset, ts, payload, sizeof(payload));
        bool pub_al = mqtt_publish(t_data, payload);
        if (pub_al) g_diag.pub_ok++; else g_diag.pub_fail++;
        pub = pub && pub_al;
    }

    /* Message 3: the DG status block, for every model that defines one (all but
     * the SGC 120, whose single overall read already covers it). has_status is
     * set from the model table even when the reads failed, so a dead DG still
     * reports a "404" status rather than silently sending nothing. */
    if (gd.has_status) {
        genset_build_status_payload(&gd, g_hw_genset, ts, payload, sizeof(payload));
        bool pub_st = mqtt_publish(t_data, payload);
        if (pub_st) g_diag.pub_ok++; else g_diag.pub_fail++;
        pub = pub && pub_st;
    }

    g_diag.genset_ok = ok && pub;
    return ok && pub;
}

/* ── Poll GPS ─────────────────────────────────────────────────────────────
 * Bare IMEI as HW_ID, no suffix — GPS describes the gateway's own fixed
 * location, not a specific piece of equipment behind it (same reasoning as
 * status/diagnostics/sysmon). Not on the RS485 bus, so no mode-switch calls
 * like poll_genset()'s rs485_bus_set_8n1(). */
static bool poll_gps(const char *ts) {
    static GpsFix_t fix;
    static char payload[192];  /* {"HW_ID":..,"gps":{"lat":..,"lon":..,"hdop":..,"nsat":..,"spd":..},"TS":..} ~135 bytes worst case */
    bool ok = gps_poll(&fix);
    gps_build_payload(&fix, g_imei, ts, payload, sizeof(payload));
    bool pub = mqtt_publish(t_data, payload);
    if (pub) g_diag.pub_ok++; else g_diag.pub_fail++;
    g_diag.gps_ok = ok && pub;
    return ok && pub;
}

/* ── Slow-cadence scheduling with a fast re-poll ──────────────────────────
 * One of these per sampled subsystem (radar, genset, GPS). `full_due` fires
 * on the regular cadence, anchored to the last FULL poll so it never drifts;
 * `retry_due` fires EQUIP_RETRY_INTERVAL_S after a poll that failed, up to
 * EQUIP_RETRY_MAX times, and is cleared by the next full poll. See the
 * EQUIP_RETRY_* note in config.h for why a 10-minute sample deserves this. */
typedef struct {
    int64_t last_full_us;   /* 0 = never polled: due on the first cycle       */
    int64_t last_try_us;    /* any poll, full or retry — paces the retries    */
    int     retries;        /* retries spent since the last full poll         */
    bool    failed;         /* the last poll (full or retry) did not succeed  */
} SlowPoll_t;

/* Decide whether to poll now. Returns 0 = not due, 1 = full poll, 2 = retry. */
static int slow_poll_due(SlowPoll_t *p, int64_t now_us, int interval_s) {
    if (p->last_full_us == 0 || now_us - p->last_full_us >= (int64_t)interval_s * 1000000LL)
        return 1;
    if (p->failed && p->retries < EQUIP_RETRY_MAX &&
        now_us - p->last_try_us >= (int64_t)EQUIP_RETRY_INTERVAL_S * 1000000LL)
        return 2;
    return 0;
}

static void slow_poll_note(SlowPoll_t *p, int kind, int64_t now_us, bool ok, const char *what) {
    if (kind == 1) { p->last_full_us = now_us; p->retries = 0; }
    else           { p->retries++; }
    p->last_try_us = now_us;
    p->failed      = !ok;
    if (!ok) {
        if (p->retries < EQUIP_RETRY_MAX)
            ESP_LOGW(TAG, "%s poll incomplete — re-trying the failed unit(s) in %d s (%d/%d)",
                     what, EQUIP_RETRY_INTERVAL_S, p->retries + 1, EQUIP_RETRY_MAX);
        else
            ESP_LOGW(TAG, "%s poll still incomplete after %d re-tries — next attempt on the "
                          "regular cadence", what, EQUIP_RETRY_MAX);
    }
}

/* ── MQTT health check with layered recovery ─────────────────────────────
 * L1: plain MQTT reconnect (broker/session hiccup)
 * L2: rebuild the PDP/network context first (stale bearer, silent LTE drop)
 * L3: power-cycle the modem via PWRKEY (modem itself is wedged)
 * L4: nothing worked — restart
 * ──────────────────────────────────────────────────────────────────────── */
static void mqtt_reconnect_if_needed(void) {
    if (mqtt_is_connected()) return;
    ESP_LOGW(TAG, "MQTT offline — reconnecting");
    /* WDT deliberately stays ARMED through the whole ladder. This function
     * used to delete it, leaving up to ~25 minutes (5 QMTOPEN retries + 3
     * network rebuilds with 120 s registration waits + 3 modem power-cycles)
     * completely unmonitored — the longest blind window in the firmware, and
     * it ran on every connectivity blip. Every blocking call below feeds the
     * WDT internally (modem_at/modem_read_until reset it on each read
     * iteration); the explicit resets here cover the bare vTaskDelay backoffs
     * and modem_hard_reset()'s PWRKEY delays. */

    /* Recovery-ladder pacing state. These are STATIC so the ladder spreads its
     * retries across calls instead of blocking the main task through the whole
     * escalation (it used to burn ~175 s per invocation). The consequence of
     * static budgets is that EVERY level must be re-armed the moment the link
     * comes back — an exhausted counter that is never reset would silently
     * disable that level for the rest of the boot. LADDER_RESET() below is the
     * single place that does it, and every success path calls it. */
    static int      l1_failures = 0, l2_failures = 0, l3_failures = 0;
    static int64_t  next_l1_us  = 0, next_l2_us  = 0, next_l3_us  = 0;
    /* Unbroken run of "bearer up, must be the broker" verdicts — see
     * MQTT_BEARER_DISTRUST_AFTER in config.h. Reset by any verdict to the
     * contrary and by LADDER_RESET() on every success, so it only ever counts a
     * continuous outage. */
    static unsigned outage_checks = 0;
    int64_t now = esp_timer_get_time();

    /* Exponential backoff carrying up to 1 s of jitter. A whole fleet that lost
     * the same broker at the same instant must not retry in lockstep and
     * stampede it the moment it returns. */
    #define LADDER_BACKOFF_US(n) ((int64_t)1000000LL * (1 << (n)) + (int64_t)(esp_random() % 1000000))
    #define LADDER_RESET() do { l1_failures = l2_failures = l3_failures = 0; \
                                next_l1_us  = next_l2_us  = next_l3_us  = 0; \
                                outage_checks = 0;                           \
                                g_diag.bearer_distrust = 0; } while (0)

    if (now < next_l1_us) return;

    if (l1_failures < MQTT_RECONNECT_RETRIES) {
        forensics_mark(FSLOT_MAIN, F_RECOVER, 1);
        if (mqtt_connect()) {
            subscribe_all();
            esp_task_wdt_reset();
            LADDER_RESET();
            return;
        }
        l1_failures++;
        g_diag.recov_l1++;          /* cumulative — see DiagState_t */
        /* Re-sample the clock AFTER the blocking mqtt_connect(): `now` was taken
         * at function entry, and anchoring the backoff to it would place the
         * deadline in the past whenever the attempt itself outran the backoff,
         * collapsing the exponential spacing and the fleet-desync jitter into
         * back-to-back retries. Same for L2/L3 below. */
        next_l1_us = esp_timer_get_time() + LADDER_BACKOFF_US(l1_failures);
        esp_task_wdt_reset();
        return;
    }

    /* ── Broker reconnect failed — decide WHY before escalating ───────────────
     * The question is "can this device open a socket at all", and it is asked
     * with network_data_bearer_up() — registration AND an active PDP context —
     * NOT with registration alone. A registered modem whose context was torn
     * down (a network-side pdpdeact, or a pulled SIM) cannot open a socket
     * while looking perfectly healthy to CEREG, so gating on registration
     * parked the ladder here forever and never ran the QIACT=1 that fixes it.
     * See network.h.
     *
     * If the bearer IS up, this is a broker/server outage, NOT a device fault.
     * Rebuilding the PDP context (L2), power-cycling
     * the modem (L3) or rebooting (L4) cannot reach a broker that is down, and
     * running that escalation every cycle for the length of the outage would
     * reboot-loop the whole fleet and thrash its modems and cell re-attach load
     * — exactly the synchronised spike config.h's maintenance-reboot note exists
     * to avoid. Stay online instead: du_task keeps buffering every dispenser
     * payload to flash and the next poll cycle retries the broker, flushing the
     * backlog the instant it returns (the same store-and-forward path a network
     * outage already uses). The check round-trips AT commands, so a modem that
     * has genuinely wedged (AT not answering) fails it and correctly falls
     * through to the hardware recovery + reboot below. */
    bool bearer_up = network_data_bearer_up();

    /* ── When the modem's own answer stops being credible ──────────────────
     * The verdict above rests on the baseband printing "+QIACT: 1,1", and a
     * baseband can keep printing that for a context that is dead. Believing it
     * indefinitely would park the ladder at L1 forever on exactly the fault
     * this rewrite exists to escape — one layer deeper, and harder to see.
     *
     * So after MQTT_BEARER_DISTRUST_AFTER unbroken outage verdicts, stop
     * believing it and let the escalation run. L2 rebuilds the context, which
     * is the correct treatment for a stale bearer and a harmless no-op for a
     * genuine broker outage (it reconnects, fails to reach the broker, and
     * lands back here). */
    if (bearer_up && outage_checks >= MQTT_BEARER_DISTRUST_AFTER) {
        ESP_LOGE(TAG, "bearer has claimed UP for %u consecutive checks with the "
                      "broker still unreachable — rebuilding the context rather "
                      "than believing it", outage_checks);
        outage_checks = 0;
        bearer_up     = false;      /* fall through to L2 */
    }

    if (bearer_up) {
        outage_checks++;
        g_diag.bearer_distrust = outage_checks;
        ESP_LOGW(TAG, "broker unreachable but the data bearer is up — server/broker "
                      "outage; staying online, buffering to flash, retry in %d s "
                      "(check %u/%d)",
                 MQTT_BROKER_OUTAGE_RETRY_S, outage_checks, MQTT_BEARER_DISTRUST_AFTER);
        /* Re-arm L1 for the NEXT probe. Without this the static L1 budget stays
         * exhausted, the bearer check keeps returning early, and mqtt_connect()
         * is never called again — the device would buffer to flash forever and
         * never notice the broker coming back, which is the exact failure this
         * whole ladder exists to prevent. L4's reboot is unreachable from here
         * by design (a healthy network must not reboot the gateway), so this
         * re-arm is the ONLY thing that reconnects us after a broker outage.
         * Paced at a fixed interval rather than the geometric backoff: the
         * outage may last hours and each probe is a single cheap QMTOPEN. */
        l1_failures = 0;
        next_l1_us  = now + (int64_t)MQTT_BROKER_OUTAGE_RETRY_S * 1000000LL
                          + (int64_t)(esp_random() % 1000000);
        esp_task_wdt_reset();
        return;
    }

    /* Past this point the bearer is down (or no longer believed), so any run of
     * outage verdicts has been broken. */
    outage_checks = 0;

    if (now < next_l2_us) return;

    if (l2_failures < NET_BRINGUP_RETRIES) {
        forensics_mark(FSLOT_MAIN, F_RECOVER, 2);
        ESP_LOGW(TAG, "[RECOVER L2] no usable data bearer — rebuilding ...");
        if (network_init() && mqtt_connect()) {
            subscribe_all();
            ESP_LOGI(TAG, "[RECOVER L2] OK");
            esp_task_wdt_reset();
            LADDER_RESET();
            return;
        }
        l2_failures++;
        g_diag.recov_l2++;
        next_l2_us = esp_timer_get_time() + LADDER_BACKOFF_US(l2_failures);   /* re-sample after network_init() — see L1 */
        esp_task_wdt_reset();
        return;
    }

    if (now < next_l3_us) return;

    if (l3_failures < MODEM_PWRCYCLE_MAX) {
        forensics_mark(FSLOT_MAIN, F_RECOVER, 3);
        ESP_LOGE(TAG, "[RECOVER L3] Power-cycling modem ...");
        modem_hard_reset();
        esp_task_wdt_reset();
        if (network_init() && mqtt_connect()) {
            subscribe_all();
            ESP_LOGI(TAG, "[RECOVER L3] OK");
            esp_task_wdt_reset();
            LADDER_RESET();
            return;
        }
        l3_failures++;
        g_diag.recov_l3++;
        next_l3_us = esp_timer_get_time() + LADDER_BACKOFF_US(l3_failures);   /* re-sample after modem_hard_reset()+network_init() — see L1 */
        esp_task_wdt_reset();
        return;
    }

    /* Marked before the restart so last_stop names the exhausted ladder as
     * the cause. This is the one deliberate restart with no MQTT session to
     * announce itself, so the breadcrumb is the only record of it. */
    if (!restart_streak_allow("[RECOVER L4] all recovery levels failed")) {
        /* Re-arm the ladder from L1 with the outage pacing, so it keeps
         * probing (and keeps power-cycling the modem at L3) for as long as
         * the fault lasts, instead of returning here every call. */
        LADDER_RESET();
        next_l1_us = esp_timer_get_time() + (int64_t)MQTT_BROKER_OUTAGE_RETRY_S * 1000000LL;
        return;
    }
    forensics_mark(FSLOT_MAIN, F_RECOVER, 4);
    ESP_LOGE(TAG, "[RECOVER L4] All recovery levels failed — restarting");
    du_task_flush_to_store();
    esp_restart();

    #undef LADDER_BACKOFF_US
    #undef LADDER_RESET
}

/* Drain any pending incoming MQTT messages (setconfig, FOTA trigger, DU
 * commands) without blocking on a reconnect attempt — used to interleave
 * responsiveness checks into run_poll_cycle(), which can otherwise block
 * for 30+ seconds (BlueSky busy-retry waits, RS485 timeouts) with no
 * chance to notice an incoming publish until the cycle fully completes.
 *
 * Also executes any rsp/kfactor tunnel command handle_msg() just queued,
 * right here — not just once per 12 s cycle. This runs every ~1s (via
 * sleep_with_poll) as well as between every poll-cycle step, so a remote
 * command now gets serviced within about a second of arriving instead of
 * waiting up to a full poll cycle. execute_pending() sets its own RS485
 * mode before each transaction, so it's safe to call from any point in the
 * poll cycle regardless of which device's mode is currently active. */
static void poll_incoming_mqtt(void) {
    static char s_topic[TOPIC_MAX];
    static char s_payload[2048];
    /* Bounded drain. The WDT is fed each iteration, so a message flood (a
     * retained-message storm, a backend bug, a busy-slot loop that re-publishes)
     * would never trip the watchdog — it would instead starve the main task's
     * own duties (radar/genset/GPS/publish/RTC-resync) for as long as it lasts.
     * Cap the batch; anything still queued drains on the next call, and this
     * runs many times per cycle. Sized to the stash so a normal reconnect burst
     * still clears in a single pass. */
    int budget = 32;
    while (budget-- > 0 && mqtt_poll(s_topic, sizeof(s_topic), s_payload, sizeof(s_payload))) {
        handle_msg(s_topic, s_payload);
        /* A reconnect can deliver a burst of stashed/retained commands; feed the
         * WDT between them so a storm cannot trip the 120 s task watchdog. */
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
    }
    execute_pending();
}

/* ── Main poll cycle ──────────────────────────────────────────────────── */
static void run_poll_cycle(void) {
    /* Low-heap guard — reboot before heap exhaustion causes undefined behaviour */
    if (esp_get_free_heap_size() < 20480) {
        ESP_LOGE(TAG, "Critical heap (%lu B) — rebooting", (unsigned long)esp_get_free_heap_size());
        /* Spill the RAM ring first: these records are already-read totalizers
         * that no reboot can re-derive, and store_enqueue writes to flash
         * rather than the heap we are about to run out of. */
        du_task_flush_to_store();
        esp_restart();
    }

    int64_t loop_start = esp_timer_get_time();
    char ts[32];
    /* RTC read — instant and cannot fail on I/O, unlike the modem AT round-trip
     * this replaced. Re-read before each subsystem below so every payload
     * carries the time of its own reading rather than the cycle's start. */
    rtc_timestamp(ts, sizeof(ts));

    diag_pm_acquire();

    /* Dispensers first, unconditionally, on every cycle. They are the only
     * equipment on this bus whose data can be permanently degraded by being
     * late: the cloud reconstructs each sale by differencing consecutive
     * totalizer samples, so any sale that begins and ends between two reads is
     * merged into its neighbour instead of being recorded on its own. Radar and
     * genset are rate-limited below specifically to keep this path tight. */
    /* ── Acquisition-task self-heal ───────────────────────────────────────
     * If du_task_start() failed at boot (heap OOM is the realistic cause) the
     * gateway runs forever collecting NO pump data — it stays online, keeps
     * publishing radar/genset/DI/diagnostics, and looks healthy while the one
     * thing it exists to do is not happening. Nothing else in the firmware
     * recovers that: the low-heap guard only fires below 20 KB, and there is
     * no scheduled reboot.
     *
     * So retry it here. If the task is running this is a NULL check; if it is
     * not, and dispensers are configured, one allocation attempt per cycle
     * costs nothing and heals a transient boot-time shortage on its own. A
     * reboot would be the cruder alternative and risks a boot loop if the
     * failure is permanent. */
    if (g_cfg.du_count > 0
        && du_task_handle() == NULL) {
        static int retry_n = 0;
        bool recovered = du_task_start();
        /* Rate-limited so a permanent failure does not flood the console —
         * first attempt, then roughly every 15 minutes. */
        if (recovered && du_task_handle() != NULL) {
            ESP_LOGW(TAG, "dispenser acquisition task recovered after %d retries", retry_n);
            retry_n = 0;
        } else if (retry_n++ % DIAG_CYCLE_INTERVAL == 0) {
            ESP_LOGE(TAG, "dispenser acquisition task still NOT running "
                          "(retry %d) — NO pump data is being collected", retry_n);
        }
    }

    /* Dispensers are no longer polled here — du_task on core 1 does that, so it
     * keeps running while this task is inside the recovery ladder. All that
     * remains is publishing what it produced, plus any flash backlog from a
     * previous outage. */
    forensics_mark(FSLOT_MAIN, F_PUBLISH, 0);
    publish_dispenser_queue();
    esp_task_wdt_reset();
    poll_incoming_mqtt();

    /* Radar and genset run on wall-clock rate limits (RADAR_POLL_INTERVAL_S /
     * GENSET_POLL_INTERVAL_S) rather than the old "every N poll cycles".
     * Cycle length is not constant — it varies with nozzle count, retries and
     * publish latency — so a cycle count mapped to no predictable period: the
     * old RADAR_CYCLE_INTERVAL=2 was nominally 60 s but measured ~168 s in the
     * field. esp_timer_get_time() is a monotonic 64-bit microsecond counter
     * (~292,000 year range), so plain subtraction needs no wrap handling.
     * 0 = "never polled", so both fire on the first cycle after boot. */
    static SlowPoll_t s_radar  = {0};
    static SlowPoll_t s_genset = {0};
    static uint8_t    s_radar_fail_mask = 0;   /* tanks the fast re-poll still owes */

    if (g_cfg.ztech_count > 0 || g_cfg.pantech_count > 0) {
        int64_t now_us = esp_timer_get_time();
        int     kind   = slow_poll_due(&s_radar, now_us, RADAR_POLL_INTERVAL_S);
        if (kind) {
            /* A full poll reads every tank; a retry reads only the ones that
             * failed, so a healthy tank is not re-sampled for its neighbour. */
            uint8_t want = (kind == 1 || s_radar_fail_mask == 0) ? radar_all_mask() : s_radar_fail_mask;
            rtc_timestamp(ts, sizeof(ts));
            forensics_mark(FSLOT_MAIN, F_POLL_RADAR, 0);
            s_radar_fail_mask = poll_radar(ts, want);
            slow_poll_note(&s_radar, kind, now_us, s_radar_fail_mask == 0, "radar");
        }
    }

    esp_task_wdt_reset();
    poll_incoming_mqtt();

    if (g_cfg.genset_type == GENSET_SEDEMAC) {
        int64_t now_us = esp_timer_get_time();
        int     kind   = slow_poll_due(&s_genset, now_us, GENSET_POLL_INTERVAL_S);
        if (kind) {
            rtc_timestamp(ts, sizeof(ts));
            forensics_mark(FSLOT_MAIN, F_POLL_GENSET, 0);
            bool ok = poll_genset(ts);
            slow_poll_note(&s_genset, kind, now_us, ok, "genset");
        }
    }
    esp_task_wdt_reset();
    poll_incoming_mqtt();
    execute_pending();
    if (g_cfg.di_enable) {
        rtc_timestamp(ts, sizeof(ts));
        g_diag.di_ok = di_publish_buffer(t_data, g_hw_di, ts, g_cfg.di_enable);
    }
    esp_task_wdt_reset();
    poll_incoming_mqtt();

    /* GPS runs on a wall-clock interval (GPS_POLL_INTERVAL_S), like radar and
     * genset above — the gateway is a stationary asset whose position never
     * changes after install, so a fix every few minutes is ample. Gated solely
     * by gps_enable, which defaults to false: the one gate that keeps the whole
     * feature dead for every gateway not explicitly enabled from the cloud.
     * 0 = "never polled", so the first fix publishes on the first cycle after
     * boot and every GPS_POLL_INTERVAL_S thereafter regardless of cycle length. */
    static SlowPoll_t s_gps = {0};
    if (g_cfg.gps_enable) {
        int64_t now_us = esp_timer_get_time();
        int     kind   = slow_poll_due(&s_gps, now_us, GPS_POLL_INTERVAL_S);
        if (kind) {
            rtc_timestamp(ts, sizeof(ts));
            forensics_mark(FSLOT_MAIN, F_POLL_GPS, 0);
            bool ok = poll_gps(ts);
            slow_poll_note(&s_gps, kind, now_us, ok, "gps");
        }
    }
    esp_task_wdt_reset();
    poll_incoming_mqtt();

    /* Periodic RTC re-sync, deliberately the LAST modem-touching work in the
     * cycle so its ~1 s AT round-trip can never land between two dispenser
     * reads. Failure is non-fatal — the RTC keeps running on the last good
     * sync, which is why this replaced per-cycle modem time reads. */
    if (modem_time_resync_due()) {
        modem_time_sync();
        esp_task_wdt_reset();
    }

    diag_record_loop_time(esp_timer_get_time() - loop_start);

    /* One-minute CPU sample — the busiest-minute figure in the report. Reads
     * the FreeRTOS task table only: no I/O, no lock, microseconds. */
    {
        static int cpu_cycle = 0;
        if (++cpu_cycle >= CPU_LOAD_SAMPLE_CYCLES) {
            cpu_cycle = 0;
            diag_cpu_sample();
        }
    }

    if (++g_diag_cycle >= DIAG_CYCLE_INTERVAL) {
        rtc_timestamp(ts, sizeof(ts));
        /* One combined diagnostics report (diagnostics + system monitor merged
         * into diagnostics.c), on <base>/logs. diag_publish() reads the window
         * counters and clears them on a confirmed publish — the single
         * report-and-clear point. */
        forensics_mark(FSLOT_MAIN, F_DIAG, 0);
        diag_publish(g_imei, ts, t_logs);
        g_diag_cycle = 0;
    }

    /* Un-provisioned heartbeat: a gateway with no equipment configured (fresh
     * from flash, or just factory-reset) re-announces "unconfigured" every
     * UNCONFIGURED_HEARTBEAT_S so the cloud keeps flagging it until a setconfig
     * arrives — the boot-time announce is a single event that is easy to miss.
     * Same condition as the boot announce in app_main(). Seeded from the first
     * cycle (not 0) so it does NOT immediately duplicate that boot announce; the
     * first heartbeat lands one interval later. Reset to 0 once configured, so
     * re-entering the unconfigured state re-arms it. */
    {
        static int64_t last_unconf_us = 0;
        bool unconfigured = (g_cfg.du_count == 0
                             && g_cfg.ztech_count == 0 && g_cfg.pantech_count == 0
                             && g_cfg.genset_type == GENSET_OFF
                             && !g_cfg.di_enable);
        if (unconfigured) {
            int64_t now_us = esp_timer_get_time();
            if (last_unconf_us == 0) {
                last_unconf_us = now_us;   /* boot already announced — start the 3-min clock */
            } else if (now_us - last_unconf_us >= (int64_t)UNCONFIGURED_HEARTBEAT_S * 1000000LL) {
                last_unconf_us = now_us;
                char msg[128];
                snprintf(msg, sizeof(msg),
                         "{\"HW_ID\":\"%s\",\"status\":\"unconfigured\",\"fw\":\"%s\"}",
                         g_imei, FW_VERSION);
                mqtt_publish(t_logs, msg);
                ESP_LOGW(TAG, "still unconfigured — re-announced (awaiting setconfig)");
            }
        } else {
            last_unconf_us = 0;            /* configured — disarm */
        }
        /* Hold the LED on for as long as this state lasts. A successful
         * setconfig reboots, so in practice the release happens through the
         * boot path; this covers the one path that does not reboot (a config
         * that fails to persist leaves the gateway unconfigured and running)
         * and keeps the indicator a function of the live state rather than of
         * how the state was reached. Idempotent — a no-op every cycle but the
         * one where it changes. */
        led_solid_set(unconfigured);
    }

    diag_pm_release();
}

/* ── Responsive per-second sleep ──────────────────────────────────────── */
static void sleep_with_poll(int seconds) {
    /* Wall-clock deadline, not a fixed iteration count: a reconnect-ladder step
     * inside the loop can block for seconds, so counting iterations lets the
     * nominal sleep stretch well past `seconds` during an outage. Bounding by
     * elapsed time keeps the inter-cycle cadence honest. (du_task on core 1 is
     * unaffected either way — this paces only the main task.) A single blocking
     * step may still carry slightly past the deadline; that is fine and strictly
     * better than the old multiplicative overrun. */
    int64_t deadline = esp_timer_get_time() + (int64_t)seconds * 1000000LL;
    while (esp_timer_get_time() < deadline) {
        /* 250 ms, not 1 s: this tick is the idle-phase latency of every inbound
         * command, and the panel unlock is the one a person stands waiting for.
         * poll_incoming_mqtt() is a 100 ms UART read when nothing is pending —
         * no AT round-trip — so the finer tick costs nothing measurable. The
         * reconnect ladder paces itself (next_l*_us) and is unaffected. */
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_task_wdt_reset();
        mqtt_reconnect_if_needed();     /* heal broken connection before polling */
        poll_incoming_mqtt();
    }
}

/* Roll a failed connectivity trial (bad APN and/or bad broker) back to the
 * last-known-good values staged as _bak when the change was pushed, and
 * persist them, so the reboot that follows comes up on settings proven to
 * work. Sets conn_rollback_notify so the device announces the rollback once
 * it reconnects. APN and broker are reverted together — see nvs_config.h for
 * why they share one pending flag. */
static void conn_rollback(const char *why) {
    ESP_LOGE(TAG, "connectivity trial failed (%s) — rolling back to last-known-good", why);
    snprintf(g_cfg.apn,             sizeof(g_cfg.apn),             "%s", g_cfg.apn_bak);
    snprintf(g_cfg.apn_user,        sizeof(g_cfg.apn_user),        "%s", g_cfg.apn_user_bak);
    snprintf(g_cfg.apn_pass,        sizeof(g_cfg.apn_pass),        "%s", g_cfg.apn_pass_bak);
    snprintf(g_cfg.mqtt_host,       sizeof(g_cfg.mqtt_host),       "%s", g_cfg.mqtt_host_bak);
    snprintf(g_cfg.mqtt_user,       sizeof(g_cfg.mqtt_user),       "%s", g_cfg.mqtt_user_bak);
    snprintf(g_cfg.mqtt_pass,       sizeof(g_cfg.mqtt_pass),       "%s", g_cfg.mqtt_pass_bak);
    snprintf(g_cfg.mqtt_base_topic, sizeof(g_cfg.mqtt_base_topic), "%s", g_cfg.mqtt_base_topic_bak);
    g_cfg.mqtt_port            = g_cfg.mqtt_port_bak;
    g_cfg.mqtt_tls             = g_cfg.mqtt_tls_bak;
    g_cfg.conn_pending         = false;
    g_cfg.conn_rollback_notify = true;
    nvs_config_save(&g_cfg);
}

/* ═══════════════════════════════════════════════════════════════════════ */
void app_main(void) {
    /* FIRST — before anything can overwrite a slot. Latches what the previous
     * boot was doing when it stopped, so a hang that produced no MQTT session
     * and no console reader is still explainable from the cloud. */
    forensics_init();
    forensics_mark(FSLOT_MAIN, F_BOOT_NVS, 0);
    restart_streak_init();

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    gpio_config_t led_io = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_io);
    led_off();
    led_blink_start();   /* fast blink from power-on through the whole boot */

    modem_uart_init();
    rs485_bus_init();
    di_uart_init();
    /* Immediately after the MS51 link is up and before anything can block:
     * relay_init()'s first action is an unconditional RL1OFF, so every reset
     * path — deadman restart, task WDT, brownout, power cycle — converges on
     * the panel box being LOCKED. Runs regardless of di_enable, since the lock
     * is independent of whether digital inputs are configured. */
    relay_init();
    diag_init();

    /* Did the bootloader just reject ANOTHER product's image and fall back to
     * us? Then that image's product stamp in NVS must not be believed — see
     * nvs_config_note_reverted_foreign_image(). Read before the load so the
     * guard inside it can act on it. */
    {
        const esp_partition_t *bad  = esp_ota_get_last_invalid_partition();
        const esp_app_desc_t  *mine = esp_app_get_description();
        esp_app_desc_t         theirs;
        if (bad && mine && esp_ota_get_partition_description(bad, &theirs) == ESP_OK &&
            strncmp(mine->project_name, theirs.project_name, sizeof(theirs.project_name)) != 0) {
            char name[sizeof(theirs.project_name) + 1];
            snprintf(name, sizeof(name), "%.*s", (int)sizeof(theirs.project_name), theirs.project_name);
            nvs_config_note_reverted_foreign_image(name);
        }
    }
    nvs_config_load(&g_cfg);

    /* ── The firmware's credentials outrank NVS ───────────────────────────
     * Runs BEFORE the APN and broker are handed to the modem, so a gateway
     * carrying a previous deployment's configuration never gets as far as
     * opening a socket to the wrong client's broker. See config.h for why this
     * exists; equipment configuration is untouched either way.
     *
     * The purge is deferred to just after store_init() below — the queue is not
     * mounted yet at this point, and it must be emptied before du_task_start()
     * so not one buffered record from the old deployment can be drained to the
     * new broker. */
    /* A product change has already erased the whole config inside
     * nvs_config_load(), so g_cfg is defaults here and the gateway will report
     * "unconfigured". Its buffered records still describe the previous
     * product's equipment, so they are purged alongside the destination case. */
    bool product_wiped = nvs_config_product_wiped();
    if (product_wiped)
        ESP_LOGE(TAG, "this NVS was configured for a different product — "
                      "equipment configuration erased; push a setconfig for the "
                      "equipment actually installed here");

    /* After the product guard (which may have erased the URL record — correct,
     * it belonged to the other product) and before anything can restart. */
    note_reverted_image_at_boot();

    /* ── An unconfirmed image must not restart itself for a network fault ──
     * With CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, an OTA image that resets
     * before check_post_ota() confirms it is thrown away by the bootloader.
     * The boot path below restarts on "network never came up" and on "broker
     * unreachable AND network down" — both are ordinary at a site with poor
     * coverage, and neither says anything about the IMAGE. For a serial-
     * flashed or already-confirmed image those restarts are the right modem
     * recovery; for a pending one they would silently undo a FOTA — on a
     * cross-product swap, boot the gateway back into the other product's
     * firmware at the wrong station. So while pending, those two paths fall
     * through into the poll loop instead, where the recovery ladder owns the
     * network (it already hard-resets the modem and rebuilds the bearer) and
     * the first completed cycle confirms the image. A genuinely broken image
     * still rolls back: it never reaches that cycle, and the next reset —
     * WDT, brownout, ladder L4 — returns the old slot. Same criterion
     * cng-cpil confirms on: the image boots and runs, network irrelevant. */
    const bool pending_verify = ota_is_pending_verify();
    if (pending_verify)
        ESP_LOGW(TAG, "running an UNCONFIRMED OTA image — network faults will not "
                      "restart until the first poll cycle confirms it");

    /* A fota_successful the previous boot owed when it restarted (see
     * s_fota_owed) — paid by this one if it runs that same image, confirmed.
     * An image still pending is a NEW update: its own confirm announces it. */
    if (s_fota_owed == FOTA_OWED_MAGIC && !pending_verify)
        s_fota_completed_unsent = true;          /* s_fota_owed stays until it is sent */
    else
        s_fota_owed = 0;

    bool creds_forced = false, creds_new_dest = false;
    if (nvs_config_enforce_firmware_creds(&g_cfg, &creds_new_dest)) {
        creds_forced = true;
        if (!nvs_config_save(&g_cfg))
            ESP_LOGE(TAG, "could not persist the firmware's credentials — they "
                          "are live for this boot and will re-apply on the next");
    }

    di_set_enable(g_cfg.di_enable);
    network_set_apn(g_cfg.apn, g_cfg.apn_user, g_cfg.apn_pass);
    mqtt_set_credentials(g_cfg.mqtt_host, g_cfg.mqtt_port, g_cfg.mqtt_user, g_cfg.mqtt_pass, g_cfg.mqtt_tls);

    ESP_LOGI(TAG, "FW %s — powering modem...", FW_VERSION);

    /* The compiled sdkconfig's CONFIG_ESP_TASK_WDT_TIMEOUT_S can silently
     * diverge from WDT_TIMEOUT_MS (sdkconfig.defaults only applies when
     * sdkconfig is first generated, not on later edits) — reconfigure
     * explicitly at boot so the real timeout always matches config.h. */
    const esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_err_t wdt_err = esp_task_wdt_reconfigure(&wdt_cfg);
    if (wdt_err != ESP_OK)
        ESP_LOGW(TAG, "WDT reconfigure: %s", esp_err_to_name(wdt_err));
    ESP_LOGI(TAG, "Task WDT armed: %d s", WDT_TIMEOUT_MS / 1000);

    /* WDT add — suspend around long ops */
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    /* ── The task WDT stays armed from here for the device's whole life ────
     * This block, network bring-up, MQTT connect and FOTA all used to run
     * with the WDT DELETED, on the reasoning that they're legitimately slow
     * and shouldn't trip a false timeout. That reasoning was wrong, and it
     * was the single worst hang risk in the firmware: deleting the WDT
     * doesn't pause monitoring, it removes it. A UART driver deadlock, a
     * kernel mutex starvation, or a silicon glitch anywhere inside those
     * windows (up to ~25 min across the recovery ladder) left the device
     * hung forever, unrecoverable without someone physically power-cycling
     * a device at a fuel station.
     *
     * Keeping it armed is safe because nothing here blocks anywhere near the
     * 120 s timeout WITHOUT feeding: modem_at()/modem_read_until() reset the
     * WDT at the top of every read iteration (so they feed even when the
     * modem is silent), and the longest unfed stretch anywhere is
     * modem_hard_reset()'s ~10.2 s of PWRKEY delays — a 12x margin. Every
     * raw UART loop that doesn't go through modem_at() (fota.c's chunk
     * reader, dispenser.c's rs485_xfer) now feeds explicitly too.
     *
     * The result is what we actually want: a real hang trips the WDT within
     * 120 s and reboots, while normal slow operations are unaffected. */

    /* Modem power-on: modem_power_on() already probes AT and pulses PWRKEY.
     * If it still fails, hard power-cycle a couple of times before rebooting
     * — a wedged modem won't recover from an ESP reset alone (it stays
     * powered), so the reboot loop must escalate to PWRKEY. */
    {
        forensics_mark(FSLOT_MAIN, F_BOOT_MODEM, 0);
        bool modem_ok = modem_power_on();
        for (int i = 0; !modem_ok && i < MODEM_PWRCYCLE_MAX; i++) {
            ESP_LOGW(TAG, "Modem unresponsive — hard reset %d/%d", i + 1, MODEM_PWRCYCLE_MAX);
            modem_hard_reset();
            esp_task_wdt_reset();
            modem_ok = modem_power_on();
        }
        if (!modem_ok) {
            ESP_LOGE(TAG, "Modem dead after power-cycles — reboot");
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_restart();
        }
    }

    /* ── Acquisition comes up BEFORE the network ──────────────────────────
     * The IMEI comes from AT+CGSN, which is the modem's own identity: it needs
     * the modem powered but neither a SIM nor registration. Reading it here
     * lets the HW_IDs, the offline queue and du_task all start while the
     * network is still being brought up.
     *
     * That ordering is the whole point. Network bring-up below can fail and
     * retry for minutes, and on total failure it reboots — so a gateway that
     * powers up during an outage (power cut plus no coverage, a dead SIM, a
     * tower down) would never reach this code at all and would poll nothing
     * whatsoever until the network returned. Sales made in that window were
     * lost with no record. Now the pumps are read and buffered from the moment
     * the modem answers, and the backlog flushes whenever the link appears.
     *
     * Nothing here touches the modem beyond the one CGSN already issued, so it
     * cannot interfere with the bring-up that follows. */
    if (!modem_get_imei(g_imei, sizeof(g_imei)))
        strncpy(g_imei, "000000000000000", sizeof(g_imei));
    ESP_LOGI(TAG, "IMEI: %s", g_imei);

    build_hw_ids();
    build_topics();

    /* Queue first, then the task: du_task spills to the store as soon as its
     * RAM queue fills, and an unmounted store would turn that into data loss. */
    forensics_mark(FSLOT_MAIN, F_BOOT_STORE, 0);
    if (!store_init())
        ESP_LOGW(TAG, "offline queue unavailable — dispenser data will be lost during outages");

    /* Anything buffered belongs to the deployment this gateway just stopped
     * being part of. Purged here — after the mount, before acquisition starts —
     * because the alternative is delivering one client's sales into another
     * client's account the moment the new broker comes up. Only when the broker
     * DESTINATION actually moved: an APN-only credential change still publishes
     * to the same place, so its backlog is still valid. */
    if (creds_new_dest || product_wiped) {
        ESP_LOGW(TAG, "%s — purging the offline queue so the previous "
                      "deployment's records cannot be published here",
                 product_wiped ? "NVS belonged to a different product"
                               : "broker destination changed by the firmware");
        store_purge();
    }

    if (!du_task_start())
        ESP_LOGE(TAG, "dispenser acquisition task failed to start — NO pump data will be collected");

    /* Network bring-up: network_init() self-recovers a wedged SIM via
     * PWRKEY. Wrap it in a retry loop that hard-resets the modem between
     * attempts so registration/PDP failures also escalate rather than
     * relying on an ESP reboot that leaves the modem in the same state. */
    bool net_ok = false;
    for (int i = 0; i < NET_BRINGUP_RETRIES && !net_ok; i++) {
        if (i > 0) {
            ESP_LOGW(TAG, "Network bring-up retry %d/%d — hard-resetting modem",
                     i + 1, NET_BRINGUP_RETRIES);
            modem_hard_reset();
            esp_task_wdt_reset();
        }
        forensics_mark(FSLOT_MAIN, F_BOOT_NET, 0);
        net_ok = network_init();
    }
    if (!net_ok) {
        /* A pending APN change that can't attach would otherwise reboot-loop
         * forever on the same bad APN (no WiFi fallback, no field access) —
         * revert to the last-known-good APN before rebooting. A transient
         * outage during an APN trial also lands here and reverts; that's the
         * safe direction (we only keep a new APN once it actually attaches).
         * A config trial outranks the pending-image rule: the operator asked
         * for exactly this rollback-on-failure, and pushing a connectivity
         * change and a FOTA at the same moment is not a supported sequence. */
        if (g_cfg.conn_pending) {
            conn_rollback("network_bringup");
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_LOGE(TAG, "Network failed after %d attempts — reboot to apply rollback", NET_BRINGUP_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_restart();
        }
        if (!pending_verify && restart_streak_allow("network failed at boot")) {
            ESP_LOGE(TAG, "Network failed after %d attempts — reboot", NET_BRINGUP_RETRIES);
            du_task_flush_to_store();
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_restart();
        }
        ESP_LOGE(TAG, "Network failed after %d attempts %s — not restarting; entering the "
                      "poll loop offline, the recovery ladder keeps trying", NET_BRINGUP_RETRIES,
                 pending_verify ? "on an UNCONFIRMED image (a restart would roll the update back)"
                                : "and the restart cap is reached");
    }
    esp_task_wdt_reset();

    /* Discipline the RTC from network time now that we're registered. From here
     * on every timestamp in the firmware is a local RTC read (see modem.h), so
     * no payload path ever blocks on an AT round-trip — that is what makes
     * per-nozzle timestamping affordable.
     *
     * Retried a few times: the network's time is occasionally not yet available
     * in the first seconds after attach. A total failure is deliberately NOT
     * fatal — payloads carry the epoch string, rtc_is_synced() stays false, and
     * the periodic re-sync in run_poll_cycle() picks it up as soon as the
     * network provides it, rather than the device rebooting over a clock. */
    {
        bool time_ok = false;
        for (int i = 0; i < 3 && !time_ok; i++) {
            if (i > 0) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_task_wdt_reset();
            }
            time_ok = modem_time_sync();
        }
        if (!time_ok)
            ESP_LOGW(TAG, "initial time sync failed — RTC unsynced until re-sync");
        esp_task_wdt_reset();
    }

    /* GNSS power-on — deliberately AFTER network_init() rather than right
     * after modem confirmation: network_init() is what sends AT+CMEE=2
     * (network.c), and without verbose CME errors active yet, a failed
     * AT+QGPS=1 comes back as a bare, undiagnosable "+CME ERROR" with no
     * code (seen in the field on 2026-07-17 — moved here specifically to
     * fix that). With gps_enable false (the default) no fix is ever read or
     * published; gps_init() then only makes sure the modem is not running
     * GNSS or AGPS on its own (three queries, no write when already off). */
    gps_init(g_cfg.gps_enable);

    /* IMEI, HW_IDs, store and du_task were all brought up before network
     * bring-up (see the note there) so acquisition survives a cold boot with
     * no coverage. Nothing to repeat here. */
    mqtt_init(g_imei);
    mqtt_set_activity_cb(led_activity_pulse);   /* blink on every cloud push */

    /* Upload the trusted root CA once, before the first TLS attempt of any
     * kind — both mqtt_connect() and fota_download() assume this file
     * already exists on the modem's UFS by the time they run. A failure
     * here is logged but not fatal: mqtt_connect() will still try TLS with
     * seclevel=1 pointing at a cacert file that doesn't exist, which the
     * modem will reject — that's a loud, visible connect failure, safer
     * than silently falling back to no validation. */
    tls_ca_provision();

    /* Initial MQTT: try a few times; if the broker/session won't come up,
     * rebuild the whole network (PDP may be stale) before giving up.
     * WDT stays armed throughout — see the note above the modem power-on. */
    for (int attempts = 0; attempts < MQTT_RECONNECT_RETRIES && !mqtt_is_connected(); attempts++) {
        if (mqtt_connect()) break;
        ESP_LOGW(TAG, "MQTT retry %d/%d", attempts + 1, MQTT_RECONNECT_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(1000 + (attempts * 1000) + (esp_random() % 2000)));
        esp_task_wdt_reset();
    }
    if (!mqtt_is_connected()) {
        ESP_LOGW(TAG, "MQTT down — rebuilding network before final retry");
        for (int i = 0; i < NET_BRINGUP_RETRIES && !mqtt_is_connected(); i++) {
            modem_hard_reset();
            esp_task_wdt_reset();
            if (network_init() && mqtt_connect()) break;
        }
    }
    if (!mqtt_is_connected()) {
        /* A connectivity change on TRIAL that can't reach a broker must revert
         * and reboot regardless of why — new settings are kept only once proven
         * end-to-end, and the broker half is the likely culprit. This path is
         * unchanged: reboot into the last-known-good settings. */
        if (g_cfg.conn_pending) {
            conn_rollback("broker_unreachable");
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_LOGE(TAG, "connectivity trial unreachable — reboot to apply rollback");
            esp_restart();
        }
        /* Not a trial. If the network is registered the broker/server is simply
         * down — rebooting here would reboot-loop the whole fleet for the length
         * of the outage. Enter the poll loop OFFLINE instead: du_task buffers
         * every payload to flash and mqtt_reconnect_if_needed() retries the
         * broker each cycle, flushing the backlog when it returns — identical to
         * the staying-online behaviour used mid-run. Only reboot if the network
         * itself never came up, where a fresh modem bring-up can still help. */
        /* Registration, deliberately NOT network_data_bearer_up() here. This
         * decision is only "is the modem alive enough to keep running", and the
         * answer should be yes as often as possible: entering the poll loop
         * keeps du_task acquiring and buffering, and mqtt_reconnect_if_needed()
         * now escalates a dead PDP context to L2 on its very first call. A
         * bearer test here would reboot instead — slower to recover, and it
         * takes the pumps down while it happens. */
        if (network_check()) {
            ESP_LOGW(TAG, "broker unreachable at boot but network registered — "
                          "entering poll loop offline, buffering to flash "
                          "(the recovery ladder rebuilds the bearer if that is "
                          "what is wrong)");
        } else if (pending_verify) {
            ESP_LOGE(TAG, "MQTT failed and network down on an UNCONFIRMED image — not "
                          "restarting (it would roll the update back); entering the poll "
                          "loop offline, the recovery ladder keeps trying");
        } else if (!restart_streak_allow("MQTT failed and network down at boot")) {
            /* Cap reached — fall through into the poll loop offline. */
        } else {
            ESP_LOGE(TAG, "MQTT failed and network down after recovery — reboot");
            du_task_flush_to_store();
            /* Pre-restart delay, matching every other boot restart path
             * (modem-dead, network-failed), so a persistently dead modem cannot
             * spin reboots back-to-back at whatever rate the preceding calls
             * happen to fail. */
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_restart();
        }
    }

    /* A pending connectivity change that reached a live broker session has
     * proven itself end-to-end — commit it as the new known-good baseline so
     * a future bad push has something correct to roll back to. */
    if (g_cfg.conn_pending) {
        g_cfg.conn_pending = false;
        nvs_config_save(&g_cfg);
        ESP_LOGI(TAG, "connectivity change confirmed — now permanent");
    }

    esp_task_wdt_reset();

    led_blink_stop();   /* boot done (MQTT up) — stop the boot blink */

    subscribe_all();
    /* check_post_ota() is deliberately NOT called here. Calling it this early
     * — right after MQTT connects, before a single sensor has been polled —
     * would mark a freshly-FOTA'd image "valid" before proving it can do its
     * actual job. A crash inside du_task/poll_genset()/poll_radar()
     * would then reboot into the SAME already-validated broken image forever:
     * the bootloader's rollback net only protects an image that never
     * confirmed itself, so confirming too early defeats it. Deferred to
     * after the first successful run_poll_cycle() below instead. */

    if (g_cfg.conn_rollback_notify) {
        char rb[160];
        snprintf(rb, sizeof(rb),
                 "{\"HW_ID\":\"%s\",\"status\":\"connectivity_rollback\","
                 "\"reason\":\"new_settings_unreachable\"}", g_imei);
        /* Only clear the flag on a CONFIRMED publish — same rule diagnostics.c
         * already applies to its counters. The link has only just come up here
         * and is at its least reliable; clearing unconditionally (as this used
         * to) meant one failed publish silently swallowed the only notice the
         * cloud ever gets that a connectivity push was rejected and reverted.
         * Left set, it simply retries on the next boot. */
        if (mqtt_publish(t_logs, rb)) {
            g_cfg.conn_rollback_notify = false;
            nvs_config_save(&g_cfg);
        } else {
            ESP_LOGW(TAG, "rollback notice publish failed — retrying next boot");
        }
    }

    if (g_cfg.di_enable) {
        /* 6144, not the original 4096: di_task's payload[256]/resp[NUVOTON_BUF]
         * locals plus FreeRTOS task overhead plus the internal stack frames
         * of uart_write_bytes/uart_read_bytes/esp_task_wdt_add left only a
         * thin margin. RAM is abundant (~22% used of 320 KB) — spend it on
         * stack safety over a bug class (silent overflow/corruption) that's
         * brutal to diagnose in the field. */
        BaseType_t di_task_created =
            xTaskCreatePinnedToCore(di_task, "di_task", 6144, NULL, 5, NULL, 1);
        if (di_task_created != pdPASS)
            ESP_LOGE(TAG, "di_task creation failed (heap OOM?) — DI monitoring will not run");
    }

    bool unconfigured = (g_cfg.du_count == 0
                         && g_cfg.ztech_count == 0
                         && g_cfg.pantech_count == 0
                         && g_cfg.genset_type == GENSET_OFF
                         && !g_cfg.di_enable);
    /* Set BEFORE the announce, so the hold starts with the very first
     * "unconfigured" message rather than one publish later — and so that
     * announce's own activity pulse already renders as the inverted flash. */
    led_solid_set(unconfigured);

    char ready_msg[128];
    snprintf(ready_msg, sizeof(ready_msg),
             "{\"HW_ID\":\"%s\",\"status\":\"%s\",\"fw\":\"%s\"}",
             g_imei,
             unconfigured ? "unconfigured" : "ready",
             FW_VERSION);
    mqtt_publish(t_logs, ready_msg);

    /* The previous update was thrown away by the bootloader (see
     * note_reverted_image_at_boot) — that push's answer. Kept until a publish
     * goes through (the poll loop retries it) rather than tried once. */
    if (s_reverted_image[0]) {
        char rv[200];
        snprintf(rv, sizeof(rv),
                 "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"fota_reverted\","
                 "\"image\":\"%s\",\"fw\":\"%s\"}", g_imei, s_reverted_image, FW_VERSION);
        fota_failed_owe(rv);
        publish_fota_failed_owed();
    }

    /* Announce a credential takeover to whoever the device now reports to. On a
     * gateway moved between deployments this is the NEW owner's first sight of
     * it, and it tells them the unit came in carrying someone else's config and
     * has been re-pointed — worth surfacing, because it is also the moment to
     * check the equipment configuration still matches what is physically
     * installed (that part is deliberately preserved, not reset). */
    if (creds_forced) {
        char cred_msg[224];
        snprintf(cred_msg, sizeof(cred_msg),
                 "{\"HW_ID\":\"%s\",\"status\":\"firmware_credentials_applied\","
                 "\"broker_changed\":%s,\"config_erased\":%s,\"fw\":\"%s\"}",
                 g_imei, creds_new_dest ? "true" : "false",
                 product_wiped ? "true" : "false", FW_VERSION);
        mqtt_publish(t_logs, cred_msg);
        ESP_LOGW(TAG, "connectivity was taken from the firmware, not NVS "
                      "(broker %s)", creds_new_dest ? "CHANGED" : "unchanged");
    }

    /* Report a lock-safety restart from the previous boot: the deadman fired,
     * meaning an unlock window never closed normally. The box is locked again
     * by now (relay_init's boot RL1OFF), but the cloud should surface it —
     * it points at either a stalled relay task or an MS51 link problem, and
     * the panel may have stood unlocked for up to RELAY_UNLOCK_DEADMAN_MS. */
    if (relay_last_boot_was_safety_restart()) {
        char lock_msg[128];
        snprintf(lock_msg, sizeof(lock_msg),
                 "{\"HW_ID\":\"%s\",\"event\":\"safety_restart\"}", g_imei);
        mqtt_publish(t_lock_out, lock_msg);
        ESP_LOGE(TAG, "PREV BOOT: relay deadman fired — verify panel lock state");
    }

    ESP_LOGI(TAG, "Poll loop start — cycle %d ms", PUBLISH_INTERVAL_MS);

    bool post_ota_checked = false;
    for (;;) {
        int64_t cycle_start = esp_timer_get_time();

        esp_task_wdt_reset();
        poll_incoming_mqtt();   /* drain any queued commands before sensor work */
        run_poll_cycle();
        if (!post_ota_checked) {
            /* Reached only after a full poll cycle RETURNED — i.e. the new
             * image survived a real pass through dispenser/radar/genset/DI
             * polling without crashing. That's the actual bar for "this
             * build works", so only now do we cancel the bootloader
             * rollback and tell the cloud fota_successful (a mark that did
             * not take is retried next cycle — check_post_ota). */
            post_ota_checked = check_post_ota();
        }
        publish_fota_completed();    /* no-op unless the success is still owed */
        publish_fota_failed_owed();  /* no-op unless a failure is still owed */

        /* Start-to-start pacing, not a flat post-work sleep.
         *
         * This used to be sleep_with_poll(30) unconditionally, which ran AFTER
         * the cycle regardless of how long the cycle took — so the real period
         * was always work + 30 s. A 2-dispenser station doing ~50 s of work
         * published every ~80 s despite PUBLISH_INTERVAL_MS being 30 s.
         * Measuring from the cycle START makes the constant mean what it says,
         * and when the work alone already exceeds it the next cycle begins
         * immediately instead of idling for a further 30 s. That directly
         * tightens the gap between consecutive reads of the same nozzle, which
         * is what bounds how many separate sales can collapse into one
         * totalizer delta. (This is also how the older Arduino firmware paced
         * itself, which is why it measured ~57 s on 4 nozzles where this
         * firmware measured ~84 s for identical RS485 work.) */
        int64_t elapsed_ms = (esp_timer_get_time() - cycle_start) / 1000;
        int     remain_s   = (int)(((int64_t)PUBLISH_INTERVAL_MS - elapsed_ms) / 1000);
        if (remain_s > 0) {
            /* The healthy resting phase. A last_stop naming "idle" means the
             * gateway wedged while doing nothing, which points at the modem
             * poll inside sleep_with_poll rather than any equipment. */
            forensics_mark(FSLOT_MAIN, F_IDLE, 0);
            sleep_with_poll(remain_s);
        } else {
            /* Overran the interval — no idle time left. Still do the per-second
             * housekeeping sleep_with_poll() would have done, so a dropped link
             * is healed and queued cloud commands are serviced even on a busy
             * station that never gets a quiet moment. */
            esp_task_wdt_reset();
            mqtt_reconnect_if_needed();
            poll_incoming_mqtt();
            /* Explicit yield so this branch can never become a tight loop.
             * Today every path that reaches it has already blocked on RS485/
             * modem I/O, so the scheduler has run — but that is a property of
             * the current call graph, not something this loop enforces. One
             * tick is ~0.2% of a 50 s cycle: unmeasurable against the dispenser
             * cadence, and it makes the guarantee local instead of inherited. */
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
