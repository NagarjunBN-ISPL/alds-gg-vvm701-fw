#include "config.h"
#include "modem.h"
#include "network.h"
#include "fota.h"
#include "mqtt.h"
#include "nvs_config.h"
#include "dispenser.h"
#include "radar.h"
#include "genset.h"
#include "di.h"
#include "gps.h"
#include "tls_ca.h"
#include "diagnostics.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
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
static DeviceConfig g_cfg;

/* BlueSky and the two radar types get independent label spaces even though
 * they share the RS485 bus. */
static char g_hw_bs[DU_BLUESKY_MAX][24];
static char g_hw_ztech[RADAR_MAX][24];
static char g_hw_pantech[RADAR_MAX][24];
static char g_hw_genset[24], g_hw_di[24];

#define TOPIC_MAX 200
static char t_data[TOPIC_MAX];
static char t_update[TOPIC_MAX];
static char t_update_bcast[TOPIC_MAX];
static char t_setcfg[TOPIC_MAX];
static char t_rsp_bs_in[DU_BLUESKY_MAX][TOPIC_MAX], t_rsp_bs_out[DU_BLUESKY_MAX][TOPIC_MAX];
static char t_kf_bs_in[DU_BLUESKY_MAX][TOPIC_MAX],  t_kf_bs_out[DU_BLUESKY_MAX][TOPIC_MAX];

typedef struct {
    bool pending;
    char cmd_hex[256];
    char resp_topic[TOPIC_MAX];
} PendingCmd_t;

static PendingCmd_t g_rsp_bs_cmd[DU_BLUESKY_MAX];
static PendingCmd_t g_kf_bs_cmd[DU_BLUESKY_MAX];
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

/* One short flash — registered with mqtt_set_activity_cb() so it fires on
 * every publish. This is the steady-state indicator: a blink = data pushed. */
static void led_activity_pulse(void) { led_blink(1, 40, 0); }

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
    led_off();
}

/* ── RS485 helpers ────────────────────────────────────────────────────── */
static void rs485_init(void) {
    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(RS485_UART, RS485_BUF, RS485_BUF, 0, NULL, 0);
    uart_param_config(RS485_UART, &cfg);
    uart_set_pin(RS485_UART, RS485_TX_PIN, RS485_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void rs485_set_8n1(uint32_t baud) {
    /* uart_write_bytes() returns once bytes are queued to the TX ring
     * buffer, not once they've physically left the pin. Switching parity/
     * baud while the last byte of a prior command is still shifting out
     * would corrupt it on the wire. Every call site here follows a
     * completed (or timed-out) read, so in practice the TX finished long
     * ago — this is a correctness guarantee regardless of call order, not
     * a fix for an observed failure. */
    uart_wait_tx_idle_polling(RS485_UART);
    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(RS485_UART, &cfg);
}

static void rs485_set_8e1_9600(void) {
    uart_wait_tx_idle_polling(RS485_UART);
    du_rs485_set_bluesky_mode();
}

/* ── Build identifiers ────────────────────────────────────────────────── */
static void build_hw_ids(void) {
    /* Auto-assigned HW_ID suffixes are numeric and collision-free even at full
     * capacity: dispensers use their unit number (01..06); Ztech radars 21..24
     * and Pantech radars 31..34 (tens digit = type, ones digit = tank number);
     * the genset uses its gensetSlave (default 10); and DI is fixed at 40. None
     * of these ranges overlap, so a fully-loaded gateway (6 dispensers, 4 Ztech,
     * 4 Pantech, genset, DI) never produces a duplicate. An explicit sfx* from
     * setconfig always overrides; validate_suffixes() computes the identical auto
     * values so the uniqueness check stays in sync with what is published here. */
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        if (g_cfg.sfx_bs[i][0])
            snprintf(g_hw_bs[i], sizeof(g_hw_bs[i]), "%s%s", g_imei, g_cfg.sfx_bs[i]);
        else
            snprintf(g_hw_bs[i], sizeof(g_hw_bs[i]), "%s%02d", g_imei, i + 1);              /* 01..06 (unit no.) */
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        if (g_cfg.sfx_ztech[i][0])
            snprintf(g_hw_ztech[i], sizeof(g_hw_ztech[i]), "%s%s", g_imei, g_cfg.sfx_ztech[i]);
        else
            snprintf(g_hw_ztech[i], sizeof(g_hw_ztech[i]), "%s%02d", g_imei, 20 + i + 1);   /* 21..24 (Ztech tank no.) */
        if (g_cfg.sfx_pantech[i][0])
            snprintf(g_hw_pantech[i], sizeof(g_hw_pantech[i]), "%s%s", g_imei, g_cfg.sfx_pantech[i]);
        else
            snprintf(g_hw_pantech[i], sizeof(g_hw_pantech[i]), "%s%02d", g_imei, 30 + i + 1); /* 31..34 (Pantech tank no.) */
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
        snprintf(g_hw_di, sizeof(g_hw_di), "%s40", g_imei);                                 /* 40 (not on the Modbus bus) */
}

static void build_topics(void) {
    /* One flat, IMEI-less outbound topic (e.g. "gogas/data") — the original
     * pipeline layout the cloud built against. ALL device→cloud messages go
     * here: equipment telemetry, lifecycle/status events, and the diagnostics
     * and sysmon health reports (which used to have their own topics but now
     * share this one). Every payload carries "HW_ID" (bare IMEI, or IMEI+suffix
     * for per-equipment telemetry) so the cloud attributes it to a device by
     * that field, and self-identifies its kind by which top-level key is
     * present: "nozzles" / "tanks" / "genset" / "di" for telemetry, "status"
     * for lifecycle, "diagnostics" / "sysmon" for the health reports. Inbound command
     * topics (setconfig/update/rsp/kfactor below) stay per-IMEI — they must
     * target one device, and flattening them would make every device act on
     * every push. */
    snprintf(t_data,        TOPIC_MAX, "%s/data",          g_cfg.mqtt_base_topic);
    snprintf(t_update,      TOPIC_MAX, "%s/%s/update",     g_cfg.mqtt_base_topic, g_imei);
    snprintf(t_update_bcast,TOPIC_MAX, "%s/update",        g_cfg.mqtt_base_topic);
    snprintf(t_setcfg,      TOPIC_MAX, "%s/%s/setconfig",  g_cfg.mqtt_base_topic, g_imei);
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        snprintf(t_rsp_bs_in[i],  TOPIC_MAX, "%s/rsp/imei/%s/in",      g_cfg.mqtt_base_topic, g_hw_bs[i]);
        snprintf(t_rsp_bs_out[i], TOPIC_MAX, "%s/rsp/imei/%s/out",     g_cfg.mqtt_base_topic, g_hw_bs[i]);
        snprintf(t_kf_bs_in[i],   TOPIC_MAX, "%s/kfactor/imei/%s/in",  g_cfg.mqtt_base_topic, g_hw_bs[i]);
        snprintf(t_kf_bs_out[i],  TOPIC_MAX, "%s/kfactor/imei/%s/out", g_cfg.mqtt_base_topic, g_hw_bs[i]);
    }
}

/* Forward decls — defined later in the file. */
static void mqtt_reconnect_if_needed(void);
static void poll_incoming_mqtt(void);


static void subscribe_all(void) {
    mqtt_subscribe(t_update);
    mqtt_subscribe(t_update_bcast);
    mqtt_subscribe(t_setcfg);
    for (int i = 0; i < g_cfg.du_bluesky_count; i++) {
        mqtt_subscribe(t_rsp_bs_in[i]);
        mqtt_subscribe(t_kf_bs_in[i]);
    }
}

/* ── Tiny JSON helpers ────────────────────────────────────────────────── */
static int json_get_int(const char *json, const char *key, int def) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return def;
    p += strlen(search);
    while (*p == ' ') p++;
    return atoi(p);
}

static bool json_get_str(const char *json, const char *key,
                          char *out, size_t out_size) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '"') return false;
    p++;
    const char *end = strchr(p, '"');
    if (!end) return false;
    size_t len = (size_t)(end - p);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool json_has_key(const char *json, const char *key) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    return strstr(json, search) != NULL;
}

/* True if a setconfig payload carries any connectivity credential (broker or
 * APN user/pass). Used ONLY to suppress the plaintext payload log below so a
 * pushed secret never reaches the serial console — the payload handed to
 * parse_setconfig() is untouched, so parsing behaviour is identical. */
static bool setcfg_has_secret(const char *json) {
    return json_has_key(json, "mqttUser") || json_has_key(json, "mqttPass") ||
           json_has_key(json, "apnUser")  || json_has_key(json, "apnPass");
}

/* Parse a JSON boolean. Accepts true/false and 1/0 — json_get_int can't be
 * used here because atoi("true") is 0, i.e. it can't tell true from false. */
static bool json_get_bool(const char *json, const char *key, bool def) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return def;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p == 't' || *p == 'T' || *p == '1') return true;
    if (*p == 'f' || *p == 'F' || *p == '0') return false;
    return def;
}

/* ── setconfig ────────────────────────────────────────────────────────── */
/* This is only the coarse "is this payload a recognized setconfig at all"
 * gate — it requires at least ONE key below to be present, then the actual
 * per-field parsing loops (sfxBs%d/sfxZtech%d/sfxPantech%d further down)
 * run over DU_BLUESKY_MAX/RADAR_MAX dynamically regardless of this list. A
 * key missing here only matters for a payload that contains *nothing else*
 * recognized (e.g. a bare {"sfxBs4":"04"}), which would otherwise be
 * rejected as bad_payload — so every suffix key up to the current caps is
 * listed explicitly to avoid that trap. Keep this in sync with
 * DU_BLUESKY_MAX/RADAR_MAX in config.h when either cap changes. */
static const char * const s_setcfg_known_keys[] = {
    "duBlueskyCount","ztechCount","pantechCount","radarType","tankCount",
    "gensetType","gensetModel","gensetSlave","gensetBaud","diEnable","gpsEnable",
    "sfxGenset","sfxDI",
    "sfxBs1","sfxBs2","sfxBs3","sfxBs4","sfxBs5","sfxBs6",
    "sfxZtech1","sfxZtech2","sfxZtech3","sfxZtech4",
    "sfxPantech1","sfxPantech2","sfxPantech3","sfxPantech4",
    "apn","apnUser","apnPass",
    "mqttHost","mqttPort","mqttUser","mqttPass","mqttBaseTopic","mqttTls",
};

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

    /* Auto values mirror build_hw_ids() exactly: dispensers 01..06, Ztech
     * 21..24, Pantech 31..34, genset = its slave (default 10), DI 40. These
     * ranges never overlap, so the auto values alone can't collide; computing
     * them here is what lets the uniqueness check below catch a CUSTOM suffix
     * that collides with another unit's auto value (e.g. a dispenser manually
     * set to "10" while a genset is active). */
    for (int i = 0; i < c->du_bluesky_count; i++) {
        snprintf(label[n], sizeof(label[n]), "BlueSky%d", i + 1);
        if (c->sfx_bs[i][0]) sfx[n] = c->sfx_bs[i];
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "%02d", i + 1); sfx[n] = auto_sfx[n]; }
        n++;
    }
    for (int i = 0; i < c->ztech_count; i++) {
        snprintf(label[n], sizeof(label[n]), "Ztech%d", i + 1);
        if (c->sfx_ztech[i][0]) sfx[n] = c->sfx_ztech[i];
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "%02d", 20 + i + 1); sfx[n] = auto_sfx[n]; }
        n++;
    }
    for (int i = 0; i < c->pantech_count; i++) {
        snprintf(label[n], sizeof(label[n]), "Pantech%d", i + 1);
        if (c->sfx_pantech[i][0]) sfx[n] = c->sfx_pantech[i];
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "%02d", 30 + i + 1); sfx[n] = auto_sfx[n]; }
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
        else { snprintf(auto_sfx[n], sizeof(auto_sfx[n]), "40"); sfx[n] = auto_sfx[n]; }
        n++;
    }

    /* Format: a non-empty custom suffix must be exactly 2 ALPHANUMERIC
     * characters. Length alone isn't enough: the suffix is embedded verbatim
     * into every JSON payload ("HW_ID":"<imei><sfx>") and into the rsp/
     * kfactor MQTT topic strings — a quote corrupts every telemetry message
     * from that unit into unparseable JSON, and a space/wildcard breaks the
     * topic matching. */
    for (int i = 0; i < n; i++) {
        if (sfx[i][0] && (strlen(sfx[i]) != 2 ||
                          !isalnum((unsigned char)sfx[i][0]) ||
                          !isalnum((unsigned char)sfx[i][1]))) {
            snprintf(err, err_size, "%s suffix must be exactly 2 alphanumeric characters", label[i]);
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

/* Publish a config_failed status with a machine-readable reason. */
static void setcfg_fail(const char *reason) {
    char err[160];
    snprintf(err, sizeof(err),
             "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"%s\"}",
             g_imei, reason);
    mqtt_publish(t_data, err);
}

/* Read side of the read/write pair the old local web portal gave for free.
 * Publishes the LIVE DeviceConfig on the data topic so the fleet is auditable
 * ("what is station X set to / did my change take?") without a site visit.
 * Secrets are never echoed: passwords are reported only as *Set booleans, so
 * a report can be copied, tweaked and re-sent without accidentally writing a
 * literal "***" over the real password (omit the pass key to keep it). */
static void publish_config_report(void) {
    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"HW_ID\":\"%s\",\"status\":\"config_report\",\"fw\":\"%s\","
        "\"duBlueskyCount\":%u,\"ztechCount\":%u,\"pantechCount\":%u,"
        "\"gensetType\":%u,\"gensetModel\":%u,\"gensetSlave\":%u,\"gensetBaud\":%lu,"
        "\"diEnable\":%u,\"gpsEnable\":%s,"
        "\"sfxGenset\":\"%s\",\"sfxDI\":\"%s\","
        "\"sfxBs1\":\"%s\",\"sfxBs2\":\"%s\",\"sfxBs3\":\"%s\","
        "\"sfxBs4\":\"%s\",\"sfxBs5\":\"%s\",\"sfxBs6\":\"%s\","
        "\"sfxZtech1\":\"%s\",\"sfxZtech2\":\"%s\",\"sfxZtech3\":\"%s\",\"sfxZtech4\":\"%s\","
        "\"sfxPantech1\":\"%s\",\"sfxPantech2\":\"%s\",\"sfxPantech3\":\"%s\",\"sfxPantech4\":\"%s\","
        "\"apn\":\"%s\",\"apnUser\":\"%s\",\"apnPassSet\":%s,"
        "\"mqttHost\":\"%s\",\"mqttPort\":%u,\"mqttUser\":\"%s\","
        "\"mqttPassSet\":%s,\"mqttBaseTopic\":\"%s\",\"mqttTls\":%s,"
        "\"connPending\":%s"
        "}",
        g_imei, FW_VERSION,
        (unsigned)g_cfg.du_bluesky_count, (unsigned)g_cfg.ztech_count, (unsigned)g_cfg.pantech_count,
        (unsigned)g_cfg.genset_type, (unsigned)g_cfg.genset_model, (unsigned)g_cfg.genset_slave,
        (unsigned long)g_cfg.genset_baud,
        (unsigned)g_cfg.di_enable, g_cfg.gps_enable ? "true" : "false",
        g_cfg.sfx_genset, g_cfg.sfx_di,
        g_cfg.sfx_bs[0], g_cfg.sfx_bs[1], g_cfg.sfx_bs[2],
        g_cfg.sfx_bs[3], g_cfg.sfx_bs[4], g_cfg.sfx_bs[5],
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
    mqtt_publish(t_data, buf);
}

static void parse_setconfig(const char *json) {
    /* Remote commands — parsed with json_get_str so both compact
     * ("cmd":"restart") and pretty-printed ("cmd": "restart") JSON work. */
    char cmdbuf[24] = "";
    if (json_get_str(json, "cmd", cmdbuf, sizeof(cmdbuf))) {
        if (strcmp(cmdbuf, "restart") == 0) {
            ESP_LOGI(TAG, "Remote restart");
            /* ACK before we vanish — mqtt_publish is synchronous (waits for the
             * QMTPUBEX result), so the cloud sees the command was accepted
             * rather than just watching the device drop off and reappear. */
            char ack[128];
            snprintf(ack, sizeof(ack),
                     "{\"HW_ID\":\"%s\",\"status\":\"restarting\"}", g_imei);
            mqtt_publish(t_data, ack);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }
        if (strcmp(cmdbuf, "factory_reset") == 0) {
            ESP_LOGI(TAG, "Factory reset");
            /* ACK before wiping NVS + rebooting, same reasoning as restart. */
            char ack[128];
            snprintf(ack, sizeof(ack),
                     "{\"HW_ID\":\"%s\",\"status\":\"factory_resetting\"}", g_imei);
            mqtt_publish(t_data, ack);
            nvs_config_factory_reset();
            vTaskDelay(pdMS_TO_TICKS(500));
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
    }

    /* Reject non-JSON / unrecognised payloads. A stray message published to
     * the setconfig topic (seen in the field: a FOTA URL sent to the wrong
     * topic) previously fell through all the json_get_int defaults, "applied"
     * nothing — and still saved NVS and REBOOTED the device. Require a JSON
     * object containing at least one known key before touching anything. */
    const char *s0 = json;
    while (*s0 == ' ' || *s0 == '\t' || *s0 == '\r' || *s0 == '\n') s0++;
    bool known_key = false;
    {
        char search[24];
        for (size_t i = 0; i < sizeof(s_setcfg_known_keys) / sizeof(s_setcfg_known_keys[0]); i++) {
            snprintf(search, sizeof(search), "\"%s\"", s_setcfg_known_keys[i]);
            if (strstr(json, search)) { known_key = true; break; }
        }
    }
    if (*s0 != '{' || !known_key) {
        ESP_LOGW(TAG, "setconfig: unrecognised payload — ignored (no reboot)");
        char err[128];
        snprintf(err, sizeof(err),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"bad_payload\"}",
                 g_imei);
        mqtt_publish(t_data, err);
        return;
    }

    char st[128];
    snprintf(st, sizeof(st),
             "{\"HW_ID\":\"%s\",\"status\":\"config_received\"}", g_imei);
    mqtt_publish(t_data, st);

    /* Stage into a copy so a validation failure below leaves the live
     * g_cfg (and the running device) completely untouched. */
    DeviceConfig nc = g_cfg;

    int du_bs = json_get_int(json,"duBlueskyCount", nc.du_bluesky_count);
    if (du_bs < 0) du_bs = 0;
    if (du_bs > DU_BLUESKY_MAX) du_bs = DU_BLUESKY_MAX;
    nc.du_bluesky_count = (uint8_t)du_bs;

    /* ── Radar count — two input styles accepted, never mixed ─────────────
     * (a) old-portal style : radarType ("ztech"/"pantech"/"none") + tankCount
     * (b) explicit style   : ztechCount and/or pantechCount (lets you run
     *                        BOTH radar types at once — the portal couldn't).
     * Ztech (slaves 4,5) and Pantech (slaves 6,7) occupy fixed non-
     * overlapping ranges, so each count is capped independently at RADAR_MAX.
     * Anything ambiguous is REJECTED loudly rather than silently guessed — a
     * stray "tankCount" with no "radarType" used to be dropped with no hint
     * that the radar config never applied. */
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
        json_get_str(json, "radarType", rtype, sizeof(rtype));
        int tc = json_get_int(json, "tankCount", 0);
        if (tc < 0) tc = 0;
        if (tc > RADAR_MAX) tc = RADAR_MAX;
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
        int ztech   = json_get_int(json,"ztechCount",   nc.ztech_count);
        int pantech = json_get_int(json,"pantechCount", nc.pantech_count);
        if (ztech < 0) ztech = 0;
        if (ztech > RADAR_MAX) ztech = RADAR_MAX;
        if (pantech < 0) pantech = 0;
        if (pantech > RADAR_MAX) pantech = RADAR_MAX;
        nc.ztech_count   = (uint8_t)ztech;
        nc.pantech_count = (uint8_t)pantech;
    }

    /* Ztech and Pantech ranges are adjacent (4.. / 6..), not spaced apart —
     * ztechCount >= 3 pushes Ztech's range onto slave 6, Pantech tank 1's
     * fixed address. Reject rather than let both types poll the same slave. */
    if (nc.ztech_count > 0 && nc.pantech_count > 0 &&
        ZTECH_SLAVE_BASE + nc.ztech_count > PANTECH_SLAVE_BASE) {
        ESP_LOGW(TAG, "setconfig: rejected — ztechCount %u pushes Ztech range onto Pantech slave %u",
                 nc.ztech_count, PANTECH_SLAVE_BASE);
        char err[192];
        snprintf(err, sizeof(err),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_failed\","
                 "\"reason\":\"ztechCount %u and pantechCount %u produce overlapping radar slave ranges\"}",
                 g_imei, nc.ztech_count, nc.pantech_count);
        mqtt_publish(t_data, err);
        return;
    }

    nc.genset_type   = (uint8_t) json_get_int(json,"gensetType", nc.genset_type);
    nc.genset_model  = (uint8_t) json_get_int(json,"gensetModel",nc.genset_model);
    nc.genset_slave  = (uint8_t) json_get_int(json,"gensetSlave",nc.genset_slave);
    nc.genset_baud   = (uint32_t)json_get_int(json,"gensetBaud", (int)nc.genset_baud);
    nc.di_enable     = (uint8_t) json_get_int(json,"diEnable",   nc.di_enable);
    nc.gps_enable    = json_get_bool(json, "gpsEnable", nc.gps_enable);

    /* Baud whitelist — an out-of-range value (0 in particular) reaches the
     * UART clock-divider division in rs485_set_8n1() and panics the chip
     * into a reboot loop. Coerce to the Sedemac default like the counts. */
    if (nc.genset_baud != 4800 && nc.genset_baud != 9600 && nc.genset_baud != 19200)
        nc.genset_baud = 9600;
    if (nc.genset_model > GENSET_MODEL_MAX) nc.genset_model = GENSET_MODEL_GC1211;
    if (nc.genset_slave < 1 || nc.genset_slave > 247) nc.genset_slave = GENSET_SLAVE_DEFAULT;

    /* Genset shares the RS485 bus with the radars, and its FC03/FC04 polls
     * are byte-identical to theirs except for the slave address — a genset
     * slave inside an active radar range would decode tank-level registers
     * as genset data with every slave/FC/CRC echo check passing. Reject. */
    if (nc.genset_type != GENSET_OFF) {
        bool clash_z = nc.ztech_count   > 0 && nc.genset_slave >= ZTECH_SLAVE_BASE
                       && nc.genset_slave < ZTECH_SLAVE_BASE + nc.ztech_count;
        bool clash_p = nc.pantech_count > 0 && nc.genset_slave >= PANTECH_SLAVE_BASE
                       && nc.genset_slave < PANTECH_SLAVE_BASE + nc.pantech_count;
        if (clash_z || clash_p) {
            ESP_LOGW(TAG, "setconfig: rejected — gensetSlave %u collides with %s radar range",
                     nc.genset_slave, clash_z ? "Ztech" : "Pantech");
            char err[192];
            snprintf(err, sizeof(err),
                     "{\"HW_ID\":\"%s\",\"status\":\"config_failed\","
                     "\"reason\":\"gensetSlave %u collides with radar slave range\"}",
                     g_imei, nc.genset_slave);
            mqtt_publish(t_data, err);
            return;
        }
    }

    json_get_str(json,"sfxGenset",nc.sfx_genset, sizeof(nc.sfx_genset));
    json_get_str(json,"sfxDI",    nc.sfx_di,     sizeof(nc.sfx_di));

    char keybuf[16];
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        snprintf(keybuf, sizeof(keybuf), "sfxBs%d", i + 1);
        json_get_str(json, keybuf, nc.sfx_bs[i], sizeof(nc.sfx_bs[i]));
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        snprintf(keybuf, sizeof(keybuf), "sfxZtech%d", i + 1);
        json_get_str(json, keybuf, nc.sfx_ztech[i], sizeof(nc.sfx_ztech[i]));
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        snprintf(keybuf, sizeof(keybuf), "sfxPantech%d", i + 1);
        json_get_str(json, keybuf, nc.sfx_pantech[i], sizeof(nc.sfx_pantech[i]));
    }

    /* ── Connectivity: APN + MQTT broker (optional — only touched if an
     * apn/mqtt key is present) ───────────────────────────────────────────
     * g_cfg's primary apn and mqtt fields are always the last connectivity
     * config that PROVED it reached a live broker session (app_main's
     * pending-trial logic enforces this on every boot), so they're exactly
     * what a bad push should roll back to. Stage them into nc's _bak fields
     * from g_cfg (still untouched here) before nc's primary fields change. */
    bool apn_present   = json_get_str(json,"apn",           nc.apn,             sizeof(nc.apn));
    bool auser_present = json_get_str(json,"apnUser",       nc.apn_user,        sizeof(nc.apn_user));
    bool apass_present = json_get_str(json,"apnPass",       nc.apn_pass,        sizeof(nc.apn_pass));
    bool host_present  = json_get_str(json,"mqttHost",      nc.mqtt_host,       sizeof(nc.mqtt_host));
    bool user_present  = json_get_str(json,"mqttUser",      nc.mqtt_user,       sizeof(nc.mqtt_user));
    bool pass_present  = json_get_str(json,"mqttPass",      nc.mqtt_pass,       sizeof(nc.mqtt_pass));
    bool topic_present = json_get_str(json,"mqttBaseTopic", nc.mqtt_base_topic, sizeof(nc.mqtt_base_topic));
    bool port_present  = json_has_key(json, "mqttPort");
    int  port_val      = port_present ? json_get_int(json, "mqttPort", nc.mqtt_port) : nc.mqtt_port;
    bool tls_present   = json_has_key(json, "mqttTls");
    bool tls_val       = tls_present ? json_get_bool(json, "mqttTls", nc.mqtt_tls) : nc.mqtt_tls;

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
        mqtt_publish(t_data, ok);
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
            mqtt_publish(t_data, err);
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
            mqtt_publish(t_data, err);
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

    char verr[128];
    if (!validate_suffixes(&nc, verr, sizeof(verr))) {
        ESP_LOGW(TAG, "setconfig: rejected — %s", verr);
        char err[320];
        snprintf(err, sizeof(err),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"%s\"}",
                 g_imei, verr);
        mqtt_publish(t_data, err);
        return;
    }

    /* Turn the GNSS engine off cleanly on a live disable rather than leaving
     * it running until the modem's next power-cycle — cheap given the reboot
     * that follows a few lines down anyway. Compared against the OLD g_cfg
     * (not yet overwritten), so this only fires on an actual on->off edge. */
    if (g_cfg.gps_enable && !nc.gps_enable) gps_end();

    g_cfg = nc;
    if (!nvs_config_save(&g_cfg)) {
        /* Not persisted (e.g. NVS partition full) — g_cfg is already updated
         * in RAM, so the device runs the new config until next boot/power
         * loss, but claiming "config_successful" here would be a lie: a
         * restart right now would just reload the stale un-saved NVS
         * content, undoing the change we just told the caller happened.
         * Report the failure and keep running on the in-RAM config instead. */
        char err[128];
        snprintf(err, sizeof(err),
                 "{\"HW_ID\":\"%s\",\"status\":\"config_failed\",\"reason\":\"nvs_save_failed\"}",
                 g_imei);
        mqtt_publish(t_data, err);
        return;
    }

    char ack[128];
    snprintf(ack, sizeof(ack),
             "{\"HW_ID\":\"%s\",\"status\":\"config_successful\",\"fw\":\"%s\"}",
             g_imei, FW_VERSION);
    mqtt_publish(t_data, ack);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

/* ── MQTT message router ──────────────────────────────────────────────── */
static void handle_msg(const char *topic, const char *payload) {
    if (strcmp(topic, t_update) == 0 || strcmp(topic, t_update_bcast) == 0) {
        /* Guard against a retained (or repeatedly re-published) update
         * message re-triggering a download that already failed: a RETAINED
         * bad URL is re-delivered on EVERY reconnect, and each attempt tears
         * down MQTT for ~30s — without this guard the device does nothing
         * but loop FOTA attempts forever. A different URL clears the block;
         * a successful FOTA restarts the device anyway. */
        static char s_fota_blocked[600] = "";
        char fota_msg[128];
        /* strcmp, not strncmp(payload, blocked, strlen(blocked)): that form
         * returns 0 whenever payload merely STARTS WITH the blocked URL, so a
         * genuinely different, never-tried firmware whose URL shares a prefix
         * — ".../fw_v1.0.bin" vs ".../fw_v1.0.bin.bak", or "/v1.1" vs "/v1.10"
         * — was refused as "known bad". The intent (and the comment above) is
         * an EXACT repeat of the URL that just failed. */
        if (s_fota_blocked[0] && strcmp(payload, s_fota_blocked) == 0) {
            ESP_LOGW(TAG, "FOTA: identical URL already failed — skipping "
                          "(clear the retained message or publish a new URL)");
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_skip_known_bad\"}", g_imei);
            mqtt_publish(t_data, fota_msg);
            return;
        }

        /* Last message before MQTT is torn down for the HTTP download — the
         * device goes radio-silent until fota_successful/fota_failed. */
        snprintf(fota_msg, sizeof(fota_msg),
                 "{\"HW_ID\":\"%s\",\"status\":\"fota_initiated\"}", g_imei);
        mqtt_publish(t_data, fota_msg);
        vTaskDelay(pdMS_TO_TICKS(500));

        /* FOTA start — keep blinking continuously through the whole download
         * + flash (MQTT is down, so there are no per-push blinks meanwhile). */
        led_blink_start();

        /* Free the modem's data session for HTTP — MQTT and QHTTP can't
         * share it cleanly. Reconnect afterwards if the download fails. */
        mqtt_disconnect();
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* WDT stays armed across FOTA too. Every long wait inside
         * fota_perform() feeds it: the HTTP GET / READFILE waits go through
         * modem_read_until() (feeds per read iteration), and fota_flash()'s
         * two raw UART loops now feed explicitly. The whole flash loop is
         * additionally bounded by FOTA_FLASH_MAX_MS. A download that genuinely
         * wedges the UART driver should reboot the device, not strand it. */
        char fota_reason[48] = "unknown";
        bool fota_ok = fota_perform(payload, t_data, fota_reason, sizeof(fota_reason));

        led_blink_stop();   /* FOTA end — back to per-push blinks / reboot */

        if (fota_ok) {
            /* Boot partition already set — reconnect just to notify, then
             * restart into the new image regardless of publish success.
             * Try twice: the first QMTOPEN right after the HTTP session
             * teardown can transiently fail (seen in the field: 0,1). */
            for (int i = 0; i < 2; i++) {
                if (mqtt_connect()) {
                    snprintf(fota_msg, sizeof(fota_msg),
                             "{\"HW_ID\":\"%s\",\"status\":\"fota_successful\"}", g_imei);
                    mqtt_publish(t_data, fota_msg);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
            esp_restart();
        }

        /* Download failed — remember this exact payload so a retained copy
         * of it can't re-trigger the loop, then bring MQTT back up. */
        strncpy(s_fota_blocked, payload, sizeof(s_fota_blocked) - 1);
        s_fota_blocked[sizeof(s_fota_blocked) - 1] = '\0';

        if (mqtt_connect()) {
            subscribe_all();
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\",\"reason\":\"%s\"}",
                     g_imei, fota_reason);
            mqtt_publish(t_data, fota_msg);
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
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        if (strcmp(topic, t_rsp_bs_in[i]) == 0) {
            g_rsp_bs_cmd[i].pending = true;
            /* strncpy does not NUL-terminate when src is >= n bytes — these
             * are global statics (BSS-zeroed at boot) so the final byte
             * happens to already be '\0' on a fresh device, but that's an
             * accident of initial state, not a guarantee after it's been
             * written once. Terminate explicitly. */
            strncpy(g_rsp_bs_cmd[i].cmd_hex, payload, sizeof(g_rsp_bs_cmd[i].cmd_hex)-1);
            g_rsp_bs_cmd[i].cmd_hex[sizeof(g_rsp_bs_cmd[i].cmd_hex)-1] = '\0';
            strncpy(g_rsp_bs_cmd[i].resp_topic, t_rsp_bs_out[i], sizeof(g_rsp_bs_cmd[i].resp_topic)-1);
            g_rsp_bs_cmd[i].resp_topic[sizeof(g_rsp_bs_cmd[i].resp_topic)-1] = '\0';
            return;
        }
        if (strcmp(topic, t_kf_bs_in[i]) == 0) {
            g_kf_bs_cmd[i].pending = true;
            strncpy(g_kf_bs_cmd[i].cmd_hex, payload, sizeof(g_kf_bs_cmd[i].cmd_hex)-1);
            g_kf_bs_cmd[i].cmd_hex[sizeof(g_kf_bs_cmd[i].cmd_hex)-1] = '\0';
            strncpy(g_kf_bs_cmd[i].resp_topic, t_kf_bs_out[i], sizeof(g_kf_bs_cmd[i].resp_topic)-1);
            g_kf_bs_cmd[i].resp_topic[sizeof(g_kf_bs_cmd[i].resp_topic)-1] = '\0';
            return;
        }
    }
}

/* ── Execute pending remote commands ──────────────────────────────────── */
static void execute_pending(void) {
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        if (g_rsp_bs_cmd[i].pending) {
            g_rsp_bs_cmd[i].pending = false;
            char resp_hex[256] = {0};
            rs485_set_8e1_9600();
            bool ok = du_exec_bluesky_rsp(g_rsp_bs_cmd[i].cmd_hex, resp_hex, sizeof(resp_hex));
            mqtt_publish(g_rsp_bs_cmd[i].resp_topic, ok ? resp_hex : "404");
        }
        if (g_kf_bs_cmd[i].pending) {
            g_kf_bs_cmd[i].pending = false;
            char resp_hex[256] = {0};
            rs485_set_8e1_9600();
            bool ok = du_exec_bluesky_kfactor(g_kf_bs_cmd[i].cmd_hex, resp_hex, sizeof(resp_hex));
            mqtt_publish(g_kf_bs_cmd[i].resp_topic, ok ? resp_hex : "404");
        }
    }
}

/* ── Post-OTA check ───────────────────────────────────────────────────── */
static void check_post_ota(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t   state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK
        && state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "{\"HW_ID\":\"%s\",\"status\":\"fota_completed\",\"fw\":\"%s\"}",
                 g_imei, FW_VERSION);
        mqtt_publish(t_data, msg);
    }
}

/* ── Poll dispensers ──────────────────────────────────────────────────────
 * One nozzle at a time: read it, timestamp it, publish it, move on.
 *
 * Two deliberate changes from the previous per-DU batching:
 *
 *  1. Each nozzle is published the moment its own read completes instead of
 *     waiting for its partner. On a 2-DU station that gets NZ1's data to the
 *     cloud ~10 s sooner and NZ3's ~10 s sooner. It also matches the BlueSky
 *     protocol doc (which specifies one JSON message per nozzle) and what the
 *     fleet's older firmware already emits, and it is the shape an
 *     event-driven capture needs — two nozzles never finish a sale together.
 *
 *  2. TS is taken PER NOZZLE, at the instant that nozzle was read, rather than
 *     once at cycle start and stamped onto everything. Previously the last
 *     nozzle of a cycle carried a timestamp ~40 s older than its own reading.
 *     That matters because the cloud reconstructs each sale by differencing
 *     consecutive totalizer samples and orders those samples by TS. This is
 *     only affordable because timestamps now come from the RTC (see modem.h) —
 *     as a modem AT round-trip it would have cost 4 extra calls per cycle.
 *
 * poll_incoming_mqtt() runs after every publish rather than every DU, so a
 * queued setconfig/FOTA/rsp command is serviced within about one nozzle read
 * of arriving. */
static void poll_dispensers(void) {
    /* One nozzle with its 3 fields at DU_RESP_MAX=32 bytes each is ~400 bytes
     * worst case, so 1024 keeps well over 2x headroom on top of the clamp_pos
     * guard in du_build_nozzle_payload() and stays well under the modem's
     * ~1400-byte QMTPUBEX ceiling (MQTT_MODEM_PAYLOAD_SAFE_MAX). */
    static char       payload[1024];
    static DuNozzle_t nz;
    char ts[32];

    /* K-factor is a near-constant calibration value, so it is read only every
     * KFACTOR_POLL_INTERVAL_S; on every other round du_poll_bluesky_nozzle
     * carries the last good value forward. Gated here once per round (applied
     * to every nozzle this round) so the tight per-nozzle totalizer cadence is
     * not spent re-reading it. 0 = never read yet, so the first round after
     * boot reads it. esp_timer_get_time() is a monotonic microsecond counter,
     * so plain subtraction needs no wrap handling (same pattern as the radar/
     * genset rate limits in run_poll_cycle). */
    static int64_t last_kfactor_us = 0;
    int64_t now_us = esp_timer_get_time();
    bool read_kf = (last_kfactor_us == 0 ||
                    now_us - last_kfactor_us >= (int64_t)KFACTOR_POLL_INTERVAL_S * 1000000LL);
    if (read_kf) last_kfactor_us = now_us;

    /* AND across every nozzle polled THIS cycle rather than last-one-wins — a
     * dead nozzle hidden behind a healthy one would otherwise report ok. */
    bool all_ok = true;

    for (int i = 0; i < g_cfg.du_bluesky_count; i++) {
        for (int half = 0; half < 2; half++) {
            int     label = i * 2 + half + 1;  /* global nozzle number, 1-based */
            uint8_t addr  = (uint8_t)label;    /* BlueSky address == label       */

            /* Re-assert the mode per nozzle rather than once per DU:
             * poll_incoming_mqtt() below can execute a queued rsp/kfactor
             * command, which reconfigures the UART for its own transaction.
             * Setting it here keeps this loop correct without depending on
             * what that path happened to leave behind. */
            rs485_set_8e1_9600();
            du_poll_bluesky_nozzle(addr, &nz, read_kf);
            rs485_set_8n1(9600);

            rtc_timestamp(ts, sizeof(ts));   /* stamped at the read, not the cycle */
            du_build_nozzle_payload(&nz, label, g_hw_bs[i], ts,
                                    payload, sizeof(payload));

            bool ok = mqtt_publish(t_data, payload);
            if (ok) g_diag.pub_ok++; else g_diag.pub_fail++;
            all_ok &= ok;

            esp_task_wdt_reset();
            poll_incoming_mqtt();  /* stay responsive to setconfig/FOTA mid-poll */
        }
    }
    g_diag.du_ok = all_ok;
}

/* ── Poll radar ───────────────────────────────────────────────────────── */
/* Returns this tank's health (read valid AND publish accepted) so the caller
 * can AND it across all tanks — see the du_ok note in poll_dispensers(). */
static bool poll_one_tank(int tank_idx, bool is_ztech, const char *ts) {
    /* Local, not static: 576 bytes easily fits this shallow frame, and static
     * storage shared between the Ztech/Pantech calls in the same poll cycle
     * is fine only because they're called sequentially today — local removes
     * that assumption at zero runtime cost. */
    uint8_t rx[64];
    char    payload[512];
    int  rx_len = 0;
    /* Up to RADAR_READ_RETRIES attempts before giving up this cycle. Radar is
     * polled infrequently (RADAR_POLL_INTERVAL_S) and its level changes slowly,
     * so a short retry burst costs almost nothing and recovers a reading lost
     * to a single bus collision / line-noise frame instead of publishing 404.
     * Uniform across Ztech/Pantech (and any future radar type) since the retry
     * wraps the type dispatch, not a specific driver. */
    bool ok = false;
    for (int attempt = 1; attempt <= RADAR_READ_RETRIES && !ok; attempt++) {
        if (attempt > 1) vTaskDelay(pdMS_TO_TICKS(100));
        ok = is_ztech ? radar_ztech_read(tank_idx, rx, &rx_len)
                      : radar_pantech_read(tank_idx, rx, &rx_len);
    }
    const char *hw = is_ztech ? g_hw_ztech[tank_idx] : g_hw_pantech[tank_idx];

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

static void poll_radar(const char *ts) {
    rs485_set_8n1(9600);
    bool all_ok = true;
    for (int i = 0; i < g_cfg.ztech_count; i++)   all_ok &= poll_one_tank(i, true,  ts);
    for (int i = 0; i < g_cfg.pantech_count; i++) all_ok &= poll_one_tank(i, false, ts);
    /* AND across every tank polled this cycle rather than last-one-wins — a
     * dead tank hidden behind a healthy one would otherwise report ok. If no
     * radar is configured this stays true (poll_radar isn't called then). */
    g_diag.radar_ok = all_ok;
}

/* ── Poll genset ──────────────────────────────────────────────────────── */
static void poll_genset(const char *ts) {
    static GensetData_t gd;
    /* Raw-hex genset payloads are small — the largest frame (SGC120: 163 bytes
     * → ~490 hex chars) is well under 600 bytes — but the buffer is left at 4096
     * so the builder's own ceiling check (MQTT_MODEM_PAYLOAD_SAFE_MAX), not the
     * buffer, is what bounds the payload, and a future wider read block can't
     * silently truncate here. */
    static char payload[4096];
    rs485_set_8n1(g_cfg.genset_baud);
    /* Up to GENSET_READ_RETRIES attempts before giving up this cycle.
     * genset_poll() memsets gd on entry, so a failed attempt leaves it cleared
     * and a later success fully repopulates it. It returns true only when EVERY
     * read the model defines succeeded; a partial result still publishes what it
     * got, with "404" for the read that failed. */
    bool ok = false;
    for (int attempt = 1; attempt <= GENSET_READ_RETRIES && !ok; attempt++) {
        if (attempt > 1) vTaskDelay(pdMS_TO_TICKS(100));
        ok = genset_poll(g_cfg.genset_slave, g_cfg.genset_model, &gd);
    }
    /* Restore the dispenser bus mode before publishing — the RS485 work is done
     * and mqtt_publish() can block for a second or more per message. */
    rs485_set_8e1_9600();

    /* Message 1 of up to 2: the measurement block. Each Modbus read is published
     * as its own message per the DG integration spec, both under the genset
     * HW_ID and sharing this cycle's timestamp. */
    genset_build_registers_payload(&gd, g_hw_genset, ts, payload, sizeof(payload));
    bool pub = mqtt_publish(t_data, payload);
    if (pub) g_diag.pub_ok++; else g_diag.pub_fail++;

    /* Message 2: the DG status block, for every model that defines one (all but
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
}

/* ── Poll GPS ─────────────────────────────────────────────────────────────
 * Bare IMEI as HW_ID, no suffix — GPS describes the gateway's own fixed
 * location, not a specific piece of equipment behind it (same reasoning as
 * status/diagnostics/sysmon). Not on the RS485 bus, so no mode-switch calls
 * like poll_genset()'s rs485_set_8n1/8e1_9600. */
static void poll_gps(const char *ts) {
    static GpsFix_t fix;
    static char payload[160];  /* {"HW_ID":..,"gps":{"lat":..,"lon":..},"TS":..} ~100 bytes worst case */
    bool ok = gps_poll(&fix);
    gps_build_payload(&fix, g_imei, ts, payload, sizeof(payload));
    bool pub = mqtt_publish(t_data, payload);
    if (pub) g_diag.pub_ok++; else g_diag.pub_fail++;
    g_diag.gps_ok = ok && pub;
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

    for (int i = 0; i < MQTT_RECONNECT_RETRIES; i++) {
        if (i > 0) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_task_wdt_reset();
        }
        if (mqtt_connect()) {
            subscribe_all();
            esp_task_wdt_reset();
            return;
        }
    }

    ESP_LOGW(TAG, "[RECOVER L2] Rebuilding network ...");
    for (int i = 0; i < NET_BRINGUP_RETRIES; i++) {
        if (i > 0) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            esp_task_wdt_reset();
        }
        if (network_init() && mqtt_connect()) {
            subscribe_all();
            ESP_LOGI(TAG, "[RECOVER L2] OK");
            esp_task_wdt_reset();
            return;
        }
    }

    ESP_LOGE(TAG, "[RECOVER L3] Power-cycling modem ...");
    for (int i = 0; i < MODEM_PWRCYCLE_MAX; i++) {
        modem_hard_reset();
        esp_task_wdt_reset();
        if (network_init() && mqtt_connect()) {
            subscribe_all();
            ESP_LOGI(TAG, "[RECOVER L3] OK on cycle %d", i + 1);
            esp_task_wdt_reset();
            return;
        }
    }

    ESP_LOGE(TAG, "[RECOVER L4] All recovery levels failed — restarting");
    esp_restart();
}

/* Drain any pending incoming MQTT messages (setconfig, FOTA trigger, DU
 * commands) without blocking on a reconnect attempt — used to interleave
 * responsiveness checks into run_poll_cycle(), which can otherwise block
 * for 30+ seconds (BlueSky busy-retry waits, RS485 timeouts) with no
 * chance to notice an incoming publish until the cycle fully completes.
 *
 * Also executes any rsp/kfactor command handle_msg() just marked pending,
 * right here — not just once per 30s cycle. This runs every ~1s (via
 * sleep_with_poll) as well as between every poll-cycle step, so a remote
 * command now gets serviced within about a second of arriving instead of
 * waiting up to a full poll cycle. execute_pending() sets its own RS485
 * mode before each transaction, so it's safe to call from any point in the
 * poll cycle regardless of which device's mode is currently active. */
static void poll_incoming_mqtt(void) {
    static char s_topic[TOPIC_MAX];
    static char s_payload[2048];
    while (mqtt_poll(s_topic, sizeof(s_topic), s_payload, sizeof(s_payload)))
        handle_msg(s_topic, s_payload);
    execute_pending();
}

/* ── Main poll cycle ──────────────────────────────────────────────────── */
static void run_poll_cycle(void) {
    /* Low-heap guard — reboot before heap exhaustion causes undefined behaviour */
    if (esp_get_free_heap_size() < 20480) {
        ESP_LOGE(TAG, "Critical heap (%lu B) — rebooting", (unsigned long)esp_get_free_heap_size());
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
    if (g_cfg.du_bluesky_count > 0) poll_dispensers();
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
    static int64_t last_radar_us  = 0;
    static int64_t last_genset_us = 0;

    if (g_cfg.ztech_count > 0 || g_cfg.pantech_count > 0) {
        int64_t now_us = esp_timer_get_time();
        if (last_radar_us == 0 ||
            now_us - last_radar_us >= (int64_t)RADAR_POLL_INTERVAL_S * 1000000LL) {
            last_radar_us = now_us;
            rtc_timestamp(ts, sizeof(ts));
            poll_radar(ts);
        }
    }
    esp_task_wdt_reset();
    poll_incoming_mqtt();

    if (g_cfg.genset_type == GENSET_SEDEMAC) {
        int64_t now_us = esp_timer_get_time();
        if (last_genset_us == 0 ||
            now_us - last_genset_us >= (int64_t)GENSET_POLL_INTERVAL_S * 1000000LL) {
            last_genset_us = now_us;
            rtc_timestamp(ts, sizeof(ts));
            poll_genset(ts);
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

    /* GPS runs on an even slower cadence than radar — the gateway is a
     * stationary asset, its position never changes after install. Gated
     * solely by gps_enable, which defaults to false: this is the one line
     * that keeps the whole feature dead for every gateway that hasn't been
     * explicitly enabled from the cloud. */
    static int gps_cycle = 0;
    if (g_cfg.gps_enable) {
        if (gps_cycle == 0) {
            rtc_timestamp(ts, sizeof(ts));
            poll_gps(ts);
        }
        gps_cycle = (gps_cycle + 1) % GPS_CYCLE_INTERVAL;
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

    if (++g_diag_cycle >= DIAG_CYCLE_INTERVAL) {
        rtc_timestamp(ts, sizeof(ts));
        /* One combined diagnostics report (diagnostics + system monitor merged
         * into diagnostics.c). diag_publish() reads the window counters and
         * clears them on a confirmed publish — the single report-and-clear point. */
        diag_publish(g_imei, ts, t_data);
        g_diag_cycle = 0;
    }

    diag_pm_release();
}

/* ── Responsive 30 s sleep ────────────────────────────────────────────── */
static void sleep_with_poll(int seconds) {
    for (int i = 0; i < seconds; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    rs485_init();
    di_uart_init();
    diag_init();
    nvs_config_load(&g_cfg);
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
        net_ok = network_init();
    }
    if (!net_ok) {
        /* A pending APN change that can't attach would otherwise reboot-loop
         * forever on the same bad APN (no WiFi fallback, no field access) —
         * revert to the last-known-good APN before rebooting. A transient
         * outage during an APN trial also lands here and reverts; that's the
         * safe direction (we only keep a new APN once it actually attaches).*/
        if (g_cfg.conn_pending) {
            conn_rollback("network_bringup");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        ESP_LOGE(TAG, "Network failed after %d attempts — reboot", NET_BRINGUP_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
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
     * fix that). A true no-op when gps_enable is false, which is the
     * default and normal state for most gateways. */
    gps_init(g_cfg.gps_enable);

    if (!modem_get_imei(g_imei, sizeof(g_imei)))
        strncpy(g_imei, "000000000000000", sizeof(g_imei));
    ESP_LOGI(TAG, "IMEI: %s", g_imei);

    build_hw_ids();
    build_topics();
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
        vTaskDelay(pdMS_TO_TICKS(5000));
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
        /* Network is up but no broker session — if a connectivity change is
         * on trial, the broker half is the likely culprit; revert the whole
         * staged delta and reboot into the last-known-good settings rather
         * than retry a bad broker forever. */
        if (g_cfg.conn_pending) {
            conn_rollback("broker_unreachable");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        ESP_LOGE(TAG, "MQTT failed after recovery — reboot");
        esp_restart();
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
     * actual job. A crash inside poll_dispensers()/poll_genset()/poll_radar()
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
        if (mqtt_publish(t_data, rb)) {
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

    bool unconfigured = (g_cfg.du_bluesky_count == 0
                         && g_cfg.ztech_count == 0
                         && g_cfg.pantech_count == 0
                         && g_cfg.genset_type == GENSET_OFF
                         && !g_cfg.di_enable);
    char ready_msg[128];
    snprintf(ready_msg, sizeof(ready_msg),
             "{\"HW_ID\":\"%s\",\"status\":\"%s\",\"fw\":\"%s\"}",
             g_imei,
             unconfigured ? "unconfigured" : "ready",
             FW_VERSION);
    mqtt_publish(t_data, ready_msg);

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
             * rollback and tell the cloud fota_completed. */
            check_post_ota();
            post_ota_checked = true;
        }

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
