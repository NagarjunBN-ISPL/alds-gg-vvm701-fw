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
#include "diagnostics.h"
#include "sysmon.h"

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

static const char *TAG = "main";

/* ── Global state ─────────────────────────────────────────────────────── */
static char        g_imei[20];
static DeviceConfig g_cfg;

/* BlueSky and Yenen are independent address spaces; Ztech/Pantech likewise
 * get independent label spaces even though they share the RS485 bus. */
static char g_hw_bs[DU_BLUESKY_MAX][24];
static char g_hw_yn[DU_YENEN_MAX][24];
static char g_hw_ztech[RADAR_MAX][24];
static char g_hw_pantech[RADAR_MAX][24];
static char g_hw_genset[24], g_hw_di[24];

#define TOPIC_MAX 200
static char t_data[TOPIC_MAX];
static char t_diag[TOPIC_MAX];
static char t_update[TOPIC_MAX];
static char t_update_bcast[TOPIC_MAX];
static char t_setcfg[TOPIC_MAX];
static char t_rsp_bs_in[DU_BLUESKY_MAX][TOPIC_MAX], t_rsp_bs_out[DU_BLUESKY_MAX][TOPIC_MAX];
static char t_kf_bs_in[DU_BLUESKY_MAX][TOPIC_MAX],  t_kf_bs_out[DU_BLUESKY_MAX][TOPIC_MAX];
static char t_rsp_yn_in[DU_YENEN_MAX][TOPIC_MAX],   t_rsp_yn_out[DU_YENEN_MAX][TOPIC_MAX];
static char t_sysmon[TOPIC_MAX];

typedef struct {
    bool pending;
    char cmd_hex[256];
    char resp_topic[TOPIC_MAX];
} PendingCmd_t;

static PendingCmd_t g_rsp_bs_cmd[DU_BLUESKY_MAX];
static PendingCmd_t g_kf_bs_cmd[DU_BLUESKY_MAX];
static PendingCmd_t g_rsp_yn_cmd[DU_YENEN_MAX];
static int g_diag_cycle = 0;

/* ── LED ──────────────────────────────────────────────────────────────── */
static void led_on(void)  { gpio_set_level(LED_PIN, 1); }
static void led_off(void) { gpio_set_level(LED_PIN, 0); }

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
    du_rs485_set_bluesky_mode();
}

/* ── Build identifiers ────────────────────────────────────────────────── */
static void build_hw_ids(void) {
    for (int i = 0; i < DU_BLUESKY_MAX; i++)
        snprintf(g_hw_bs[i], sizeof(g_hw_bs[i]), "%sB%d", g_imei, i + 1);
    for (int i = 0; i < DU_YENEN_MAX; i++)
        snprintf(g_hw_yn[i], sizeof(g_hw_yn[i]), "%sY%d", g_imei, i + 1);
    for (int i = 0; i < RADAR_MAX; i++) {
        snprintf(g_hw_ztech[i],   sizeof(g_hw_ztech[i]),   "%sZ%d", g_imei, i + 1);
        snprintf(g_hw_pantech[i], sizeof(g_hw_pantech[i]), "%sP%d", g_imei, i + 1);
    }
    snprintf(g_hw_genset, sizeof(g_hw_genset), "%s%s", g_imei, g_cfg.sfx_genset);
    snprintf(g_hw_di,     sizeof(g_hw_di),     "%s%s", g_imei, g_cfg.sfx_di);
}

static void build_topics(void) {
    snprintf(t_data,        TOPIC_MAX, "%s/%s/data",       MQTT_TOPIC_BASE, g_imei);
    snprintf(t_diag,        TOPIC_MAX, "%s/diagnostics",   MQTT_TOPIC_BASE);
    snprintf(t_update,      TOPIC_MAX, "%s/%s/update",     MQTT_TOPIC_BASE, g_imei);
    snprintf(t_update_bcast,TOPIC_MAX, "%s/update",        MQTT_TOPIC_BASE);
    snprintf(t_setcfg,      TOPIC_MAX, "%s/%s/setconfig",  MQTT_TOPIC_BASE, g_imei);
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        snprintf(t_rsp_bs_in[i],  TOPIC_MAX, "%s/rsp/imei/%s/in",      MQTT_TOPIC_BASE, g_hw_bs[i]);
        snprintf(t_rsp_bs_out[i], TOPIC_MAX, "%s/rsp/imei/%s/out",     MQTT_TOPIC_BASE, g_hw_bs[i]);
        snprintf(t_kf_bs_in[i],   TOPIC_MAX, "%s/kfactor/imei/%s/in",  MQTT_TOPIC_BASE, g_hw_bs[i]);
        snprintf(t_kf_bs_out[i],  TOPIC_MAX, "%s/kfactor/imei/%s/out", MQTT_TOPIC_BASE, g_hw_bs[i]);
    }
    for (int i = 0; i < DU_YENEN_MAX; i++) {
        snprintf(t_rsp_yn_in[i],  TOPIC_MAX, "%s/rsp/imei/%s/in",  MQTT_TOPIC_BASE, g_hw_yn[i]);
        snprintf(t_rsp_yn_out[i], TOPIC_MAX, "%s/rsp/imei/%s/out", MQTT_TOPIC_BASE, g_hw_yn[i]);
    }
    snprintf(t_sysmon,      TOPIC_MAX, "%s/sysmon",              MQTT_TOPIC_BASE);
}

static void subscribe_all(void) {
    mqtt_subscribe(t_update);
    mqtt_subscribe(t_update_bcast);
    mqtt_subscribe(t_setcfg);
    for (int i = 0; i < g_cfg.du_bluesky_count; i++) {
        mqtt_subscribe(t_rsp_bs_in[i]);
        mqtt_subscribe(t_kf_bs_in[i]);
    }
    for (int i = 0; i < g_cfg.du_yenen_count; i++) {
        mqtt_subscribe(t_rsp_yn_in[i]);
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

/* ── setconfig ────────────────────────────────────────────────────────── */
static void parse_setconfig(const char *json) {
    if (strstr(json, "\"cmd\":\"restart\"")) {
        ESP_LOGI(TAG, "Remote restart");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
    if (strstr(json, "\"cmd\":\"factory_reset\"")) {
        ESP_LOGI(TAG, "Factory reset");
        nvs_config_factory_reset();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    int du_bs = json_get_int(json,"duBlueskyCount", g_cfg.du_bluesky_count);
    int du_yn = json_get_int(json,"duYenenCount",   g_cfg.du_yenen_count);
    int ztech = json_get_int(json,"ztechCount",     g_cfg.ztech_count);
    int pantech = json_get_int(json,"pantechCount", g_cfg.pantech_count);

    if (du_bs < 0) du_bs = 0;
    if (du_bs > DU_BLUESKY_MAX) du_bs = DU_BLUESKY_MAX;
    if (du_yn < 0) du_yn = 0;
    if (du_yn > DU_YENEN_MAX) du_yn = DU_YENEN_MAX;
    if (ztech < 0) ztech = 0;
    if (ztech > RADAR_MAX) ztech = RADAR_MAX;
    if (pantech < 0) pantech = 0;
    if (pantech > RADAR_MAX) pantech = RADAR_MAX;
    /* Ztech (slave 4..) and Pantech (slave 7..) share one Modbus bus —
     * combined total must stay <= RADAR_MAX so their ranges never overlap. */
    if (ztech + pantech > RADAR_MAX) {
        int room = RADAR_MAX - ztech;
        pantech = (room > 0) ? room : 0;
    }

    g_cfg.du_bluesky_count = (uint8_t)du_bs;
    g_cfg.du_yenen_count   = (uint8_t)du_yn;
    g_cfg.ztech_count      = (uint8_t)ztech;
    g_cfg.pantech_count    = (uint8_t)pantech;
    g_cfg.genset_type   = (uint8_t) json_get_int(json,"gensetType", g_cfg.genset_type);
    g_cfg.genset_slave  = (uint8_t) json_get_int(json,"gensetSlave",g_cfg.genset_slave);
    g_cfg.genset_baud   = (uint32_t)json_get_int(json,"gensetBaud", (int)g_cfg.genset_baud);
    g_cfg.di_enable     = (uint8_t) json_get_int(json,"diEnable",   g_cfg.di_enable);

    json_get_str(json,"sfxGenset",g_cfg.sfx_genset, sizeof(g_cfg.sfx_genset));
    json_get_str(json,"sfxDI",    g_cfg.sfx_di,     sizeof(g_cfg.sfx_di));

    nvs_config_save(&g_cfg);

    char ack[128];
    snprintf(ack, sizeof(ack),
             "{\"status\":\"config_applied\",\"fw\":\"%s\"}", FW_VERSION);
    mqtt_publish(t_data, ack);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

/* ── MQTT message router ──────────────────────────────────────────────── */
static void handle_msg(const char *topic, const char *payload) {
    if (strcmp(topic, t_update) == 0 || strcmp(topic, t_update_bcast) == 0) {
        char fota_msg[128];
        snprintf(fota_msg, sizeof(fota_msg),
                 "{\"HW_ID\":\"%s\",\"status\":\"fota_start\"}", g_imei);
        mqtt_publish(t_data, fota_msg);
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_task_wdt_delete(NULL);
        if (fota_perform(payload, t_data)) {
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_success\"}", g_imei);
            mqtt_publish(t_data, fota_msg);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            snprintf(fota_msg, sizeof(fota_msg),
                     "{\"HW_ID\":\"%s\",\"status\":\"fota_failed\"}", g_imei);
            mqtt_publish(t_data, fota_msg);
        }
        esp_task_wdt_add(NULL);
        return;
    }
    if (strcmp(topic, t_setcfg) == 0) {
        ESP_LOGI(TAG, "setconfig payload: [%s]", payload);
        parse_setconfig(payload);
        return;
    }
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        if (strcmp(topic, t_rsp_bs_in[i]) == 0) {
            g_rsp_bs_cmd[i].pending = true;
            strncpy(g_rsp_bs_cmd[i].cmd_hex, payload, sizeof(g_rsp_bs_cmd[i].cmd_hex)-1);
            strncpy(g_rsp_bs_cmd[i].resp_topic, t_rsp_bs_out[i], sizeof(g_rsp_bs_cmd[i].resp_topic)-1);
            return;
        }
        if (strcmp(topic, t_kf_bs_in[i]) == 0) {
            g_kf_bs_cmd[i].pending = true;
            strncpy(g_kf_bs_cmd[i].cmd_hex, payload, sizeof(g_kf_bs_cmd[i].cmd_hex)-1);
            strncpy(g_kf_bs_cmd[i].resp_topic, t_kf_bs_out[i], sizeof(g_kf_bs_cmd[i].resp_topic)-1);
            return;
        }
    }
    for (int i = 0; i < DU_YENEN_MAX; i++) {
        if (strcmp(topic, t_rsp_yn_in[i]) == 0) {
            g_rsp_yn_cmd[i].pending = true;
            strncpy(g_rsp_yn_cmd[i].cmd_hex, payload, sizeof(g_rsp_yn_cmd[i].cmd_hex)-1);
            strncpy(g_rsp_yn_cmd[i].resp_topic, t_rsp_yn_out[i], sizeof(g_rsp_yn_cmd[i].resp_topic)-1);
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
    for (int i = 0; i < DU_YENEN_MAX; i++) {
        if (g_rsp_yn_cmd[i].pending) {
            g_rsp_yn_cmd[i].pending = false;
            char resp_hex[256] = {0};
            du_rs485_set_yenen_mode();
            bool ok = du_exec_bluesky_rsp(g_rsp_yn_cmd[i].cmd_hex, resp_hex, sizeof(resp_hex));
            mqtt_publish(g_rsp_yn_cmd[i].resp_topic, ok ? resp_hex : "404");
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
                 "{\"HW_ID\":\"%s\",\"status\":\"fota_done\",\"fw\":\"%s\"}",
                 g_imei, FW_VERSION);
        mqtt_publish(t_data, msg);
    }
}

/* ── Poll dispensers ──────────────────────────────────────────────────── */
static void poll_dispensers(const char *ts) {
    static char     payload[1024];
    static DuNozzle_t nz_a, nz_b;

    for (int i = 0; i < g_cfg.du_bluesky_count; i++) {
        rs485_set_8e1_9600();
        du_poll_bluesky_unit(i, &nz_a, &nz_b);
        rs485_set_8n1(9600);
        du_build_payload(&nz_a, &nz_b, i * 2 + 1, i * 2 + 2,
                         false, g_hw_bs[i], ts, payload, sizeof(payload));
        led_on();
        bool ok = mqtt_publish(t_data, payload);
        led_off();
        if (ok) g_diag.pub_ok++; else g_diag.pub_fail++;
        g_diag.du_ok = ok;
    }

    for (int i = 0; i < g_cfg.du_yenen_count; i++) {
        du_rs485_set_yenen_mode();
        du_poll_yenen_unit(i, &nz_a, &nz_b);
        du_build_payload(&nz_a, &nz_b, i * 2 + 1, i * 2 + 2,
                         true, g_hw_yn[i], ts, payload, sizeof(payload));
        led_on();
        bool ok = mqtt_publish(t_data, payload);
        led_off();
        if (ok) g_diag.pub_ok++; else g_diag.pub_fail++;
        g_diag.du_ok = ok;
    }
}

/* ── Poll radar ───────────────────────────────────────────────────────── */
static void poll_one_tank(int tank_idx, bool is_ztech, const char *ts) {
    static uint8_t rx[64];
    static char    payload[512];
    int  rx_len = 0;
    bool ok = is_ztech ? radar_ztech_read(tank_idx, rx, &rx_len)
                        : radar_pantech_read(tank_idx, rx, &rx_len);
    const char *hw = is_ztech ? g_hw_ztech[tank_idx] : g_hw_pantech[tank_idx];

    if (!ok || rx_len == 0) {
        snprintf(payload, sizeof(payload),
                 "{\"HW_ID\":\"%s\",\"tanks\":{\"%d\":{\"data\":\"404\"}},\"TS\":\"%s\"}",
                 hw, tank_idx + 1, ts);
    } else {
        char hex[256] = {0};
        int  pos = 0;
        for (int i = 0; i < rx_len; i++) {
            if (i > 0) hex[pos++] = ',';
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x", rx[i]);
        }
        snprintf(payload, sizeof(payload),
                 "{\"HW_ID\":\"%s\",\"tanks\":{\"%d\":{\"data\":\"%s\"}},\"TS\":\"%s\"}",
                 hw, tank_idx + 1, hex, ts);
    }
    led_on();
    bool pub = mqtt_publish(t_data, payload);
    led_off();
    if (pub) g_diag.pub_ok++; else g_diag.pub_fail++;
    g_diag.radar_ok = ok && pub;
}

static void poll_radar(const char *ts) {
    rs485_set_8n1(9600);
    for (int i = 0; i < g_cfg.ztech_count; i++)   poll_one_tank(i, true,  ts);
    for (int i = 0; i < g_cfg.pantech_count; i++) poll_one_tank(i, false, ts);
}

/* ── Poll genset ──────────────────────────────────────────────────────── */
static void poll_genset(const char *ts) {
    static GensetData_t gd;
    static char payload[4096];  /* genset with all alarms can exceed 2 KB */
    rs485_set_8n1(g_cfg.genset_baud);
    bool ok = genset_poll(g_cfg.genset_slave, &gd);
    genset_build_payload(&gd, g_hw_genset, ts, payload, sizeof(payload));
    rs485_set_8e1_9600();
    led_on();
    bool pub = mqtt_publish(t_data, payload);
    led_off();
    if (pub) g_diag.pub_ok++; else g_diag.pub_fail++;
    g_diag.genset_ok = ok && pub;
}

/* ── Timestamp hour extractor ─────────────────────────────────────────── */
/* ts format: "2026-07-01 02:00:15"  →  returns 2 */
static int ts_hour(const char *ts) {
    if (!ts || strlen(ts) < 13) return -1;
    int h = (ts[11] - '0') * 10 + (ts[12] - '0');
    return (h >= 0 && h <= 23) ? h : -1;
}

/* ── Daily scheduled reboot at 2 AM ──────────────────────────────────── */
static bool s_reboot_armed = false;

static void check_daily_reboot(const char *ts) {
    int h = ts_hour(ts);
    if (h < 0) return;
    if (h >= 3) s_reboot_armed = true;          /* re-arm once per day after 3 AM */
    if (h == 2 && s_reboot_armed) {
        s_reboot_armed = false;
        ESP_LOGI(TAG, "Scheduled daily reboot");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
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
    esp_task_wdt_delete(NULL);

    for (int i = 0; i < MQTT_RECONNECT_RETRIES; i++) {
        if (i > 0) vTaskDelay(pdMS_TO_TICKS(5000));
        if (mqtt_connect()) {
            subscribe_all();
            esp_task_wdt_add(NULL);
            esp_task_wdt_reset();
            return;
        }
    }

    ESP_LOGW(TAG, "[RECOVER L2] Rebuilding network ...");
    for (int i = 0; i < NET_BRINGUP_RETRIES; i++) {
        if (i > 0) vTaskDelay(pdMS_TO_TICKS(10000));
        if (network_init() && mqtt_connect()) {
            subscribe_all();
            ESP_LOGI(TAG, "[RECOVER L2] OK");
            esp_task_wdt_add(NULL);
            esp_task_wdt_reset();
            return;
        }
    }

    ESP_LOGE(TAG, "[RECOVER L3] Power-cycling modem ...");
    for (int i = 0; i < MODEM_PWRCYCLE_MAX; i++) {
        modem_hard_reset();
        if (network_init() && mqtt_connect()) {
            subscribe_all();
            ESP_LOGI(TAG, "[RECOVER L3] OK on cycle %d", i + 1);
            esp_task_wdt_add(NULL);
            esp_task_wdt_reset();
            return;
        }
    }

    ESP_LOGE(TAG, "[RECOVER L4] All recovery levels failed — restarting");
    esp_restart();
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
    modem_get_timestamp(ts, sizeof(ts));

    check_daily_reboot(ts);  /* scheduled reboot at 2 AM */

    sysmon_pm_acquire();

    if (g_cfg.du_bluesky_count > 0 || g_cfg.du_yenen_count > 0) poll_dispensers(ts);
    if (g_cfg.ztech_count > 0 || g_cfg.pantech_count > 0)       poll_radar(ts);
    if (g_cfg.genset_type == GENSET_SEDEMAC)                    poll_genset(ts);
    execute_pending();
    if (g_cfg.di_enable)
        di_publish_buffer(t_data, g_hw_di, ts, g_cfg.di_enable);

    diag_record_loop_time(esp_timer_get_time() - loop_start);

    if (++g_diag_cycle >= DIAG_CYCLE_INTERVAL) {
        modem_get_timestamp(ts, sizeof(ts));
        diag_publish(g_imei, ts, t_diag);
        sysmon_publish(g_imei, ts, t_sysmon);
        g_diag_cycle = 0;
    }

    sysmon_pm_release();
}

/* ── Responsive 30 s sleep ────────────────────────────────────────────── */
static void sleep_with_poll(int seconds) {
    /* Static buffers — keep off stack to prevent overflow */
    static char s_topic[TOPIC_MAX];
    static char s_payload[2048];
    for (int i = 0; i < seconds; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_task_wdt_reset();
        mqtt_reconnect_if_needed();     /* heal broken connection before polling */
        if (mqtt_poll(s_topic, sizeof(s_topic), s_payload, sizeof(s_payload)))
            handle_msg(s_topic, s_payload);
    }
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

    modem_uart_init();
    rs485_init();
    di_uart_init();
    diag_init();
    sysmon_init();
    nvs_config_load(&g_cfg);
    di_set_enable(g_cfg.di_enable);

    ESP_LOGI(TAG, "FW %s — powering modem...", FW_VERSION);

    /* WDT add — suspend around long ops */
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    if (!modem_power_on()) {
        ESP_LOGE(TAG, "Modem failed — reboot");
        vTaskDelay(pdMS_TO_TICKS(30000));
        esp_restart();
    }

    esp_task_wdt_delete(NULL);
    if (!network_init()) {
        ESP_LOGE(TAG, "Network failed — reboot");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    if (!modem_get_imei(g_imei, sizeof(g_imei)))
        strncpy(g_imei, "000000000000000", sizeof(g_imei));
    ESP_LOGI(TAG, "IMEI: %s", g_imei);

    build_hw_ids();
    build_topics();
    mqtt_init(g_imei);

    esp_task_wdt_delete(NULL);
    for (int attempts = 0; attempts < 5; attempts++) {
        if (mqtt_connect()) break;
        ESP_LOGW(TAG, "MQTT retry %d", attempts + 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if (!mqtt_is_connected()) {
        ESP_LOGE(TAG, "MQTT failed — reboot");
        esp_restart();
    }
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    subscribe_all();
    check_post_ota();

    if (g_cfg.di_enable)
        xTaskCreatePinnedToCore(di_task, "di_task", 4096, NULL, 5, NULL, 1);

    bool unconfigured = (g_cfg.du_bluesky_count == 0
                         && g_cfg.du_yenen_count == 0
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

    for (;;) {
        esp_task_wdt_reset();
        run_poll_cycle();
        sleep_with_poll(PUBLISH_INTERVAL_MS / 1000);
    }
}
