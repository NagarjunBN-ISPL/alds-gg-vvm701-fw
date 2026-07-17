#include "nvs_config.h"
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "nvs_cfg";
#define NS "iot_cfg"

void nvs_config_defaults(DeviceConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->du_bluesky_count = 0;
    cfg->ztech_count      = 0;
    cfg->pantech_count    = 0;
    cfg->genset_type      = GENSET_OFF;
    cfg->genset_model     = GENSET_MODEL_GC1211;
    cfg->genset_slave     = GENSET_SLAVE_DEFAULT;
    cfg->genset_baud      = 9600;
    cfg->di_enable        = 0;
    cfg->gps_enable       = false;
    /* sfx all empty strings */

    /* Connectivity defaults = the compile-time fallback in config.h. Primary
     * and _bak start identical so a fresh device (never setconfig'd) has
     * nothing to "roll back" to but itself. */
    strncpy(cfg->apn,                 MODEM_APN,       sizeof(cfg->apn) - 1);
    strncpy(cfg->apn_user,            MODEM_APN_USER,  sizeof(cfg->apn_user) - 1);
    strncpy(cfg->apn_pass,            MODEM_APN_PASS,  sizeof(cfg->apn_pass) - 1);
    strncpy(cfg->mqtt_host,           MQTT_HOST,       sizeof(cfg->mqtt_host) - 1);
    strncpy(cfg->mqtt_user,           MQTT_USER,       sizeof(cfg->mqtt_user) - 1);
    strncpy(cfg->mqtt_pass,           MQTT_PASS,       sizeof(cfg->mqtt_pass) - 1);
    strncpy(cfg->mqtt_base_topic,     MQTT_TOPIC_BASE, sizeof(cfg->mqtt_base_topic) - 1);
    cfg->mqtt_port = MQTT_PORT;
    cfg->mqtt_tls  = true;   /* TLS on by default — matches the hardcoded past */
    strncpy(cfg->apn_bak,             cfg->apn,             sizeof(cfg->apn_bak) - 1);
    strncpy(cfg->apn_user_bak,        cfg->apn_user,        sizeof(cfg->apn_user_bak) - 1);
    strncpy(cfg->apn_pass_bak,        cfg->apn_pass,        sizeof(cfg->apn_pass_bak) - 1);
    strncpy(cfg->mqtt_host_bak,       cfg->mqtt_host,       sizeof(cfg->mqtt_host_bak) - 1);
    strncpy(cfg->mqtt_user_bak,       cfg->mqtt_user,       sizeof(cfg->mqtt_user_bak) - 1);
    strncpy(cfg->mqtt_pass_bak,       cfg->mqtt_pass,       sizeof(cfg->mqtt_pass_bak) - 1);
    strncpy(cfg->mqtt_base_topic_bak, cfg->mqtt_base_topic, sizeof(cfg->mqtt_base_topic_bak) - 1);
    cfg->mqtt_port_bak        = cfg->mqtt_port;
    cfg->mqtt_tls_bak         = cfg->mqtt_tls;
    cfg->conn_pending         = false;
    cfg->conn_rollback_notify = false;
}

bool nvs_config_load(DeviceConfig *cfg) {
    nvs_config_defaults(cfg);

    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "No config in NVS, using defaults");
        return false;
    }

    uint8_t  u8;
    uint16_t u16;
    uint32_t u32;
    size_t   sz;

#define LOAD_U8(key, field) \
    if (nvs_get_u8(h, key, &u8) == ESP_OK) cfg->field = u8

#define LOAD_U32(key, field) \
    if (nvs_get_u32(h, key, &u32) == ESP_OK) cfg->field = u32

#define LOAD_U16(key, field) \
    if (nvs_get_u16(h, key, &u16) == ESP_OK) cfg->field = u16

#define LOAD_STR(key, field) \
    sz = sizeof(cfg->field); \
    nvs_get_str(h, key, cfg->field, &sz)

    LOAD_U8 ("du_bs_cnt",    du_bluesky_count);
    LOAD_U8 ("ztech_cnt",    ztech_count);
    LOAD_U8 ("pantech_cnt",  pantech_count);
    LOAD_U8 ("genset_type",  genset_type);
    LOAD_U8 ("genset_model", genset_model);
    LOAD_U8 ("genset_slave", genset_slave);
    LOAD_U8 ("di_enable",    di_enable);
    /* No key in NVS yet on any gateway flashed before this feature existed —
     * nvs_get_u8 simply misses and leaves the nvs_config_defaults() false
     * untouched, so upgrading an already-deployed gateway's firmware is a
     * behavioral no-op. */
    LOAD_U8 ("gps_enable",   gps_enable);
    LOAD_U32("genset_baud",  genset_baud);
    LOAD_STR("sfx_genset",   sfx_genset);
    LOAD_STR("sfx_di",       sfx_di);

    char key[16];
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        snprintf(key, sizeof(key), "sfx_bs%d", i);
        sz = sizeof(cfg->sfx_bs[i]);
        nvs_get_str(h, key, cfg->sfx_bs[i], &sz);
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        snprintf(key, sizeof(key), "sfx_zt%d", i);
        sz = sizeof(cfg->sfx_ztech[i]);
        nvs_get_str(h, key, cfg->sfx_ztech[i], &sz);
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        snprintf(key, sizeof(key), "sfx_pt%d", i);
        sz = sizeof(cfg->sfx_pantech[i]);
        nvs_get_str(h, key, cfg->sfx_pantech[i], &sz);
    }

    LOAD_STR("apn",           apn);
    LOAD_STR("apn_user",      apn_user);
    LOAD_STR("apn_pass",      apn_pass);
    LOAD_STR("mqtt_host",     mqtt_host);
    LOAD_U16("mqtt_port",     mqtt_port);
    LOAD_STR("mqtt_user",     mqtt_user);
    LOAD_STR("mqtt_pass",     mqtt_pass);
    LOAD_STR("mqtt_topic",    mqtt_base_topic);
    LOAD_U8 ("mqtt_tls",      mqtt_tls);
    LOAD_STR("apn_b",         apn_bak);
    LOAD_STR("apn_user_b",    apn_user_bak);
    LOAD_STR("apn_pass_b",    apn_pass_bak);
    LOAD_STR("mqtt_host_b",   mqtt_host_bak);
    LOAD_U16("mqtt_port_b",   mqtt_port_bak);
    LOAD_STR("mqtt_user_b",   mqtt_user_bak);
    LOAD_STR("mqtt_pass_b",   mqtt_pass_bak);
    LOAD_STR("mqtt_topic_b",  mqtt_base_topic_bak);
    LOAD_U8 ("mqtt_tls_b",    mqtt_tls_bak);
    LOAD_U8 ("conn_pend",     conn_pending);
    LOAD_U8 ("conn_rbn",      conn_rollback_notify);

    nvs_close(h);

    /* Clamp all fields to valid ranges — guards against NVS corruption.
     * Ztech (slaves 4,5) and Pantech (slaves 6,7) occupy fixed, non-
     * overlapping ranges, so each type is capped independently at RADAR_MAX. */
    if (cfg->du_bluesky_count > DU_BLUESKY_MAX) cfg->du_bluesky_count = DU_BLUESKY_MAX;
    if (cfg->ztech_count      > RADAR_MAX)      cfg->ztech_count      = RADAR_MAX;
    if (cfg->pantech_count    > RADAR_MAX)      cfg->pantech_count    = RADAR_MAX;
    if (cfg->genset_type > GENSET_SEDEMAC) cfg->genset_type = GENSET_OFF;
    if (cfg->genset_model > GENSET_MODEL_MAX) cfg->genset_model = GENSET_MODEL_GC1211;
    if (cfg->genset_slave < 1 || cfg->genset_slave > 247) cfg->genset_slave = GENSET_SLAVE_DEFAULT;
    /* Baud whitelist — 0 (or any junk) would divide-by-zero the UART clock
     * divider in rs485_set_8n1() and panic-loop the device. */
    if (cfg->genset_baud != 4800 && cfg->genset_baud != 9600 && cfg->genset_baud != 19200)
        cfg->genset_baud = 9600;
    /* Genset slave must not sit inside an active radar slave range (their
     * Modbus polls are identical apart from the slave byte — see
     * parse_setconfig). Corrupt/legacy NVS falls back to the DG default. */
    if (cfg->genset_type != GENSET_OFF) {
        if ((cfg->ztech_count   > 0 && cfg->genset_slave >= ZTECH_SLAVE_BASE
             && cfg->genset_slave < ZTECH_SLAVE_BASE + cfg->ztech_count) ||
            (cfg->pantech_count > 0 && cfg->genset_slave >= PANTECH_SLAVE_BASE
             && cfg->genset_slave < PANTECH_SLAVE_BASE + cfg->pantech_count)) {
            ESP_LOGW(TAG, "genset_slave %u collides with radar range — reset to %d",
                     cfg->genset_slave, GENSET_SLAVE_DEFAULT);
            cfg->genset_slave = GENSET_SLAVE_DEFAULT;
        }
    }
    if (cfg->di_enable  > 0x0F)            cfg->di_enable   = 0;

    /* A corrupt/truncated connectivity field would otherwise brick the
     * device with no way to recover remotely (no WiFi fallback, no physical
     * access at a field station) — fall back to the compiled-in defaults
     * rather than try to attach with a blank APN or connect to a blank host
     * / out-of-range port. */
    if (cfg->apn[0] == '\0') {
        ESP_LOGW(TAG, "APN invalid/missing — reverting to compiled-in default");
        strncpy(cfg->apn,      MODEM_APN,      sizeof(cfg->apn) - 1);
        strncpy(cfg->apn_user, MODEM_APN_USER, sizeof(cfg->apn_user) - 1);
        strncpy(cfg->apn_pass, MODEM_APN_PASS, sizeof(cfg->apn_pass) - 1);
    }
    if (cfg->mqtt_host[0] == '\0' || cfg->mqtt_port == 0 || cfg->mqtt_base_topic[0] == '\0') {
        ESP_LOGW(TAG, "MQTT config invalid/missing — reverting to compiled-in defaults");
        strncpy(cfg->mqtt_host,       MQTT_HOST,       sizeof(cfg->mqtt_host) - 1);
        strncpy(cfg->mqtt_user,       MQTT_USER,       sizeof(cfg->mqtt_user) - 1);
        strncpy(cfg->mqtt_pass,       MQTT_PASS,       sizeof(cfg->mqtt_pass) - 1);
        strncpy(cfg->mqtt_base_topic, MQTT_TOPIC_BASE, sizeof(cfg->mqtt_base_topic) - 1);
        cfg->mqtt_port = MQTT_PORT;
    }
    if (cfg->apn_bak[0] == '\0' || cfg->mqtt_host_bak[0] == '\0' ||
        cfg->mqtt_port_bak == 0 || cfg->mqtt_base_topic_bak[0] == '\0') {
        strncpy(cfg->apn_bak,             cfg->apn,             sizeof(cfg->apn_bak) - 1);
        strncpy(cfg->apn_user_bak,        cfg->apn_user,        sizeof(cfg->apn_user_bak) - 1);
        strncpy(cfg->apn_pass_bak,        cfg->apn_pass,        sizeof(cfg->apn_pass_bak) - 1);
        strncpy(cfg->mqtt_host_bak,       cfg->mqtt_host,       sizeof(cfg->mqtt_host_bak) - 1);
        strncpy(cfg->mqtt_user_bak,       cfg->mqtt_user,       sizeof(cfg->mqtt_user_bak) - 1);
        strncpy(cfg->mqtt_pass_bak,       cfg->mqtt_pass,       sizeof(cfg->mqtt_pass_bak) - 1);
        strncpy(cfg->mqtt_base_topic_bak, cfg->mqtt_base_topic, sizeof(cfg->mqtt_base_topic_bak) - 1);
        cfg->mqtt_port_bak = cfg->mqtt_port;
        cfg->mqtt_tls_bak  = cfg->mqtt_tls;
        cfg->conn_pending  = false;
    }

    ESP_LOGI(TAG, "Config loaded: du_bs=%d ztech=%d pantech=%d genset=%d model=%d slave=%d di=0x%02x",
             cfg->du_bluesky_count, cfg->ztech_count, cfg->pantech_count,
             cfg->genset_type, cfg->genset_model, cfg->genset_slave, cfg->di_enable);
    return true;
}

bool nvs_config_save(const DeviceConfig *cfg) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return false;

    /* nvs_commit() only reports whether the commit-as-a-whole succeeded — a
     * single nvs_set_* failing earlier (e.g. NVS partition full) still lets
     * commit succeed for everything else, silently leaving that one key
     * stale with no indication until the next boot loads inconsistent
     * config. Track failures explicitly so we can report and reject. */
    bool ok = true;
    ok &= (nvs_set_u8 (h, "du_bs_cnt",    cfg->du_bluesky_count) == ESP_OK);
    ok &= (nvs_set_u8 (h, "ztech_cnt",    cfg->ztech_count)      == ESP_OK);
    ok &= (nvs_set_u8 (h, "pantech_cnt",  cfg->pantech_count)    == ESP_OK);
    ok &= (nvs_set_u8 (h, "genset_type",  cfg->genset_type)      == ESP_OK);
    ok &= (nvs_set_u8 (h, "genset_model", cfg->genset_model)     == ESP_OK);
    ok &= (nvs_set_u8 (h, "genset_slave", cfg->genset_slave)     == ESP_OK);
    ok &= (nvs_set_u8 (h, "di_enable",    cfg->di_enable)        == ESP_OK);
    ok &= (nvs_set_u8 (h, "gps_enable",   cfg->gps_enable)       == ESP_OK);
    ok &= (nvs_set_u32(h, "genset_baud",  cfg->genset_baud)      == ESP_OK);
    ok &= (nvs_set_str(h, "sfx_genset",   cfg->sfx_genset)       == ESP_OK);
    ok &= (nvs_set_str(h, "sfx_di",       cfg->sfx_di)           == ESP_OK);

    ok &= (nvs_set_str(h, "apn",          cfg->apn)                 == ESP_OK);
    ok &= (nvs_set_str(h, "apn_user",     cfg->apn_user)            == ESP_OK);
    ok &= (nvs_set_str(h, "apn_pass",     cfg->apn_pass)            == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_host",    cfg->mqtt_host)           == ESP_OK);
    ok &= (nvs_set_u16(h, "mqtt_port",    cfg->mqtt_port)           == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_user",    cfg->mqtt_user)           == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_pass",    cfg->mqtt_pass)           == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_topic",   cfg->mqtt_base_topic)     == ESP_OK);
    ok &= (nvs_set_u8 (h, "mqtt_tls",     cfg->mqtt_tls)            == ESP_OK);
    ok &= (nvs_set_str(h, "apn_b",        cfg->apn_bak)             == ESP_OK);
    ok &= (nvs_set_str(h, "apn_user_b",   cfg->apn_user_bak)        == ESP_OK);
    ok &= (nvs_set_str(h, "apn_pass_b",   cfg->apn_pass_bak)        == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_host_b",  cfg->mqtt_host_bak)       == ESP_OK);
    ok &= (nvs_set_u16(h, "mqtt_port_b",  cfg->mqtt_port_bak)       == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_user_b",  cfg->mqtt_user_bak)       == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_pass_b",  cfg->mqtt_pass_bak)       == ESP_OK);
    ok &= (nvs_set_str(h, "mqtt_topic_b", cfg->mqtt_base_topic_bak) == ESP_OK);
    ok &= (nvs_set_u8 (h, "mqtt_tls_b",   cfg->mqtt_tls_bak)        == ESP_OK);
    ok &= (nvs_set_u8 (h, "conn_pend",    cfg->conn_pending)        == ESP_OK);
    ok &= (nvs_set_u8 (h, "conn_rbn",     cfg->conn_rollback_notify)== ESP_OK);
    if (!ok) ESP_LOGE(TAG, "one or more core config keys failed to write");

    char key[16];
    for (int i = 0; i < DU_BLUESKY_MAX; i++) {
        snprintf(key, sizeof(key), "sfx_bs%d", i);
        if (nvs_set_str(h, key, cfg->sfx_bs[i]) != ESP_OK) {
            ESP_LOGE(TAG, "failed to write %s", key);
            ok = false;
        }
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        snprintf(key, sizeof(key), "sfx_zt%d", i);
        if (nvs_set_str(h, key, cfg->sfx_ztech[i]) != ESP_OK) {
            ESP_LOGE(TAG, "failed to write %s", key);
            ok = false;
        }
    }
    for (int i = 0; i < RADAR_MAX; i++) {
        snprintf(key, sizeof(key), "sfx_pt%d", i);
        if (nvs_set_str(h, key, cfg->sfx_pantech[i]) != ESP_OK) {
            ESP_LOGE(TAG, "failed to write %s", key);
            ok = false;
        }
    }

    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "NVS commit failed"); return false; }
    if (!ok) { ESP_LOGE(TAG, "Config saved with errors — one or more keys not updated"); return false; }
    ESP_LOGI(TAG, "Config saved");
    return true;
}

void nvs_config_factory_reset(void) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Factory reset done");
}

uint32_t nvs_boot_count_bump(void) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return 0;
    uint32_t count = 0;
    nvs_get_u32(h, "boot_count", &count);   /* stays 0 if key absent */
    count++;
    if (nvs_set_u32(h, "boot_count", count) == ESP_OK)
        nvs_commit(h);
    nvs_close(h);
    return count;
}
