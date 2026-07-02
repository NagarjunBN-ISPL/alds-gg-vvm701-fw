#include "nvs_config.h"
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "nvs_cfg";
#define NS "iot_cfg"

void nvs_config_defaults(DeviceConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->du_bluesky_count = 0;
    cfg->du_yenen_count   = 0;
    cfg->ztech_count      = 0;
    cfg->pantech_count    = 0;
    cfg->genset_type      = GENSET_OFF;
    cfg->genset_slave     = 1;
    cfg->genset_baud      = 9600;
    cfg->di_enable        = 0;
    /* sfx all empty strings */
}

bool nvs_config_load(DeviceConfig *cfg) {
    nvs_config_defaults(cfg);

    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "No config in NVS, using defaults");
        return false;
    }

    uint8_t  u8;
    uint32_t u32;
    size_t   sz;

#define LOAD_U8(key, field) \
    if (nvs_get_u8(h, key, &u8) == ESP_OK) cfg->field = u8

#define LOAD_U32(key, field) \
    if (nvs_get_u32(h, key, &u32) == ESP_OK) cfg->field = u32

#define LOAD_STR(key, field) \
    sz = sizeof(cfg->field); \
    nvs_get_str(h, key, cfg->field, &sz)

    LOAD_U8 ("du_bs_cnt",    du_bluesky_count);
    LOAD_U8 ("du_yn_cnt",    du_yenen_count);
    LOAD_U8 ("ztech_cnt",    ztech_count);
    LOAD_U8 ("pantech_cnt",  pantech_count);
    LOAD_U8 ("genset_type",  genset_type);
    LOAD_U8 ("genset_slave", genset_slave);
    LOAD_U8 ("di_enable",    di_enable);
    LOAD_U32("genset_baud",  genset_baud);
    LOAD_STR("sfx_genset",   sfx_genset);
    LOAD_STR("sfx_di",       sfx_di);

    nvs_close(h);

    /* Clamp all fields to valid ranges — guards against NVS corruption */
    if (cfg->du_bluesky_count > DU_BLUESKY_MAX) cfg->du_bluesky_count = DU_BLUESKY_MAX;
    if (cfg->du_yenen_count   > DU_YENEN_MAX)   cfg->du_yenen_count   = DU_YENEN_MAX;
    if (cfg->ztech_count      > RADAR_MAX)      cfg->ztech_count      = RADAR_MAX;
    if (cfg->pantech_count    > RADAR_MAX)      cfg->pantech_count    = RADAR_MAX;
    /* Ztech (slave 4..) and Pantech (slave 7..) share one Modbus bus —
     * combined total must stay <= RADAR_MAX so their address ranges
     * never overlap. Trim Pantech first. */
    if ((int)cfg->ztech_count + (int)cfg->pantech_count > RADAR_MAX) {
        int room = RADAR_MAX - cfg->ztech_count;
        cfg->pantech_count = (room > 0) ? (uint8_t)room : 0;
    }
    if (cfg->genset_type > GENSET_SEDEMAC) cfg->genset_type = GENSET_OFF;
    if (cfg->genset_slave < 1 || cfg->genset_slave > 247) cfg->genset_slave = 1;
    if (cfg->di_enable  > 0x0F)            cfg->di_enable   = 0;

    ESP_LOGI(TAG, "Config loaded: du_bs=%d du_yn=%d ztech=%d pantech=%d genset=%d di=0x%02x",
             cfg->du_bluesky_count, cfg->du_yenen_count,
             cfg->ztech_count, cfg->pantech_count, cfg->genset_type, cfg->di_enable);
    return true;
}

bool nvs_config_save(const DeviceConfig *cfg) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return false;

    nvs_set_u8 (h, "du_bs_cnt",    cfg->du_bluesky_count);
    nvs_set_u8 (h, "du_yn_cnt",    cfg->du_yenen_count);
    nvs_set_u8 (h, "ztech_cnt",    cfg->ztech_count);
    nvs_set_u8 (h, "pantech_cnt",  cfg->pantech_count);
    nvs_set_u8 (h, "genset_type",  cfg->genset_type);
    nvs_set_u8 (h, "genset_slave", cfg->genset_slave);
    nvs_set_u8 (h, "di_enable",    cfg->di_enable);
    nvs_set_u32(h, "genset_baud",  cfg->genset_baud);
    nvs_set_str(h, "sfx_genset",   cfg->sfx_genset);
    nvs_set_str(h, "sfx_di",       cfg->sfx_di);

    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "NVS commit failed"); return false; }
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
