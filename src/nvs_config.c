#include "nvs_config.h"
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "nvs_cfg";
#define NS "iot_cfg"

/* A stored HW_ID suffix must be empty or exactly 2 DIGITS, 01..99 —
 * setconfig enforces this on the way in (validate_suffixes, main.c), and this
 * enforces it on the way out of flash: corrupt NVS, or a value written by an
 * older firmware with the alphanumeric rule, would otherwise corrupt every
 * JSON payload built with that HW_ID or (for a dispenser or radar tank) send
 * the poll to no address at all. Clearing falls back to the auto value. */
static bool sfx_is_number(const char *s) {
    return strlen(s) == 2 && isdigit((unsigned char)s[0]) && isdigit((unsigned char)s[1]) &&
           !(s[0] == '0' && s[1] == '0');
}

static void clamp_sfx(char *s) {
    if (!s[0]) return;
    if (!sfx_is_number(s)) {
        ESP_LOGW(TAG, "invalid stored suffix [%s] — cleared to auto-fallback", s);
        s[0] = '\0';
    }
}

int du_number_of(const DeviceConfig *c, int i) {
    if (!c || i < 0 || i >= DU_MAX_DISPENSERS) return 0;
    if (!c->du_sfx[i][0]) return i + 1;
    if (!sfx_is_number(c->du_sfx[i])) return 0;
    return (c->du_sfx[i][0] - '0') * 10 + (c->du_sfx[i][1] - '0');
}

void nvs_config_defaults(DeviceConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->du_count = 0;
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
    cfg->mqtt_tls  = MQTT_TLS_DEFAULT;
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

/* ── Product guard ────────────────────────────────────────────────────────
 * Rationale in config.h. Stored under its own key rather than inside the config
 * blob, deliberately: the blob is a raw struct dump, so its layout is only
 * meaningful to the product that wrote it — the tag has to be readable WITHOUT
 * trusting that layout, or it could not do its job.
 *
 * Key ABSENT is not a mismatch. Every gateway provisioned before this existed
 * reads back nothing, and wiping those would deconfigure the whole fleet on the
 * first firmware carrying this feature. They are stamped and left alone. */
#define PRODUCT_KEY "product"

/* The CNG sibling's own namespace (cng-cpil-vvm701-fw, nvs_store.c). Never
 * parsed here — its layout belongs to that build — but ERASED WHOLE when this
 * firmware finds the board carrying another product's stamp. Rationale: a
 * gateway that lived as CNG at station A, came here as LPG, and later goes
 * back to CNG at station B would otherwise boot that CNG image straight into
 * station A's dispenser plan (its own realm check passes: the config really is
 * CNG's). Wiping the namespace makes it come up "unprovisioned" instead, which
 * is the same honest state this firmware chooses for itself. Erasing a whole
 * namespace needs no knowledge of what is inside it. */
#define SIBLING_NS_CNG "cng"

static void erase_sibling_namespaces(void) {
    nvs_handle_t h;
    if (nvs_open(SIBLING_NS_CNG, NVS_READWRITE, &h) != ESP_OK) return;   /* absent = nothing to do */
    if (nvs_erase_all(h) == ESP_OK && nvs_commit(h) == ESP_OK)
        ESP_LOGW(TAG, "erased the sibling product's namespace [%s] as well — a later "
                      "reflash to that firmware comes up unprovisioned rather than "
                      "on a stale station plan", SIBLING_NS_CNG);
    nvs_close(h);
}

static bool s_product_wiped = false;
static char s_reverted_foreign[40] = "";

void nvs_config_note_reverted_foreign_image(const char *project_name) {
    snprintf(s_reverted_foreign, sizeof(s_reverted_foreign), "%s", project_name ? project_name : "");
}

static void product_check(void) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;

    char   stored[40] = {0};
    size_t sz         = sizeof(stored);
    esp_err_t err     = nvs_get_str(h, PRODUCT_KEY, stored, &sz);

    if (err == ESP_OK && strcmp(stored, PRODUCT_ID) != 0 && s_reverted_foreign[0]) {
        /* The other product's image was tried and thrown away by the
         * bootloader; we are the slot it fell back to. Its stamp is the only
         * thing it left behind — the blob is still ours. Keep it. */
        ESP_LOGW(TAG, "NVS is stamped [%s] by an image (%s) the bootloader just ROLLED BACK "
                      "— that product never took over this gateway; restamping [%s] and "
                      "keeping the stored configuration", stored, s_reverted_foreign, PRODUCT_ID);
        err = ESP_ERR_NVS_NOT_FOUND;                  /* fall into the stamp path below */
    }

    if (err == ESP_OK && strcmp(stored, PRODUCT_ID) != 0) {
        ESP_LOGE(TAG, "NVS belongs to product [%s], this firmware is [%s] — "
                      "erasing the stored configuration; the gateway will come "
                      "up UNCONFIGURED and needs a setconfig for the equipment "
                      "actually installed at this site", stored, PRODUCT_ID);
        nvs_erase_all(h);
        s_product_wiped = true;
        erase_sibling_namespaces();
    } else if (err == ESP_OK) {
        nvs_close(h);
        return;                      /* matches — the overwhelmingly normal path */
    } else {
        ESP_LOGI(TAG, "no product tag in NVS — stamping [%s] and keeping the "
                      "existing configuration", PRODUCT_ID);
    }

    if (nvs_set_str(h, PRODUCT_KEY, PRODUCT_ID) == ESP_OK)
        nvs_commit(h);
    else
        ESP_LOGE(TAG, "could not stamp the product tag — it will be retried next boot");
    nvs_close(h);
}

bool nvs_config_product_wiped(void) { return s_product_wiped; }

/* ── bool fields from a raw blob ──────────────────────────────────────────
 * A _Bool that holds anything other than 0 or 1 is undefined behaviour to even
 * READ, not merely a surprising value: the compiler is entitled to assume the
 * byte is 0/1 and generate code that misbehaves for anything else. The blob is
 * a raw struct image, so a foreign, truncated or bit-rotted one lands arbitrary
 * bytes in gps_enable, mqtt_tls and friends — caught by UBSan on 2026-09-02
 * fuzzing this loader with random blobs ("load of value 46, not valid for
 * _Bool" at the mqtt_tls_bak assignment).
 *
 * The read therefore has to go through an unsigned char, which is always
 * defined, before anything else in the firmware touches the field. */
static void normalize_bool(bool *b) {
    unsigned char raw;
    memcpy(&raw, b, sizeof(raw));
    *b = (raw != 0);
}

static void normalize_bools(DeviceConfig *cfg) {
    normalize_bool(&cfg->gps_enable);
    normalize_bool(&cfg->mqtt_tls);
    normalize_bool(&cfg->mqtt_tls_bak);
    normalize_bool(&cfg->conn_pending);
    normalize_bool(&cfg->conn_rollback_notify);
}

/* Guarantee every string field is NUL-terminated after a blob load. The blob is
 * a raw struct image: one written by a different build, or damaged in flash, can
 * leave an array with no terminator, and every later strlen/strcmp/%s on it then
 * runs into the next field. The product guard above makes that unlikely rather
 * than impossible — a same-product blob damaged by bit-rot reaches here too — so
 * terminate unconditionally. Costs a handful of stores once per boot. */
static void terminate_strings(DeviceConfig *cfg) {
    cfg->apn[sizeof(cfg->apn) - 1]                         = '\0';
    cfg->apn_user[sizeof(cfg->apn_user) - 1]               = '\0';
    cfg->apn_pass[sizeof(cfg->apn_pass) - 1]               = '\0';
    cfg->mqtt_host[sizeof(cfg->mqtt_host) - 1]             = '\0';
    cfg->mqtt_user[sizeof(cfg->mqtt_user) - 1]             = '\0';
    cfg->mqtt_pass[sizeof(cfg->mqtt_pass) - 1]             = '\0';
    cfg->mqtt_base_topic[sizeof(cfg->mqtt_base_topic) - 1] = '\0';
    cfg->apn_bak[sizeof(cfg->apn_bak) - 1]                         = '\0';
    cfg->apn_user_bak[sizeof(cfg->apn_user_bak) - 1]               = '\0';
    cfg->apn_pass_bak[sizeof(cfg->apn_pass_bak) - 1]               = '\0';
    cfg->mqtt_host_bak[sizeof(cfg->mqtt_host_bak) - 1]             = '\0';
    cfg->mqtt_user_bak[sizeof(cfg->mqtt_user_bak) - 1]             = '\0';
    cfg->mqtt_pass_bak[sizeof(cfg->mqtt_pass_bak) - 1]             = '\0';
    cfg->mqtt_base_topic_bak[sizeof(cfg->mqtt_base_topic_bak) - 1] = '\0';
    cfg->sfx_genset[sizeof(cfg->sfx_genset) - 1] = '\0';
    cfg->sfx_di[sizeof(cfg->sfx_di) - 1]         = '\0';
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) cfg->du_sfx[i][sizeof(cfg->du_sfx[i]) - 1] = '\0';
    for (int i = 0; i < RADAR_MAX; i++) {
        cfg->sfx_ztech[i][sizeof(cfg->sfx_ztech[i]) - 1]     = '\0';
        cfg->sfx_pantech[i][sizeof(cfg->sfx_pantech[i]) - 1] = '\0';
        cfg->reserved_sfx[i][0] = '\0';
    }
}

bool nvs_config_load(DeviceConfig *cfg) {
    nvs_config_defaults(cfg);

    /* Before anything is read: does this NVS even belong to this product? */
    product_check();

    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "No config in NVS, using defaults");
        return false;
    }

    /* Single-blob config: one nvs_set_blob is atomic, so a brownout can no
     * longer land half a connectivity change (new broker committed, trial flag
     * not) — the failure mode the per-key writer could only narrow.
     *
     * Accepting a blob SMALLER than the current struct is deliberate and is
     * what keeps this FOTA-safe: adding a field to DeviceConfig changes
     * sizeof(), and an exact-size check would reject every already-provisioned
     * unit's config on the first firmware that grows the struct, silently
     * reverting the whole fleet to compiled-in defaults. Loading the stored
     * prefix into a defaults-initialised struct leaves any new field at its
     * default and preserves everything the site was configured with.
     *
     * The load deliberately does NOT return here. Flow falls through to the
     * clamp block below, so a blob damaged by bit-rot or a partial write gets
     * exactly the same range validation as the legacy key path — without it a
     * corrupt du_bluesky_count would index straight past its array. */
    bool from_blob = false;
    size_t blob_sz = 0;
    if (nvs_get_blob(h, "cfg_blob", NULL, &blob_sz) == ESP_OK &&
        blob_sz > 0 && blob_sz <= sizeof(DeviceConfig)) {
        size_t rd = blob_sz;
        if (nvs_get_blob(h, "cfg_blob", cfg, &rd) == ESP_OK && rd == blob_sz) {
            from_blob = true;
            ESP_LOGI(TAG, "Config loaded from blob (%u of %u bytes)",
                     (unsigned)blob_sz, (unsigned)sizeof(DeviceConfig));
        }
    }
    if (!from_blob)
        ESP_LOGI(TAG, "Blob not found or unusable, falling back to legacy key load");

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

    /* Legacy per-key path. Skipped entirely once a blob exists, otherwise these
     * reads would overwrite the blob we just loaded with whatever stale keys
     * the pre-blob firmware left behind. */
    if (!from_blob) {

    LOAD_U8 ("du_cnt",       du_count);
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
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
        snprintf(key, sizeof(key), "du_mk%d", i);
        LOAD_U8(key, du_make[i]);
        snprintf(key, sizeof(key), "du_sfx%d", i);
        sz = sizeof(cfg->du_sfx[i]);
        nvs_get_str(h, key, cfg->du_sfx[i], &sz);
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

    }   /* end !from_blob legacy path */

    nvs_close(h);

    /* Clamp all fields to valid ranges — guards against NVS corruption.
     * Ztech and Pantech counts are capped independently at RADAR_MAX here. Note
     * their auto slave ranges are ADJACENT, not spaced apart: a ztech_count of 3
     * with any pantech_count > 0 pushes an auto Ztech slave onto Pantech tank 1.
     * Cross-unit Modbus-address and HW_ID-suffix collisions (radar↔radar,
     * radar↔genset) are rejected on the way IN by validate_bus_addresses() /
     * validate_suffixes() (main.c), so a blob that was ever saved by setconfig
     * cannot hold one, and NVS's per-entry CRC guards that blob against silent
     * corruption. This load path re-derives only the DISPENSER plan below (its
     * numbering doubles as bus addressing); it does NOT re-run the radar/genset
     * collision check. If a future path could persist an unvalidated plan, add
     * that re-check here rather than trusting the way-in guard alone. */
    /* Both run BEFORE any clamp below reads a bool or a string. */
    normalize_bools(cfg);
    terminate_strings(cfg);
    if (cfg->du_count > DU_MAX_DISPENSERS) cfg->du_count = DU_MAX_DISPENSERS;
    /* A make outside the enum (bit-rot, or a value from a build that once
     * defined more makes) is not dangerous — du_task simply matches neither
     * branch — but a position the operator counted in duCount and that then
     * silently polls nothing is exactly the "looks healthy" failure the whole
     * clamp block exists to avoid. Fall back to BlueSky, the make every
     * earlier build of this product family defaulted to, and say so. */
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
        if (cfg->du_make[i] > DU_MAKE_MAX) {
            ESP_LOGW(TAG, "DU%d make %u invalid in NVS — treating as bluesky", i + 1, cfg->du_make[i]);
            cfg->du_make[i] = DU_MAKE_BLUESKY;
        }
    }
    if (cfg->ztech_count      > RADAR_MAX)      cfg->ztech_count      = RADAR_MAX;
    if (cfg->pantech_count    > RADAR_MAX)      cfg->pantech_count    = RADAR_MAX;
    cfg->reserved_u8 = 0;
    if (cfg->genset_type > GENSET_SEDEMAC) cfg->genset_type = GENSET_OFF;
    if (cfg->genset_model > GENSET_MODEL_MAX) cfg->genset_model = GENSET_MODEL_GC1211;
    if (cfg->genset_slave < 1 || cfg->genset_slave > 247) cfg->genset_slave = GENSET_SLAVE_DEFAULT;
    /* Baud whitelist — 0 (or any junk) would divide-by-zero the UART clock
     * divider in rs485_set_8n1() and panic-loop the device. */
    if (cfg->genset_baud != 4800 && cfg->genset_baud != 9600 && cfg->genset_baud != 19200)
        cfg->genset_baud = 9600;

    if (cfg->di_enable  > 0x0F)            cfg->di_enable   = 0;
    clamp_sfx(cfg->sfx_genset);
    if (cfg->sfx_genset[0] >= '0' && cfg->sfx_genset[0] <= '9') {
        int gs = atoi(cfg->sfx_genset);
        if (gs > 0 && gs <= 247) cfg->genset_slave = (uint8_t)gs;
    } else if (cfg->genset_slave > 0 && cfg->genset_slave <= 99 && cfg->sfx_genset[0] == '\0') {
        snprintf(cfg->sfx_genset, sizeof(cfg->sfx_genset), "%02u", (unsigned)(cfg->genset_slave % 100));
    }
    clamp_sfx(cfg->sfx_di);
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) clamp_sfx(cfg->du_sfx[i]);
    for (int i = 0; i < RADAR_MAX; i++) {
        clamp_sfx(cfg->sfx_ztech[i]);
        clamp_sfx(cfg->sfx_pantech[i]);
    }

    /* ── Dispenser plan, the way-out guard ─────────────────────────────────
     * Every entry inside du_count must name a dispenser the bus can address
     * (1..DU_MAX_DISPENSERS; Yenen 1..DU_YENEN_MAX — head 6 has no nozzle
     * code) and no two entries may name the same one: two entries on one
     * address would poll one pump twice and publish it under two HW_IDs.
     * parse_setconfig() refuses such a plan on the way in; this catches a blob
     * damaged in flash. The plan is CUT at the first bad entry rather than
     * patched — a guessed address is the "looks healthy, polls the wrong pump"
     * failure this whole block exists to avoid — and the cut is loud. */
    {
        bool used[DU_MAX_DISPENSERS + 1] = {0};
        for (int i = 0; i < cfg->du_count; i++) {
            int n = du_number_of(cfg, i);
            int max_n = (cfg->du_make[i] == DU_MAKE_YENEN) ? DU_YENEN_MAX : DU_MAX_DISPENSERS;
            const char *why = NULL;
            if (n < 1 || n > DU_MAX_DISPENSERS) why = "not addressable";
            else if (n > max_n)                why = "Yenen cannot address head 6";
            else if (used[n])                  why = "already named by an earlier entry";
            if (why) {
                ESP_LOGE(TAG, "DU entry %d names dispenser %d [sfx '%s'] — %s; "
                              "cutting the plan to %d dispenser(s)",
                         i + 1, n, cfg->du_sfx[i], why, i);
                cfg->du_count = (uint8_t)i;
                break;
            }
            used[n] = true;
        }
    }

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

    /* Persist the clamped result as a blob when this boot either migrated from
     * the legacy keys or read a short blob written by an older, smaller
     * DeviceConfig. Both cases are one-shot: a normal blob boot writes nothing,
     * so this costs no flash wear on the 99% path. Writing only AFTER the clamp
     * means a damaged source can never be re-persisted in its damaged form. */
    if (!from_blob || blob_sz != sizeof(DeviceConfig)) {
        ESP_LOGI(TAG, "Persisting config as blob (%s)",
                 from_blob ? "struct grew since last write" : "migrated from legacy keys");
        nvs_handle_t hw;
        if (nvs_open(NS, NVS_READWRITE, &hw) == ESP_OK) {
            if (nvs_set_blob(hw, "cfg_blob", cfg, sizeof(DeviceConfig)) == ESP_OK) {
                nvs_commit(hw);
            }
            nvs_close(hw);
        }
    }
    ESP_LOGI(TAG, "Config loaded: du_cnt=%d ztech=%d pantech=%d "
                  "genset=%d model=%d slave=%d di=0x%02x",
             cfg->du_count,
             cfg->ztech_count, cfg->pantech_count,
             cfg->genset_type, cfg->genset_model, cfg->genset_slave, cfg->di_enable);
    return true;
}

bool nvs_config_save(const DeviceConfig *cfg) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return false;

    if (nvs_set_blob(h, "cfg_blob", cfg, sizeof(DeviceConfig)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write cfg_blob");
        nvs_close(h);
        return false;
    }

    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "NVS commit failed"); return false; }
    ESP_LOGI(TAG, "Config saved (blob)");
    return true;
}

/* ── Compiled-credential enforcement ──────────────────────────────────────
 * Rationale in config.h. FNV-1a over the exact byte content of the compiled
 * credential set: any edit to any one of them changes the fingerprint, and
 * nothing else does. The NUL terminators are hashed too, so ("ab","c") and
 * ("a","bc") cannot collide. */
static uint32_t fnv1a(uint32_t h, const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static uint32_t fnv1a_str(uint32_t h, const char *s) {
    return fnv1a(h, s, strlen(s) + 1);   /* +1: hash the terminator */
}

uint32_t nvs_config_compiled_cred_fp(void) {
    uint32_t h    = 2166136261u;
    uint16_t port = MQTT_PORT;
    uint8_t  tls  = MQTT_TLS_DEFAULT ? 1u : 0u;

    h = fnv1a_str(h, MQTT_HOST);
    h = fnv1a(h, &port, sizeof(port));
    h = fnv1a_str(h, MQTT_USER);
    h = fnv1a_str(h, MQTT_PASS);
    h = fnv1a_str(h, MQTT_TOPIC_BASE);
    h = fnv1a_str(h, MODEM_APN);
    h = fnv1a_str(h, MODEM_APN_USER);
    h = fnv1a_str(h, MODEM_APN_PASS);
    h = fnv1a(h, &tls, sizeof(tls));

    /* 0 means "never stamped" in NVS, so it must not be a reachable hash —
     * otherwise one particular credential set would silently re-apply itself
     * on every boot. */
    return h ? h : 1u;
}

bool nvs_config_enforce_firmware_creds(DeviceConfig *cfg, bool *destination_changed) {
    if (destination_changed) *destination_changed = false;

    uint32_t fp = nvs_config_compiled_cred_fp();
    if (cfg->cred_fp == fp) return false;    /* same build's credentials — NVS wins */

    /* Did the data actually change destination? Only host/port/base topic
     * decide that: they are what determines whose broker and whose topic tree
     * the readings land in. */
    bool dest = (strcmp(cfg->mqtt_host, MQTT_HOST) != 0 ||
                 cfg->mqtt_port != MQTT_PORT ||
                 strcmp(cfg->mqtt_base_topic, MQTT_TOPIC_BASE) != 0);

    ESP_LOGW(TAG, "credential fingerprint mismatch (stored %08lx, image %08lx) — "
                  "applying the firmware's credentials over NVS",
             (unsigned long)cfg->cred_fp, (unsigned long)fp);
    if (dest)
        ESP_LOGW(TAG, "broker destination changes %s:%u/%s -> %s:%u/%s",
                 cfg->mqtt_host, cfg->mqtt_port, cfg->mqtt_base_topic,
                 MQTT_HOST, MQTT_PORT, MQTT_TOPIC_BASE);

    /* Connectivity ONLY. Everything else in cfg is the site's equipment
     * configuration and is deliberately left exactly as it was. */
    snprintf(cfg->apn,             sizeof(cfg->apn),             "%s", MODEM_APN);
    snprintf(cfg->apn_user,        sizeof(cfg->apn_user),        "%s", MODEM_APN_USER);
    snprintf(cfg->apn_pass,        sizeof(cfg->apn_pass),        "%s", MODEM_APN_PASS);
    snprintf(cfg->mqtt_host,       sizeof(cfg->mqtt_host),       "%s", MQTT_HOST);
    snprintf(cfg->mqtt_user,       sizeof(cfg->mqtt_user),       "%s", MQTT_USER);
    snprintf(cfg->mqtt_pass,       sizeof(cfg->mqtt_pass),       "%s", MQTT_PASS);
    snprintf(cfg->mqtt_base_topic, sizeof(cfg->mqtt_base_topic), "%s", MQTT_TOPIC_BASE);
    cfg->mqtt_port = MQTT_PORT;
    cfg->mqtt_tls  = MQTT_TLS_DEFAULT;

    /* The rollback baseline has to move with them. Leaving the old site's
     * values in _bak would mean a later failed setconfig trial "rolled back"
     * to the PREVIOUS client's broker — the exact outcome this exists to make
     * impossible. Any trial that was in flight is abandoned for the same
     * reason: it was a trial of the old deployment's settings. */
    snprintf(cfg->apn_bak,             sizeof(cfg->apn_bak),             "%s", cfg->apn);
    snprintf(cfg->apn_user_bak,        sizeof(cfg->apn_user_bak),        "%s", cfg->apn_user);
    snprintf(cfg->apn_pass_bak,        sizeof(cfg->apn_pass_bak),        "%s", cfg->apn_pass);
    snprintf(cfg->mqtt_host_bak,       sizeof(cfg->mqtt_host_bak),       "%s", cfg->mqtt_host);
    snprintf(cfg->mqtt_user_bak,       sizeof(cfg->mqtt_user_bak),       "%s", cfg->mqtt_user);
    snprintf(cfg->mqtt_pass_bak,       sizeof(cfg->mqtt_pass_bak),       "%s", cfg->mqtt_pass);
    snprintf(cfg->mqtt_base_topic_bak, sizeof(cfg->mqtt_base_topic_bak), "%s", cfg->mqtt_base_topic);
    cfg->mqtt_port_bak = cfg->mqtt_port;
    cfg->mqtt_tls_bak  = cfg->mqtt_tls;
    cfg->conn_pending  = false;

    cfg->cred_fp = fp;
    if (destination_changed) *destination_changed = dest;
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

/* ── FOTA URL memory ──────────────────────────────────────────────────────
 * See nvs_config.h. Two keys, written together; a torn write (power cut
 * between them) leaves at worst a fingerprint with the wrong state, which the
 * boot-time reconciliation in main.c corrects on the next boot. Kept outside
 * the config blob and never touched by nvs_config_save(). The product guard's
 * nvs_erase_all() DOES clear it — correct, because the URL it remembers was
 * applied by another product's firmware and means nothing to this one. */
#define FOTA_URL_FP_KEY "fota_fp"
#define FOTA_URL_ST_KEY "fota_st"

uint32_t nvs_fota_url_fp(const char *url) {
    uint32_t h = fnv1a_str(2166136261u, url ? url : "");
    return h ? h : 1u;                       /* 0 is the "nothing recorded" sentinel */
}

fota_url_state_t nvs_fota_url_get(uint32_t *fp_out) {
    if (fp_out) *fp_out = 0;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return FOTA_URL_NONE;
    uint32_t fp = 0; uint8_t st = 0;
    bool ok = nvs_get_u32(h, FOTA_URL_FP_KEY, &fp) == ESP_OK &&
              nvs_get_u8 (h, FOTA_URL_ST_KEY, &st) == ESP_OK;
    nvs_close(h);
    if (!ok || fp == 0 || st == 0 || st > FOTA_URL_REVERTED) return FOTA_URL_NONE;
    if (fp_out) *fp_out = fp;
    return (fota_url_state_t)st;
}

bool nvs_fota_url_set(uint32_t fp, fota_url_state_t st) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_u32(h, FOTA_URL_FP_KEY, fp) == ESP_OK &&
              nvs_set_u8 (h, FOTA_URL_ST_KEY, (uint8_t)st) == ESP_OK &&
              nvs_commit(h) == ESP_OK;
    nvs_close(h);
    if (!ok) ESP_LOGE(TAG, "could not record the FOTA URL state");
    return ok;
}

void nvs_fota_url_clear(void) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, FOTA_URL_FP_KEY);
    nvs_erase_key(h, FOTA_URL_ST_KEY);
    nvs_commit(h);
    nvs_close(h);
}
