#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Dispenser make at a station-list position. Stored as a uint8_t in the
 * config blob, so the values are part of the NVS contract: never renumber. */
typedef enum {
    DU_MAKE_BLUESKY = 0,
    DU_MAKE_YENEN   = 1,
    DU_MAKE_MAX     = DU_MAKE_YENEN   /* highest valid value — clamp target */
} du_make_t;

/* ── LAYOUT IS A FLASH CONTRACT ─────────────────────────────────────────────
 * nvs_config_save() writes this struct to NVS as ONE raw blob and
 * nvs_config_load() reads it back through the same layout. A blob written by
 * an older build of this product is accepted when it is SHORTER than the
 * current struct: the stored prefix is copied in and every field past its end
 * keeps its default. That is what makes adding a field FOTA-safe — but only
 * if the new field is APPENDED. Inserting one in the middle shifts every
 * later field by its size, and a provisioned unit then reads its APN, broker
 * and credentials through the wrong offsets on the first boot of the new
 * image. New fields go at the END, after cred_fp. If a field ever has to move
 * or change size, bump the "/N" in PRODUCT_ID (config.h) so the product guard
 * treats the old blob as foreign and wipes it instead of misreading it. */
typedef struct {
    /* ── Dispenser slots: slot i (0-based, setconfig keys du<i+1>Make /
     * du<i+1>Sfx) is ON when its suffix is set, and polls dispenser
     * du_number_of(cfg, i) = that suffix. An empty suffix = slot OFF, never
     * polled. du_count = the number of slots ON: setconfig keeps it in step
     * and nvs_config_load() re-derives it from the suffixes. */
    uint8_t  du_count;                        /* slots ON, 0-6 */
    uint8_t  du_make[DU_MAX_DISPENSERS];      /* du_make_t per slot */
    char     du_sfx[DU_MAX_DISPENSERS][3];    /* "01".."06", or "" = slot OFF */
    uint8_t  ztech_count;       /* 0-4 — slave per tank = its suffix (auto 4..) */
    uint8_t  pantech_count;     /* 0-4 — auto slaves continue after Ztech      */
    uint8_t  reserved_u8;       /* was masibus_count (alds-ggy). Kept as a byte
                                 * so the blob layout ahead of cred_fp is
                                 * unchanged; always 0 here. */
    uint8_t  genset_type;       /* 0=Off,1=SEDEMAC                  */
    uint8_t  genset_model;      /* GENSET_MODEL_* in config.h: 0=GC1211 1=GC1111
                                 * 2=SGC120 3=GC1201 4=GC1202 5=GC1203 6=GC902 */
    uint8_t  genset_slave;      /* Modbus slave, default 10         */
    uint32_t genset_baud;       /* 4800/9600/19200, default 9600    */
    uint8_t  di_enable;         /* bitmask: bit0=DI1..bit3=DI4      */
    char     sfx_genset[3];     /* 2-char suffix + NUL              */
    char     sfx_di[3];
    /* Per-unit HW_ID suffixes (2 digits + NUL). Empty = fall back to the
     * auto value in build_hw_ids(), which for a radar tank is also its Modbus
     * slave. */
    char     sfx_ztech[RADAR_MAX][3];
    char     sfx_pantech[RADAR_MAX][3];
    char     reserved_sfx[RADAR_MAX][3];   /* was sfx_masibus (alds-ggy); layout keeper, always "" */

    /* GNSS via the modem's built-in GPS (see gps.c) — default false. Most
     * gateways stay disabled since not every unit has the GNSS antenna
     * physically connected; toggled remotely via setconfig (gpsEnable). */
    bool     gps_enable;

    /* ── Connectivity (remotely configurable via setconfig) ───────────────
     * Covers both the cellular APN and the MQTT broker. _bak holds whatever
     * connectivity config last PROVED it could get all the way to a live
     * broker session. When setconfig changes any of these primary fields,
     * the pre-change values are staged into _bak and conn_pending is set.
     * The next boot runs a trial (see app_main): reaching a broker session
     * clears conn_pending and the new settings become permanent; failing at
     * EITHER stage — network bring-up (bad APN) or broker connect (bad
     * host/creds) — restores _bak over the primary fields and reboots.
     * A bad push can never leave the device stranded, since there's no WiFi
     * fallback and no physical access at a field-deployed station.
     *
     * APN and broker share ONE pending flag deliberately: on a trial failure
     * we can't reliably attribute the fault to APN vs. broker, so we revert
     * the whole staged delta. Change one connectivity parameter at a time if
     * you want each committed independently. */
    char     apn[48];
    char     apn_user[32];
    char     apn_pass[32];
    char     mqtt_host[64];
    uint16_t mqtt_port;
    char     mqtt_user[32];
    char     mqtt_pass[64];
    char     mqtt_base_topic[32];
    bool     mqtt_tls;               /* true=TLS (ctx2), false=plaintext     */
    char     apn_bak[48];
    char     apn_user_bak[32];
    char     apn_pass_bak[32];
    char     mqtt_host_bak[64];
    uint16_t mqtt_port_bak;
    char     mqtt_user_bak[32];
    char     mqtt_pass_bak[64];
    char     mqtt_base_topic_bak[32];
    bool     mqtt_tls_bak;
    bool     conn_pending;           /* trying new connectivity this boot    */
    bool     conn_rollback_notify;   /* announce a just-happened rollback    */

    /* Fingerprint of the COMPILED credential set that last wrote the fields
     * above (see the note in config.h). 0 = never stamped, which is what every
     * gateway provisioned before this field existed reads back — their stored
     * blob is shorter than this struct, so nvs_config_load() leaves this at its
     * default and the first boot of a firmware carrying this feature adopts the
     * compiled credentials once, then stamps them. */
    uint32_t cred_fp;

    /* ── Appended 2026-10-16 (see the layout note above) ─────────────────
     * How du_count/du_sfx above are to be read. DU_PLAN_LIST (0) is what every
     * blob written before the slot rule reads back — the field lies past its
     * end, so it keeps the default — and means "a list of du_count entries,
     * an empty suffix = the entry's position". nvs_config_load() converts such
     * a plan to slots ONCE, in RAM, polling exactly the dispensers the old
     * build polled, and leaves DU_PLAN_SLOTS here; the next save stores it.
     * Never 0 in a blob this build wrote. */
    uint8_t  du_plan;
} DeviceConfig;

#define DU_PLAN_LIST   0   /* blob from a build before the slot rule */
#define DU_PLAN_SLOTS  1   /* du_sfx are slots (this build)           */

/* ── Dispenser number ──────────────────────────────────────────────────────
 * Slot i (0-based) polls dispenser N = its suffix as a number. N IS the bus
 * address (BlueSky nozzles 2N-1 and 2N, Yenen head N) and the HW_ID suffix
 * ("%02d") — this is the ONE place a suffix becomes a number, so the label the
 * cloud sees and the address the bus is asked cannot disagree. Returns 0 for
 * a slot that is OFF (empty suffix), a suffix that is not a valid number (never
 * after nvs_config_load(), which turns such a slot OFF; possible on a
 * setconfig candidate, where the validator refuses it), or i out of range.
 * Pure — no ESP-IDF, so the host tests compile it. */
int du_number_of(const DeviceConfig *c, int i);

/* True when slot i is ON (its suffix is set). After nvs_config_load() an ON
 * slot always has a valid, unique du_number_of(). Pure. */
bool du_slot_on(const DeviceConfig *c, int i);

void nvs_config_defaults(DeviceConfig *cfg);

/* Call BEFORE nvs_config_load() when the bootloader has just rolled back an
 * image of ANOTHER product (main.c reads the otadata state; this file stays
 * free of esp_ota so the host tests can compile it). That image ran far
 * enough to stamp its product tag over ours but never became the running
 * product, so the tag is a lie about who owns this configuration: the guard
 * restamps ours and keeps everything, instead of wiping a good station plan
 * because a swap attempt failed. NULL/absent = no hint = normal guard. */
void nvs_config_note_reverted_foreign_image(const char *project_name);

bool nvs_config_load(DeviceConfig *cfg);
bool nvs_config_save(const DeviceConfig *cfg);

/* erase_config: every station-equipment field back to its default — the
 * dispenser slots, radar tanks, genset, DI, GPS and every suffix — while the
 * connectivity run (apn .. cred_fp: APN, broker, their rollback copies, the
 * trial flags and the credential stamp) is kept byte for byte, so the gateway
 * restarts on the network and broker it is on now, "unconfigured". Pure, no
 * NVS: the caller saves. */
void nvs_config_clear_equipment(DeviceConfig *cfg);

/* factory_reset: erases this firmware's whole NVS namespace — the config
 * blob, the boot counter, the FOTA URL memory and the product tag — and the
 * sibling product's. The next boot comes up on the compiled-in defaults, as a
 * freshly flashed unit does. The caller restarts. */
void nvs_config_factory_reset(void);

/* True if nvs_config_load() found NVS stamped with a DIFFERENT product and
 * erased it — i.e. this gateway is running a build for other hardware than the
 * configuration it was carrying. The caller comes up on defaults (so it reports
 * "unconfigured"), and must also purge the offline queue: those records describe
 * the previous product's equipment. Valid only after nvs_config_load(). */
bool nvs_config_product_wiped(void);

/* Fingerprint of this build's compiled-in credentials. Never 0 — 0 is the
 * "no stamp in NVS" sentinel. */
uint32_t nvs_config_compiled_cred_fp(void);

/* Force the compiled-in connectivity credentials over whatever NVS holds, if
 * and only if the stored fingerprint says they came from a DIFFERENT build.
 * Call once, straight after nvs_config_load() and before the APN and broker
 * are handed to the modem. Equipment configuration is untouched.
 *
 * Returns true if the credentials were replaced (the caller should announce it
 * and persist), false when NVS already matches this build — the normal path.
 *
 * *destination_changed is set true only when the broker the device publishes to
 * actually moved (host, port or base topic). That is the case where anything
 * still buffered in the offline queue belongs to the PREVIOUS deployment and
 * must be purged rather than delivered to the new one. An APN-only change
 * leaves it false: the destination is the same, so the backlog is still valid.
 * Safe to pass NULL. */
bool nvs_config_enforce_firmware_creds(DeviceConfig *cfg, bool *destination_changed);

/* Persistent power-on counter — incremented once per boot and returned. Lets
 * the fleet spot a station silently reboot-looping (flaky modem, NVS
 * corruption, brownout) that telemetry gaps alone wouldn't reveal. Stored
 * separately from DeviceConfig so a setconfig save never disturbs it. */
uint32_t nvs_boot_count_bump(void);

/* ── FOTA URL memory ─────────────────────────────────────────────────────
 * Which firmware URL this gateway last APPLIED, and whether the image it
 * produced stayed or was rolled back by the bootloader. Kept as a 32-bit
 * FNV-1a of the URL under its own NVS keys (not in the config blob — a
 * setconfig save must never disturb it, and it has to survive a config wipe
 * so a re-delivered retained URL is still recognised after a product swap).
 *
 * This is the guard that stops a RETAINED update message from re-flashing
 * the same image on every boot, fleet-wide: the URL that succeeded is
 * remembered across the restart it triggers, and the image that boots next
 * (same NVS) recognises and skips it. A URL whose image was later REVERTED is
 * remembered too, so a retained URL cannot loop download -> boot -> rollback.
 * A different URL always clears the record. */
typedef enum {
    FOTA_URL_NONE     = 0,   /* nothing recorded                            */
    FOTA_URL_APPLIED  = 1,   /* the image from this URL is what is running  */
    FOTA_URL_REVERTED = 2,   /* that image failed to confirm and was undone */
} fota_url_state_t;

uint32_t nvs_fota_url_fp(const char *url);                 /* never 0 */
fota_url_state_t nvs_fota_url_get(uint32_t *fp_out);       /* fp_out may be NULL */
bool nvs_fota_url_set(uint32_t fp, fota_url_state_t st);
void nvs_fota_url_clear(void);
