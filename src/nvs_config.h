#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef struct {
    uint8_t  du_bluesky_count;  /* 0-3                              */
    uint8_t  ztech_count;       /* 0-2 (slaves 4,5)                 */
    uint8_t  pantech_count;     /* 0-2 (slaves 6,7)                 */
    uint8_t  genset_type;       /* 0=Off,1=SEDEMAC                  */
    uint8_t  genset_model;      /* 0=GC1211,1=GC1111,2=SGC120       */
    uint8_t  genset_slave;      /* Modbus slave, default 10         */
    uint32_t genset_baud;       /* 4800/9600/19200, default 9600    */
    uint8_t  di_enable;         /* bitmask: bit0=DI1..bit3=DI4      */
    char     sfx_genset[3];     /* 2-char suffix + NUL              */
    char     sfx_di[3];
    /* Per-unit HW_ID suffixes (2-char + NUL). Empty = fall back to the
     * auto-generated B<n>/Z<n>/P<n> pattern in build_hw_ids(). */
    char     sfx_bs[DU_BLUESKY_MAX][3];
    char     sfx_ztech[RADAR_MAX][3];
    char     sfx_pantech[RADAR_MAX][3];

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
} DeviceConfig;

void nvs_config_defaults(DeviceConfig *cfg);
bool nvs_config_load(DeviceConfig *cfg);
bool nvs_config_save(const DeviceConfig *cfg);
void nvs_config_factory_reset(void);

/* Persistent power-on counter — incremented once per boot and returned. Lets
 * the fleet spot a station silently reboot-looping (flaky modem, NVS
 * corruption, brownout) that telemetry gaps alone wouldn't reveal. Stored
 * separately from DeviceConfig so a setconfig save never disturbs it. */
uint32_t nvs_boot_count_bump(void);
