#pragma once

/* ── Firmware identity ──────────────────────────────────────────────────── */
#define FW_VERSION          "1.0.0"
#define MQTT_TOPIC_BASE     "fuelsense"
#define PUBLISH_INTERVAL_MS 30000
#define WDT_TIMEOUT_MS      120000

/* ── MQTT broker ────────────────────────────────────────────────────────── */
#define MQTT_HOST   "dev-fuelsense.iotronsys.com"
#define MQTT_PORT   8777
#define MQTT_USER   "iotrondev"
#define MQTT_PASS   "iotrondev@1234"
#define MQTT_QOS    1
/* Quectel's AT+QMTPUBEX has a hard ~1500-byte payload ceiling on the EC200U
 * family — data beyond it is rejected/omitted at the modem level, not by
 * anything in this firmware. Genset is the only payload type whose worst
 * case (many simultaneous active alarms) can realistically approach this.
 * Comfortably below 1500 to leave margin for exact modem-firmware behavior. */
#define MQTT_MODEM_PAYLOAD_SAFE_MAX 1400

/* ── Quectel EC200U (UART2) ─────────────────────────────────────────────── */
#define MODEM_UART      UART_NUM_2
#define MODEM_TX_PIN    GPIO_NUM_41
#define MODEM_RX_PIN    GPIO_NUM_40
#define MODEM_PWRKEY    GPIO_NUM_42
#define MODEM_BAUD      115200
#define MODEM_APN       "AIRTELMETERV4"
/* APN auth — empty = no PAP/CHAP (the usual case for M2M SIMs). These are
 * only the factory/recovery defaults; the live values live in NVS and are
 * remotely reconfigurable via setconfig (apn/apnUser/apnPass). */
#define MODEM_APN_USER  ""
#define MODEM_APN_PASS  ""
/* 50% larger than FOTA_CHUNK (4096, fota.c) — the UART ring buffer needs
 * margin over the largest single burst read so a scheduling delay of a
 * few hundred ms during a FOTA chunk can't overflow it and corrupt the
 * image. */
#define MODEM_BUF       6144

/* ── RS485 bus (UART1) ──────────────────────────────────────────────────── */
#define RS485_UART      UART_NUM_1
#define RS485_TX_PIN    GPIO_NUM_1
#define RS485_RX_PIN    GPIO_NUM_2
#define RS485_BUF       256
#define RS485_RECV_TIMEOUT_MS 2000   /* per-byte-reset read window */

/* ── Nuvoton MS51 (UART0 — console must move to USB CDC) ────────────────── */
#define NUVOTON_UART        UART_NUM_0
#define NUVOTON_TX_PIN      GPIO_NUM_43
#define NUVOTON_RX_PIN      GPIO_NUM_44
#define NUVOTON_BAUD        9600
#define NUVOTON_BUF         256
#define NUVOTON_TIMEOUT_MS  3000

/* ── GPIO ───────────────────────────────────────────────────────────────── */
#define LED_PIN         GPIO_NUM_18
/* GPIO 35/36/37 are used by 8MB PSRAM — must not be touched */

/* ── Diagnostics ────────────────────────────────────────────────────────── */
#define DIAG_CYCLE_INTERVAL  10       /* publish diag every N poll cycles    */
#define DIAG_BOOT_GRACE_MS   60000    /* ignore loop_max during boot warmup  */

/* ── Secondary-equipment cadence (wall-clock, not cycle counts) ─────────────
 * Radar and genset are deliberately slow-cadence: tank level and genset state
 * change over minutes, while a dispenser nozzle can start and finish a sale in
 * seconds. Every second spent on the shared RS485 bus talking to a tank or a
 * DG is a second the dispensers are NOT being read, and a sale that begins and
 * ends inside that gap collapses into the next totalizer delta instead of
 * being recorded as its own transaction.
 *
 * These are elapsed-time budgets, NOT "every N poll cycles" as before. Cycle
 * length is not constant — it varies with nozzle count, retries and publish
 * latency — so a cycle count maps to no predictable wall-clock period. The old
 * RADAR_CYCLE_INTERVAL=2 was nominally "every 60 s" but measured ~168 s in the
 * field once dispenser polling stretched the cycle. Anchoring to elapsed
 * microseconds makes "every 3 minutes" mean exactly that. */
#define RADAR_POLL_INTERVAL_S   180   /* tanks:  every 3 min */
#define GENSET_POLL_INTERVAL_S  300   /* DG:     every 5 min */

/* ── RTC discipline ─────────────────────────────────────────────────────────
 * Network time sets the ESP32 RTC at boot and is refreshed on this interval to
 * bound drift (~±20 ppm on the 32.768 kHz crystal ≈ ±1.7 s/day). Everything
 * else reads the RTC, so no payload path ever blocks on a modem AT round-trip
 * just to get a timestamp. 6 h keeps worst-case drift to a few hundred ms while
 * costing 4 AT commands a day — against the ~2,900/day the per-cycle timestamp
 * read used to cost. */
#define TIME_RESYNC_INTERVAL_S  21600  /* 6 hours */

/* ── GPS/GNSS cadence ──────────────────────────────────────────────────────
 * Controls how often poll_gps() re-reads AT+QGPSLOC and publishes. The GNSS
 * engine runs continuously in the background from boot (gps_init); this only
 * gates the read/publish rate. N -> every N poll cycles (PUBLISH_INTERVAL_MS
 * each), so 1 = every cycle (~30s), 20 = ~10 min. Kept as its own constant
 * (not reusing DIAG_CYCLE_INTERVAL) so the two can be tuned independently.
 *
 * Set to 1 (publish GPS every ~30s) per request 2026-07-21. Note: the asset
 * is stationary, so the coordinate is essentially unchanging — a 30s cadence
 * is far more frequent than the position warrants; raise this back toward 20
 * if the extra fleet-wide GPS traffic becomes a concern. */
#define GPS_CYCLE_INTERVAL  1

/* ── Connectivity recovery (layered escalation) ──────────────────────────── */
#define MQTT_RECONNECT_RETRIES 5      /* L1: plain MQTT reconnect attempts   */
#define NET_BRINGUP_RETRIES    3      /* L2: network rebuild attempts        */
#define MODEM_PWRCYCLE_MAX     3      /* L3: modem PWRKEY power-cycles       */
#define MQTT_PUB_FAIL_MAX      10     /* consecutive publish fails → link down */

/* ── Dispenser units ──────────────────────────────────────────────────────
 * Only BlueSky is supported. Up to 6 dispensers, each occupying 2 RS485
 * nozzle addresses (unit_idx*2 + 1/2): DU1=1,2 .. DU6=11,12. Raising this
 * further is safe and mechanical — every runtime loop (subscribe_all,
 * poll_dispensers, execute_pending) iterates the actual configured count
 * (g_cfg.du_bluesky_count), never this cap, so idle headroom above what's
 * configured costs nothing at runtime. Bump the cap, rebuild; nothing else
 * to touch except s_setcfg_known_keys[] in main.c (sfxBsN entries) and the
 * cloud integration doc's setconfig key table. */
#define DU_BLUESKY_MAX  6

/* ── Dispenser nozzle status gate ─────────────────────────────────────────
 * The totalizer/RSP/K-factor/RFID reads are issued ONLY when the nozzle
 * reports idle, per the BlueSky protocol doc ("If nozzle is delivering or
 * busy, then dont read anything"). Reading mid-delivery returns a totalizer
 * captured part-way through a sale, which the cloud — which derives every
 * transaction from the delta between consecutive readings — turns into two
 * transactions for one fill. Skipping instead means the next reading is taken
 * with the nozzle at rest, so each delta is a whole number of COMPLETED sales.
 *
 * Idle is (status byte 3 == 0x80 || == 0x00). 0x80 is the documented value;
 * 0x00 is a known controller-firmware defect seen in the field where an idle
 * nozzle reports zero instead. Treating only 0x80 as idle made those units
 * look permanently busy, so they were never read at all.
 *
 * DU_STATUS_RETRY_MS is 300, not the 2000 the old Arduino firmware used. With
 * 0x00 now accepted, retries no longer exist to wait out a mis-reported idle —
 * they exist to recover a status frame lost to line noise, which needs
 * milliseconds, not seconds. 3 x 2 s of bus-blocking per busy nozzle would
 * directly widen the very polling gap that causes back-to-back sales to be
 * merged into one transaction. Set to 2000 to reproduce the old timing. */
#define DU_STATUS_RETRIES   3
#define DU_STATUS_RETRY_MS  300

/* ── Radar ─────────────────────────────────────────────────────────────────
 * Ztech and Pantech are standard Modbus RTU on the shared RS485 bus. Slave
 * addressing matches the convention actually wired at outlets: Ztech =
 * 4..4+N-1 (normally 4,5), Pantech = 6..6+N-1 (normally 6,7) — confirmed
 * 2026-07-20 against real field installs. (An earlier PANTECH_SLAVE_BASE=8
 * was never correct against any real outlet; it was a guess that happened to
 * leave 8-11 free, not a documented convention.)
 *
 * Unlike the old 8-11 spacing, these two ranges are NOT structurally
 * non-overlapping at RADAR_MAX: ztechCount >= 3 pushes Ztech's range onto
 * slave 6, which is Pantech tank 1's fixed address. This only matters if a
 * station runs more than the normal 2 Ztech tanks AND has Pantech enabled at
 * the same time — the field convention (2 + 2) never hits it. Because the
 * macros no longer guarantee non-overlap by themselves, parse_setconfig()
 * and nvs_config_validate() both actively reject/clamp ztech_count/
 * pantech_count combinations that would collide, the same way they already
 * guard genset_slave against landing inside either radar range. If you ever
 * change either base or RADAR_MAX, re-check those two guards. */
#define RADAR_MAX          4
#define ZTECH_SLAVE_BASE   4
#define PANTECH_SLAVE_BASE 6

/* ── Genset ─────────────────────────────────────────────────────────────── */
#define GENSET_OFF          0
#define GENSET_SEDEMAC      1
#define GENSET_SLAVE_DEFAULT 10   /* DG default Modbus slave ID */

/* Genset model — selects the driver in genset.c (see the g_drivers table).
 * Each model has its own register map, function code(s) and scaling:
 *   GC1211                 → Sedemac, FC04 (Read Input Registers) + FC03 holding
 *   GC1111                 → Sedemac, FC03 (Read Holding Registers), no holding
 *   SGC120                 → DEIF SGC 120 Mk II, FC03, separate DEIF register map
 *   GC1201/1202/1203/GC902 → NOT interchangeable with GC1211/GC1111 despite being
 *                            the same Sedemac GenControl family. Their alarm/status
 *                            block is documented one register higher than the
 *                            wire-verified GC1211/GC1111 offset (see genset.c) —
 *                            using GC1211's driver against this hardware would
 *                            silently misread every alarm by one register. These
 *                            IDs exist so the field can select "this station has
 *                            one of these, not a plain GC1211/GC1111" and get a
 *                            clean 404 instead of wrong data, until each gets its
 *                            own verified driver (protocol doc pending as of
 *                            2026-07-14).
 * Adding a model: define its id here, add a driver row in genset.c, and bump
 * GENSET_MODEL_MAX to the new highest id. */
#define GENSET_MODEL_GC1211  0
#define GENSET_MODEL_GC1111  1
#define GENSET_MODEL_SGC120  2
#define GENSET_MODEL_GC1201  3
#define GENSET_MODEL_GC1202  4
#define GENSET_MODEL_GC1203  5
#define GENSET_MODEL_GC902   6
#define GENSET_MODEL_MAX     6   /* highest valid model id — update when adding */

/* ── Digital input FIFO ─────────────────────────────────────────────────── */
#define DI_FIFO_SIZE  20

/* ── Thermal zones (°C) — tuned for 24/7 enclosed operation, no heatsink ─── */
#define THERMAL_NORMAL_MAX_C    65.0f
#define THERMAL_WARM_MAX_C      75.0f
#define THERMAL_CRITICAL_MIN_C  80.0f
