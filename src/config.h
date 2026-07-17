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

/* ── Radar cadence ──────────────────────────────────────────────────────────
 * Tank levels change slowly, so radar is polled once every N poll cycles
 * instead of every cycle. With PUBLISH_INTERVAL_MS=30s, 2 → every 60s. */
#define RADAR_CYCLE_INTERVAL  2

/* ── GPS/GNSS cadence ──────────────────────────────────────────────────────
 * VVM701 is a stationary fuel-station asset — its position never changes
 * after installation, so there's no value in re-querying the fix on the same
 * cadence as mobile telemetry. The GNSS engine runs continuously in the
 * background from boot (gps_init); this only controls how often poll_gps()
 * re-reads AT+QGPSLOC. 20 -> every 20 poll cycles, 20*30s = 10 min. Kept as
 * its own constant (not reusing DIAG_CYCLE_INTERVAL) so the two can be tuned
 * independently. (Field-verified working end-to-end on 2026-07-17 with a
 * temporary 1-cycle debug override, since reverted back to 20.) */
#define GPS_CYCLE_INTERVAL  20

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

/* ── Radar ─────────────────────────────────────────────────────────────────
 * Ztech and Pantech are standard Modbus RTU on the shared RS485 bus with
 * fixed, non-overlapping slave ranges: Ztech = 4..4+N-1 (4..7), Pantech =
 * 8..8+N-1 (8..11). RADAR_MAX caps each type independently at 4 tanks so the
 * two ranges can never meet — if you raise RADAR_MAX again, PANTECH_SLAVE_BASE
 * MUST move to at least ZTECH_SLAVE_BASE + RADAR_MAX, or the two ranges
 * overlap and the bus crosstalk guard in genset.c/radar.c's slave-echo check
 * can no longer tell a Ztech reply from a Pantech one. (BlueSky nozzle
 * addresses 4-6 numerically overlap these but use different framing/protocol
 * and are time-multiplexed on the bus, so there is no real collision there.)
 * Same "actual count, not the cap, drives every runtime loop" property as
 * DU_BLUESKY_MAX above — the genset/radar slave-collision check (main.c,
 * nvs_config.c) is already computed from these macros dynamically, so it
 * stays correct automatically when these change. */
#define RADAR_MAX          4
#define ZTECH_SLAVE_BASE   4
#define PANTECH_SLAVE_BASE 8

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
