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

/* ── Connectivity recovery (layered escalation) ──────────────────────────── */
#define MQTT_RECONNECT_RETRIES 5      /* L1: plain MQTT reconnect attempts   */
#define NET_BRINGUP_RETRIES    3      /* L2: network rebuild attempts        */
#define MODEM_PWRCYCLE_MAX     3      /* L3: modem PWRKEY power-cycles       */
#define MQTT_PUB_FAIL_MAX      10     /* consecutive publish fails → link down */

/* ── Dispenser units ──────────────────────────────────────────────────────
 * Only BlueSky is supported. Up to 3 dispensers, each occupying 2 RS485
 * nozzle addresses (unit_idx*2 + 1/2): DU1=1,2  DU2=3,4  DU3=5,6.          */
#define DU_BLUESKY_MAX  3

/* ── Radar ─────────────────────────────────────────────────────────────────
 * Ztech and Pantech are standard Modbus RTU on the shared RS485 bus with
 * fixed, non-overlapping slave ranges: Ztech = 4..4+N-1 (4,5), Pantech =
 * 6..6+N-1 (6,7). RADAR_MAX caps each type independently at 2 tanks so the
 * two ranges can never meet. (BlueSky nozzle addresses 4-6 numerically
 * overlap these but use different framing/protocol and are time-multiplexed
 * on the bus, so there is no real collision.) */
#define RADAR_MAX          2
#define ZTECH_SLAVE_BASE   4
#define PANTECH_SLAVE_BASE 6

/* ── Genset ─────────────────────────────────────────────────────────────── */
#define GENSET_OFF          0
#define GENSET_SEDEMAC      1
#define GENSET_SLAVE_DEFAULT 10   /* DG default Modbus slave ID */

/* Genset model — selects which Modbus function code reads the measurement,
 * alarm and status registers. The register MAP is identical across both;
 * the only difference (verified against live wire captures) is the FC:
 *   GC1211/1201/02/03/902 → FC04 (Read Input Registers)
 *   GC1111/1112           → FC03 (Read Holding Registers)
 * The genset command/operating-mode holding readback (batch3) exists only on
 * the GC1211 family, where the FC03 space is separate from measurements. */
#define GENSET_MODEL_GC1211  0
#define GENSET_MODEL_GC1111  1

/* ── Digital input FIFO ─────────────────────────────────────────────────── */
#define DI_FIFO_SIZE  20

/* ── Thermal zones (°C) — tuned for 24/7 enclosed operation, no heatsink ─── */
#define THERMAL_NORMAL_MAX_C    65.0f
#define THERMAL_WARM_MAX_C      75.0f
#define THERMAL_CRITICAL_MIN_C  80.0f
