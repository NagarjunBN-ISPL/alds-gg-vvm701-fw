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
#define MODEM_BUF       4096

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

/* ── Connectivity recovery (layered escalation) ──────────────────────────── */
#define MQTT_RECONNECT_RETRIES 5      /* L1: plain MQTT reconnect attempts   */
#define NET_BRINGUP_RETRIES    3      /* L2: network rebuild attempts        */
#define MODEM_PWRCYCLE_MAX     3      /* L3: modem PWRKEY power-cycles       */

/* ── Dispenser units ──────────────────────────────────────────────────────
 * BlueSky and Yenen are separate wire protocols (different framing), so they
 * occupy independent address spaces and can be mixed freely.               */
#define DU_BLUESKY_MAX  6   /* BlueSky units: RS485 addr = unit_idx*2 + 1/2 */
#define DU_YENEN_MAX    6   /* Yenen units:    protocol addr = unit_idx + 1 */

/* ── Radar ─────────────────────────────────────────────────────────────────
 * Ztech and Pantech are both standard Modbus RTU sharing the same RS485
 * bus, so their slave addresses must not collide: Ztech uses 4..4+N-1,
 * Pantech uses 7..7+N-1. RADAR_MAX caps the combined total so Ztech can
 * never reach as high as Pantech's base address (7) when both are in use. */
#define RADAR_MAX      4

/* ── Genset ─────────────────────────────────────────────────────────────── */
#define GENSET_OFF      0
#define GENSET_SEDEMAC  1

/* ── Digital input FIFO ─────────────────────────────────────────────────── */
#define DI_FIFO_SIZE  20
