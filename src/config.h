#pragma once

/* ── Firmware identity ──────────────────────────────────────────────────── */
#define FW_VERSION          "1.0.0"
#define MQTT_TOPIC_BASE     "fuelsense"
/* Start-to-start pacing of one dispenser poll round. Each nozzle is polled and
 * published once per round, so this doubles as the per-nozzle re-read interval:
 * a nozzle's totalizer is sampled roughly every 20 s. Tightening this from the
 * former 30 s — together with dropping the RFID read entirely and moving the
 * k-factor read to a 15-min cadence (carried forward in between) so each round
 * stays short — is what shrinks the window in which two back-to-back sales on
 * the SAME nozzle merge into a single totalizer delta. When a due radar/genset
 * poll falls in a round it runs a couple seconds longer (shared RS485 bus);
 * that is bounded and unavoidable. */
#define PUBLISH_INTERVAL_MS 20000
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
#define DIAG_CYCLE_INTERVAL  45       /* publish the combined diagnostics report every N poll cycles (~15 min at 20 s/cycle) */
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
        #define RADAR_POLL_INTERVAL_S   600  /* tanks:  every 10 min */
#define GENSET_POLL_INTERVAL_S  300   /* DG:     every 5 min  */
/* BlueSky k-factor is a near-constant calibration value; read it only this
 * often and carry the last good reading forward on every other dispenser round
 * (see du_poll_bluesky_nozzle) so the tight totalizer cadence isn't spent
 * re-reading it. */
    #define KFACTOR_POLL_INTERVAL_S 900   /* k-factor: every 15 min */

/* ── RTC discipline ─────────────────────────────────────────────────────────
 * Network time sets the ESP32 RTC at boot and is refreshed on this interval to
 * bound drift (~±20 ppm on the 32.768 kHz crystal ≈ ±1.7 s/day). Everything
 * else reads the RTC, so no payload path ever blocks on a modem AT round-trip
 * just to get a timestamp. 6 h keeps worst-case drift to a few hundred ms while
 * costing 4 AT commands a day — against the ~2,900/day the per-cycle timestamp
 * read used to cost. */
#define TIME_RESYNC_INTERVAL_S  21600  /* 6 hours */

/* ── Maintenance reboot ───────────────────────────────────────────────────
 * There is deliberately NO on-device scheduled reboot. It used to fire on a
 * fixed local RTC hour (20:00 UTC / 01:30 IST), but that meant every gateway
 * in the fleet re-attached to the cellular network and the MQTT broker at the
 * same instant — a synchronised load spike that only gets worse as the fleet
 * grows, for a mechanism this firmware doesn't actually need (there is no
 * malloc anywhere, so heap fragmentation cannot occur, and esp_restart() does
 * not power-cycle the modem, so it cannot clear a wedged SIM/PDP either — see
 * the existing low-heap guard in run_poll_cycle() and the L1-L4 ladder in
 * mqtt_reconnect_if_needed() for the guards that actually matter).
 *
 * Maintenance reboots are now entirely cloud-driven: push {"cmd":"restart"}
 * to a device's own <base>/<imei>/setconfig topic (see parse_setconfig() in
 * main.c). This lets the cloud stagger a fleet-wide reboot across a schedule
 * of its choosing (e.g. 10% of the fleet per time slot) instead of every unit
 * doing it at once. See CLOUD_INTEGRATION_GUIDE.md section 7 for the batching
 * guidance given to the cloud team. */

/* ── GPS/GNSS cadence ──────────────────────────────────────────────────────
 * Controls how often poll_gps() re-reads AT+QGPSLOC and publishes. The GNSS
 * engine runs continuously in the background from boot (gps_init); this only
 * gates the read/publish rate. N -> every N poll cycles (PUBLISH_INTERVAL_MS
 * each), so 1 = every cycle (~20s), 20 = ~7 min. Kept as its own constant
 * (not reusing DIAG_CYCLE_INTERVAL) so the two can be tuned independently.
 *
 * Set to 1 (publish GPS every ~20s) per request 2026-07-21. Note: the asset
 * is stationary, so the coordinate is essentially unchanging — a 20s cadence
 * is far more frequent than the position warrants; raise this well above 1
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
 * The totalizer/RSP/K-factor reads are issued ONLY when the nozzle reports
 * idle, per the BlueSky protocol doc ("If nozzle is delivering or busy, then
 * dont read anything"). (RFID is no longer read at all.) Reading mid-delivery
 * returns a totalizer
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
/* Retry budgets for the per-nozzle DATA reads (attempts, not extra tries — 3
 * means up to 3 total, matching DU_STATUS_RETRIES). The totalizer is the
 * sale-critical value so it is retried like the status frame; k-factor (read
 * only every KFACTOR_POLL_INTERVAL_S) is retried the same. RSP keeps a single
 * read — see du_poll_bluesky_nozzle — so a normal round stays short. A read
 * that fails every attempt degrades to "404" for that field this cycle. */
#define DU_TOTAL_RETRIES    3
#define DU_KFACTOR_RETRIES  3

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
/* Attempts per tank per radar poll before it reports 404 (total, not extra).
 * Applied uniformly to every radar type (Ztech/Pantech/future) at the poll
 * call site, so a new radar driver inherits the retry with no extra work. */
#define RADAR_READ_RETRIES 3

/* ── Genset ─────────────────────────────────────────────────────────────── */
#define GENSET_OFF          0
#define GENSET_SEDEMAC      1
#define GENSET_SLAVE_DEFAULT 10   /* DG default Modbus slave ID */
#define GENSET_READ_RETRIES  3    /* attempts per DG poll before reporting 404 (total, not extra) */

/* Genset model — selects the Modbus read set (see g_models[] in genset.c). Per
 * the DG integration spec each poll issues up to TWO reads, and each is
 * published as its OWN message: a measurement block under "registers" and a DG
 * status block under "status". Both carry the read's DATA bytes as a
 * comma-separated hex string (Modbus header and CRC stripped) — the same shape
 * radar uses. The firmware never interprets a genset register; the cloud owns
 * every register map and all scaling. Reads per model (all CRC-verified against
 * the spec's documented captures):
 *   GC1211        → Mahindra, FC04: registers 1..36 (72 B), status 67..68 (4 B)
 *   GC1111        → Mahindra, FC03: registers 1..36 (72 B), status 67..68 (4 B)
 *                   Identical to GC1211 apart from the function code — the spec
 *                   documents FC03 for this model explicitly.
 *   SGC120        → FC03: one overall read, registers 1..78 (156 B). No status
 *                   command; the single block already covers it.
 *   GC1201/02/03  → Ashok Leyland ("Bottling Plant DG"), FC04: registers 1..36
 *                   (72 B), status 67 (2 B — one register, unlike the Mahindras)
 *   GC902         → NOT covered by the spec; mapped to the Ashok Leyland read
 *                   set, which the vendor register map lists as one shared
 *                   family map. Verify on real GC902 hardware before relying.
 * The separate alarm command the spec lists for GC1211/GC1111 is deliberately
 * NOT issued — alarm state is carried inside the blocks above, and skipping it
 * keeps the genset off the shared RS485 bus for one fewer transaction.
 * Adding a model: add one g_models[] row. Bump GENSET_MODEL_MAX. */
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
