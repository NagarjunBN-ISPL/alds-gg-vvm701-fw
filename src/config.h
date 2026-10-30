#pragma once

/* ── Firmware identity ──────────────────────────────────────────────────── */
#define FW_VERSION          "1.0.0"

/* ── Product / config-schema identity ──────────────────────────────────────
 * Identifies WHOSE equipment configuration the NVS blob holds. The credential
 * guard below deals with a gateway moving between customers of the same
 * product; this one deals with it moving between PRODUCTS — an LPG gateway
 * pulled from a filling station and reflashed as a CNG unit, say.
 *
 * Credentials alone are not enough there. NVS would still hold the LPG site's
 * equipment configuration: BlueSky and Yenen dispenser counts, HW_ID suffixes,
 * radar and genset settings that describe hardware which is not at the new
 * site. Nothing crashes — every field is range-clamped on load — which is
 * exactly the problem: the gateway comes up reporting "ready", polls RS485
 * addresses where nothing is installed, and publishes a stream of 404s under
 * HW_IDs belonging to another station's equipment. It looks healthy.
 *
 * It is also not safe to simply reinterpret the stored blob: it is a raw dump
 * of DeviceConfig, so a different product's build reads another struct layout's
 * bytes through its own field offsets. The clamps stop that from being
 * dangerous, not from being wrong.
 *
 * So this string is stamped in NVS under its own key — OUTSIDE the blob, so it
 * can be read without trusting the layout. A stored value that disagrees with
 * the running image means the whole configuration belongs to a different
 * product: NVS is erased and the gateway comes up UNCONFIGURED, waiting for
 * setconfig, which is the only honest state for a unit whose installed
 * equipment nobody has told it about yet.
 *
 * Bump the suffix if DeviceConfig's field order ever changes incompatibly.
 * APPENDING a field needs no bump — nvs_config_load() already handles a short
 * blob by leaving new fields at their defaults. */
#define PRODUCT_ID          "alds-gg-vvm701/1"
/* MQTT base topic for the GG fleet: <base>/data (polled readings), <base>/logs
 * (everything else), <base>/<imei>/setconfig, <base>/<imei>/update,
 * <base>/fleet/gg/update, ... Part of the credential set
 * below, so changing it re-points every unit that runs this build.
 *
 * GG and GGY will share ONE production broker and base topic (~300 GG stations,
 * ~20 GGY). Product separation on that shared tree rests on three things and
 * nothing else: the fleet FOTA topic carries the product name (fleet/gg vs
 * fleet/ggy), the FOTA JSON "target" belt (gg vs ggy), and the distinct
 * PRODUCT_ID above (a GG<->GGY swap by FOTA wipes equipment config and comes up
 * unconfigured, which is correct — the two store different config layouts). */
#define MQTT_TOPIC_BASE     "fuelsense"
/* Start-to-start pacing of one dispenser poll round on du_task. Yenen nozzles
 * are read every round (10 s); BlueSky nozzles every SECOND round (20 s), see
 * du_task.c — a BlueSky nozzle costs up to 7 RS485 reads and the site mix
 * this build targets is Yenen-heavy, so the slower protocol is de-rated rather
 * than the whole cadence. Each nozzle is published once per read, so this is
 * also the per-nozzle totalizer sampling interval, which bounds the window in
 * which two back-to-back sales on ONE nozzle merge into a single delta.
 *
 * 10 s is a throughput floor as well as a latency one: everything a round
 * emits must clear the link before the next round produces more. At ~0.6 s
 * per publish that sustains ~16 nozzle payloads per 10 s; the absolute maximum
 * config (6 Yenen = 12/round) has margin, and a BlueSky-heavy site emits half
 * as often. Do not shorten this further without message batching. */
#define PUBLISH_INTERVAL_MS 10000
/* BlueSky cadence, wall-clock. A BlueSky nozzle is read every 20 s — the value
 * the field fleet has always run at — and this is enforced by TIME, not by
 * "every second round": du_task polls BlueSky in a round only when at least
 * this long has passed since the last round that did, so an overrunning round
 * can never quietly stretch the cadence. On a site with no Yenen the round
 * itself is paced at this interval (there is nothing to do in between); with
 * Yenen present the round stays at PUBLISH_INTERVAL_MS and BlueSky joins every
 * second one. Must be a multiple of PUBLISH_INTERVAL_MS. */
#define BLUESKY_POLL_INTERVAL_MS 20000
#define WDT_TIMEOUT_MS      120000

/* ==========================================================================
 * MQTT Configuration
 * ========================================================================== */

/* PRODUCTION broker — the same broker, user and base topic as alds-ggy. Every
 * unit running this build is FORCED onto these values (see the credential
 * guard below), so an image built with any other set (a dev image) re-points
 * every station it reaches. */
#define MQTT_HOST   "dev-fuelsense.iotronsys.com"
#define MQTT_PORT   8777
#define MQTT_USER   "iotrondev"
#define MQTT_PASS   "iotrondev@1234"
#define MQTT_QOS    1
/* Use TLS by default. Like the server address, this is baked in —
 * changing it is a credential change like any other. */
#define MQTT_TLS_DEFAULT true

/* ── The compiled credential set owns the device ───────────────────────────
 * MQTT_HOST/PORT/USER/PASS/TOPIC_BASE/TLS + MODEM_APN/USER/PASS above are not
 * merely first-boot fallbacks. They are the deployment's identity, and the
 * firmware ENFORCES them over whatever NVS holds — see
 * nvs_config_enforce_firmware_creds().
 *
 * The problem that forces this: NVS survives both FOTA and a USB reflash. A
 * gateway that spent a year at client A, then gets pulled, reflashed with
 * client B's build and installed at client B, would come up on A's broker with
 * A's credentials and publish B's sales into A's account. Nothing in a firmware
 * update fixes that on its own, because updating firmware deliberately does not
 * touch configuration.
 *
 * So each build stamps a fingerprint of the values above into NVS. On boot, a
 * stored fingerprint that does not match the running image means this device is
 * carrying some other build's connectivity config: the compiled values are
 * written over it and the offline queue is purged if the destination changed,
 * so not one of A's buffered records can reach B.
 *
 * What this deliberately does NOT do:
 *   - It does not touch equipment config. Dispenser/radar/genset counts, HW_ID
 *     suffixes, DI mask and GPS survive untouched; only connectivity is forced.
 *   - It does not fight setconfig. A credential pushed at runtime persists
 *     normally, because the fingerprint tracks the IMAGE's values, not NVS's —
 *     reflashing the same build never reverts an operator's change. The next
 *     build with different compiled credentials does win, which is the point.
 *   - It does not fire on a re-flash of the same image, so it costs no flash
 *     wear on the normal path. */
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
/* UART ring buffer for the modem. 8x the FOTA_CHUNK (2048, fota.c): the ring
 * is what absorbs a scheduling delay of a few hundred ms during a chunk read,
 * and a +QMTRECV payload (up to 2 KB) landing on top of URC traffic. RAM is
 * abundant; a too-small ring corrupts a FOTA image silently. */
#define MODEM_BUF       16384

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

/* Wait for the shared MS51 transaction lock (ms51_bus.h). Must exceed
 * NUVOTON_TIMEOUT_MS: a DI poll can hold the port for the full 3 s when the
 * MS51 is silent, and a relay command that gave up before then would be
 * dropped merely because DI happened to be mid-poll. 4 s still sits far
 * inside RELAY_UNLOCK_WINDOW_MS and the deadman ceiling below. */
#define MS51_BUS_LOCK_TIMEOUT_MS  4000

/* ── Panel-box solenoid lock (RL1 via the MS51) ────────────────────────────
 * Ported unchanged from the tested OTA_Integration implementation. Fail-locked
 * / energize-to-unlock: relay_unlock() retracts the bolt for the window below,
 * then de-energizes; the spring latch re-locks mechanically when the door is
 * next pushed shut. The deadman is an independent hard ceiling that forces a
 * restart (which re-locks at boot) if the window never closes. It MUST stay
 * greater than the window. */
#define RELAY_UNLOCK_WINDOW_MS   8000
#define RELAY_UNLOCK_DEADMAN_MS  11000
/* Headroom kept between the end of an unlock window and the deadman ceiling.
 * Repeated unlock commands refresh the window, and without this the refreshed
 * window can be pushed PAST the deadman — so a support engineer pressing the
 * dashboard button three times because nothing seemed to happen would trip
 * the deadman, restart the gateway and lose every unpublished record still in
 * RAM. Ordinary human behaviour must not reboot a device. So relay.c clamps a
 * refreshed window to end this long before the deadman; the deadman then fires
 * only for a genuine fault (a wedged task, a lost timer), which is what it was
 * for. Must cover the relay task waking and completing the MS51 exchange
 * (~420 ms). Ported from cng-cpil (LOCK_RELOCK_MARGIN_MS). */
#define RELAY_RELOCK_MARGIN_MS   1000
/* How long relay_task waits on its command queue before looping to feed the
 * task watchdog. Purely a WDT heartbeat — a queued command still wakes the
 * task immediately, so this does not add command latency. Must stay well under
 * WDT_TIMEOUT_MS. */
#define RELAY_TASK_TICK_MS       2000

/* ── GPIO ───────────────────────────────────────────────────────────────── */
#define LED_PIN         GPIO_NUM_18
/* GPIO 35/36/37 are used by 8MB PSRAM — must not be touched */

/* ── Diagnostics ────────────────────────────────────────────────────────── */
#define DIAG_CYCLE_INTERVAL  90       /* publish the combined diagnostics report every N poll cycles (~15 min at 10 s/cycle) */
/* Per-core CPU load in that report: the average over the report's window plus
 * the busiest one-minute sample in it — one sample every N cycles (~1 min at
 * 10 s/cycle). See cpu_load.h. */
#define CPU_LOAD_SAMPLE_CYCLES  6
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
#define RADAR_POLL_INTERVAL_S   600   /* Modbus tanks (Ztech/Pantech): every 10 min */
#define GENSET_POLL_INTERVAL_S  300   /* DG:     every 5 min  */
/* ── Fast re-poll for the slow-cadence equipment ──────────────────────────
 * Radar, genset and GPS are sampled every 5-10 minutes, so ONE failed poll —
 * a tank that was mid-reboot, a burst of bus noise, a bus lock lost to a long
 * dispenser round, a publish that hit a link blip — costs a whole interval of
 * missing data, and these samples are not store-and-forwarded. The
 * back-to-back retries inside a poll (RADAR_READ_RETRIES / GENSET_READ_RETRIES,
 * MODBUS_RETRY_GAP_MS apart) only cover noise and a slow reply. So a poll that did not yield a
 * published reading is re-tried EQUIP_RETRY_INTERVAL_S later, up to
 * EQUIP_RETRY_MAX times, and only the units that failed are re-polled (a
 * healthy tank is not re-sampled). The regular cadence is anchored to the last
 * FULL poll and never drifts; the retries fill the gap. A permanently dead
 * unit therefore costs at most EQUIP_RETRY_MAX extra polls per interval —
 * bounded bus time, not an endless hammer. */
#define EQUIP_RETRY_INTERVAL_S  60
#define EQUIP_RETRY_MAX         3
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
/* NTP fallback (modem.c ntp_sync). Network time (NITZ, AT+QLTS) stays first,
 * but a cell that never sends it left the clock unset for good — and an unset
 * clock holds back every dispenser reading (du_ts_now). So when QLTS gives
 * nothing, an NTP server is asked through the modem's data context (AT+QNTP),
 * before falling back to the modem's own free-running clock (CCLK). A try
 * holds the main loop for up to NTP_TIMEOUT_MS, so tries are at least
 * NTP_RETRY_INTERVAL_S apart and alternate servers. Sites whose cell sends
 * network time never reach this. */
#define NTP_SERVER_1          "time.google.com"
#define NTP_SERVER_2          "pool.ntp.org"
#define NTP_TIMEOUT_MS        15000
#define NTP_RETRY_INTERVAL_S  900    /* 15 min */

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
 * Wall-clock interval between poll_gps() reads/publishes — like
 * RADAR_POLL_INTERVAL_S / GENSET_POLL_INTERVAL_S, and NOT a poll-cycle count:
 * cycle length varies with nozzle count, retries and publish latency, so a
 * cycle count maps to no predictable period. The GNSS engine runs continuously
 * in the background from boot (gps_init); this only gates the read/publish
 * rate. Gated further by gps_enable (default false), so it stays inert on every
 * gateway that hasn't been explicitly enabled from the cloud.
 *
 * 300 s (every 5 min): the asset is stationary, so its coordinate is
 * essentially unchanging — a fix every 5 minutes is ample and keeps fleet-wide
 * GPS traffic low. */
#define GPS_POLL_INTERVAL_S  300

/* The GNSS driver (gps.c/gps.h) is the one bwsr-neo-vvm701-fw uses, byte for
 * byte, so every product runs the same GPS logic; only the cadence above and
 * the default of gpsEnable are the product's own. The settings below are that
 * driver's, with bwsr's values.
 *
 * ── AGPS (assisted GNSS) ──────────────────────────────────────────────────
 * The slow part of a cold start is not finding satellites but copying each
 * one's orbit data (ephemeris) out of its 50 bit/s broadcast: 30 s of clean
 * signal per satellite, restarted by any dropout — ~35 s in the open, ~10 min
 * measured under the office roof on 2026-09-17. apflash only spares a restart
 * within ~2 h of the last fix; a gateway that was powered down for longer (a
 * station power cut, a bowser parked overnight) cold-starts. With AGPS the
 * modem fetches the current ephemeris from Quectel's assistance server over
 * the cellular link when the engine starts, and again once the copy is ~2 h
 * old (it checks by itself), so the sky search has nothing left to wait for:
 * typically a fix in 5-15 s even with partial sky. If the download fails (no
 * network yet, server unreachable, second context refused) the engine
 * cold-starts exactly as before — nothing gets worse.
 *
 * The AGPS engine activates a PDP context of its own to fetch the data, and
 * the app note warns it may DEACTIVATE that context afterwards — so it must
 * never share context 1 with MQTT. It gets GPS_AGPS_PDP_CTX with the same APN
 * and credentials as the data context (gps.c writes both). Server, vendor and
 * model IDs and the password are the modem's own documented defaults (EC200U
 * Series GNSS Application Note V1.1, 2.3.8 and 3.4) — Quectel's account with
 * the assistance provider, not a secret. GPS_AGPS_ENABLE 0 switches AGPS off
 * on the modem as well (the switch is saved in its NVRAM). None of this runs
 * unless gpsEnable is on; with it off, gps_init() switches the modem's GNSS,
 * autogps and AGPS off instead, so a disabled unit fetches nothing. */
#define GPS_AGPS_ENABLE      1
#define GPS_AGPS_PDP_CTX     2
#define GPS_AGPS_URL         "http://quectel-api1.rx-networks.cn/rxn-api/locationApi/rtcm"
#define GPS_AGPS_VENDOR_ID   "wLgWwv6JQt"
#define GPS_AGPS_MODEL_ID    "Quectel"
#define GPS_AGPS_PASSWORD    "aFltUERDZzZxeTY5cEp2eA=="

/* One-shot antenna check: at the first fix, or GPS_ANT_CHECK_NOFIX_S after
 * boot without one, read the satellites in view (GSV) and log how many are
 * tracked at a usable signal level and the best C/N0. A roof antenna under a
 * clear sky shows 35-45 dB-Hz on several satellites; under 30 on all of them
 * means the antenna, its cable or its placement — not the engine. Console
 * only: it is an installation check, read off the boot log. */
#define GPS_ANT_CHECK_NOFIX_S    180
#define GPS_ANT_CHECK_STRONG_DB  30

/* BENCH ONLY. 1 = wipe the engine's assistance data at every boot
 * (AT+QGPSDEL=0, then restart the engine) so the next fix is a true cold
 * start — the only way to measure cold TTFF with and without AGPS without
 * waiting hours for apflash and the AGPS file to expire. Logged at ERROR on
 * every boot while on. Must be 0 in any build that leaves the bench; the
 * release workflow refuses a tag while it is 1. */
#define GPS_TEST_COLD_START  0

/* ── Un-provisioned heartbeat ──────────────────────────────────────────────
 * A gateway with NO equipment configured (fresh from flash, or just
 * factory-reset) re-announces status "unconfigured" on this interval so the
 * cloud keeps flagging an un-provisioned unit until a setconfig arrives —
 * rather than a single boot-time announce that is easy to miss. Stops the
 * instant any equipment is configured. */
#define UNCONFIGURED_HEARTBEAT_S  180   /* every 3 min while unconfigured */

/* ── Inbound command retain-grace ─────────────────────────────────────────
 * A broker re-delivers a RETAINED message on every (re)subscribe. For a
 * one-shot command that is a loop: a retained restart reboots the gateway
 * forever, a retained erase_config or factory_reset wipes it on every boot, a retained unlock
 * opens the panel box unattended on every reconnect, a retained FOTA URL
 * re-flashes the same image on every boot. The cloud contract is retain=false
 * on every command topic; this is the device-side backstop. Anything arriving
 * on a one-shot command topic within this window after subscribe_all() is
 * acknowledged as ignored and dropped. Retained deliveries land within a
 * second of the SUBSCRIBE, but this task drains them only after the boot-time
 * announcements (each a blocking publish), so the window is sized to cover a
 * slow link with margin; a genuine command that falls inside it is simply
 * re-sent by the operator, who sees the ack. setconfig is exempt: its own
 * byte-identical guard handles the retained case without losing a real push. */
#define CMD_RETAIN_GRACE_MS  15000
/* The panel lock keeps its OWN, shorter grace to match cng-cpil's lock contract
 * byte-for-byte (lock_cmd.h LOCK_RETAIN_GRACE_MS): a human pressing unlock and a
 * broker re-delivering a retained one are told apart by the 5 s window, and the
 * lock ack/refusal rides <base>/<imei>/lock/out, not <base>/data. The other
 * one-shot commands (restart/erase_config/factory_reset/FOTA) keep CMD_RETAIN_GRACE_MS. */
#define LOCK_RETAIN_GRACE_MS  5000

/* ── Connectivity recovery (layered escalation) ──────────────────────────── */
#define MQTT_RECONNECT_RETRIES 5      /* L1: plain MQTT reconnect attempts   */
#define NET_BRINGUP_RETRIES    3      /* L2: network rebuild attempts        */
#define MODEM_PWRCYCLE_MAX     3      /* L3: modem PWRKEY power-cycles       */
#define MQTT_PUB_FAIL_MAX      10     /* consecutive publish fails → link down */
/* Broker-only outage (cellular still registered): how often to re-probe the
 * broker. The escalation ladder deliberately does NOT run here — rebooting or
 * power-cycling cannot reach a broker that is down — so this interval is what
 * bounds how long after the broker returns the fleet takes to reconnect and
 * start flushing its backlog. Each probe is one QMTOPEN, so keep it cheap. */
#define MQTT_BROKER_OUTAGE_RETRY_S 30
/* Consecutive "the bearer is up, so it must be the broker" verdicts before the
 * modem's own "+QIACT: 1,1" stops being believed and the context is rebuilt
 * anyway.
 *
 * network_data_bearer_up() fixes the case where the modem honestly reports the
 * PDP context down. It cannot fix the case where the baseband keeps printing
 * "1,1" for a context that is dead — and that state exists. Without this
 * counter the ladder would park at L1 forever on exactly the fault it was
 * rewritten to escape, just one layer further in.
 *
 * The ladder re-probes every MQTT_BROKER_OUTAGE_RETRY_S (30 s) and spends
 * MQTT_RECONNECT_RETRIES (5) attempts before each check, so 8 checks is on the
 * order of twenty minutes of unbroken outage: long enough that a routine broker
 * restart never triggers a needless rebuild, short enough that a stale context
 * is not a site visit. Any successful connect resets it. */
#define MQTT_BEARER_DISTRUST_AFTER 8

/* ── Dispenser units ──────────────────────────────────────────────────────
 * SIX dispenser slots (du1..du6: du<N>Make + du<N>Sfx), any mix of BlueSky and
 * Yenen — the same slot rule as the btpl meters (2026-10-16).
 *
 * A SLOT IS ON EXACTLY WHEN ITS SUFFIX IS SET, AND THE SUFFIX IS THE DISPENSER
 * NUMBER, WHICH IS THE BUS ADDRESS. A suffix is two DIGITS, 01..06 (Yenen
 * 01..05 — see DU_YENEN_MAX). du2Sfx:"02" polls dispenser 2: a BlueSky on
 * nozzle addresses 3 and 4 (2n-1, 2n), publishing nozzle labels "3" and "4"
 * under HW_ID <imei>02; a Yenen as head 2. du<N>Sfx "" turns slot N OFF —
 * there is NO position fallback any more. A station whose dispenser 1 is out
 * of service sends du1Sfx "" and keeps du2Sfx "02" / du3Sfx "03": only those
 * two are polled, and they still publish as 02 and 03. Nothing is ever
 * re-addressed at the pump. duCount is a cross-check (it must equal the slots
 * ON), not a switch. Two slots naming the same dispenser are refused
 * (validate_suffixes). The two protocols never collide on the wire because
 * they use different line settings and framing.
 *
 * du_number_of() (nvs_config.c) is the ONE place a suffix is turned into a
 * number; every address, label and HW_ID derives from it, so the cloud's
 * HW_ID and the bus address cannot disagree. Every runtime loop walks the
 * DU_MAX_DISPENSERS slots and skips the OFF ones (du_slot_on). */
#define DU_MAX_DISPENSERS  6

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
 * only every KFACTOR_POLL_INTERVAL_S) and RSP are retried the same. RSP used
 * to be a single read "so a normal round stays short" — that was when every
 * read cost the full 2 s silence timeout; a read now ends on the frame's own
 * length (~30 ms), so the retries only cost time when the read is actually
 * failing, which is exactly when they earn it. A read that fails every attempt
 * degrades to "404" for that field this cycle. */
#define DU_TOTAL_RETRIES    3
#define DU_RSP_RETRIES      3
#define DU_KFACTOR_RETRIES  3

/* ── Yenen dispenser units ────────────────────────────────────────────────
 * The second dispenser protocol this build supports, alongside BlueSky, in
 * the same station list (position = head number, see above).
 *
 * DU_YENEN_MAX is 5, NOT 6 like BlueSky, and that is a protocol limit rather
 * than an arbitrary cap. A Yenen nozzle is addressed by a single ASCII digit:
 * side A is '0'+(head-1) and side B is '5'+(head-1), so head 6 side A would
 * encode as '5' — already taken by head 1 side B. Verified against every frame
 * in the vendor doc (DU1 -> '0','5'; DU2 -> '1','6'). Because the head IS the
 * dispenser number (the suffix), a Yenen dispenser may be numbered 01..05
 * only: parse_setconfig() refuses a Yenen numbered 06 and nvs_config_load()
 * drops such an entry if a stored plan carries one. If Yenen documents a
 * different encoding beyond 5 heads, raise this and revisit yenen_nzcode(). */
#define DU_YENEN_MAX  5

/* Line settings — 8N1. Parity NONE is CONFIRMED (2026-08-19); BlueSky's 8E1
 * is the odd one out on this bus, which is why each poll re-asserts its own
 * mode per nozzle. Baud is still the assumption: the vendor doc lists frames
 * but no line rate, and 9600 matches the rest of the bus. With parity now
 * known-good, garbled frames (rather than silence) would point at baud. */
#define YENEN_BAUD    9600

/* Attempts for the ONE command this driver sends (total, not extra — matches
 * the DU_*_RETRIES convention above). Command 2 carries the totalizer the
 * cloud differences, so it gets the full sale-critical retry budget.
 *
 * YN_TXN_RETRIES / YN_TXN_GIVEUP_AFTER / YN_TXN_REPROBE_S are gone with
 * Command 1 (2026-08-20). The auto-disable state machine they drove existed
 * only to stop a pump that does not implement Command 1 from burning 6 s of
 * dead air per nozzle per round; not sending the command at all is the simpler
 * form of the same fix, and it removes the 3-round learning period and the
 * 15-minute re-probe burst along with it. See yenen.h. */
#define YN_TOTAL_RETRIES  3
#define YN_RETRY_MS       300

/* ── Radar ─────────────────────────────────────────────────────────────────
 * Ztech and Pantech are standard Modbus RTU on the shared RS485 bus.
 * THE SLAVE ADDRESS IS THE HW_ID SUFFIX: a tank whose sfx is "04" is polled at
 * slave 4, so what the cloud sees and what the bus is asked agree by
 * construction (get_radar_slave(), main.c). An empty suffix falls back to the
 * field convention — Ztech from ZTECH_SLAVE_BASE upwards, Pantech continuing
 * after the last Ztech tank — and the fallback value is also the auto suffix,
 * so the equivalence holds either way. Suffixes are two DIGITS (01..99) on
 * every unit in this product; letters are refused by validate_suffixes().
 *
 * Because addresses are per-tank, there are no fixed per-type ranges to keep
 * apart. validate_suffixes() rejects a plan in which two active units share a
 * suffix, and validate_bus_addresses() rejects one in which two Modbus units
 * (any radar tank, the genset) resolve to the same slave — the two checks
 * together replace the old fixed-range guards. */
#define RADAR_MAX          4
#define ZTECH_SLAVE_BASE   4
/* Attempts per tank per radar poll before it reports 404 (total, not extra).
 * Applied uniformly to every radar type (Ztech/Pantech/future) at the poll
 * call site, so a new radar driver inherits the retry with no extra work. */
#define RADAR_READ_RETRIES 3

/* ── Modbus reply timing — radar (Ztech/Pantech) and DG ──────────────────────
 * A device gets MODBUS_REPLY_TIMEOUT_MS from our request to START its reply —
 * a working one answers in ~10-100 ms, and 1 s is the usual Modbus master
 * default. Once bytes flow the read ends at the expected length, or after
 * MODBUS_BYTE_GAP_MS of silence (an exception, or a reply that stopped short),
 * and never later than 2 x MODBUS_REPLY_TIMEOUT_MS after the request, so line
 * noise trickling in cannot hold the bus. The largest reply, SGC120's 161
 * bytes, takes 335 ms at 4800 baud, well inside that.
 *
 * A failed attempt is retried after MODBUS_RETRY_GAP_MS. The gap lets a late
 * reply finish before we transmit again — sending over it would garble both —
 * and the RX flush before every request then discards it.
 *
 * Until 2026-09-28 radar waited 200 ms then listened 500 ms, DG listened 500 ms
 * for the whole reply, and both retried after 100 ms. */
#define MODBUS_REPLY_TIMEOUT_MS  1000
#define MODBUS_BYTE_GAP_MS        100
#define MODBUS_RETRY_GAP_MS       500

/* ── Genset ─────────────────────────────────────────────────────────────── */
#define GENSET_OFF          0
#define GENSET_SEDEMAC      1
#define GENSET_SLAVE_DEFAULT 10   /* DG default Modbus slave ID */
#define GENSET_READ_RETRIES  3    /* attempts per DG poll before reporting 404 (total, not extra) */

/* Genset model — selects the Modbus read set (see g_models[] in genset.c). Per
 * the DG integration spec each poll issues up to THREE reads, and each is
 * published as its OWN message: a measurement block under "registers", the
 * alarm registers under "Alarms" (Mahindra only; the spec's capital A) and a
 * DG status block under "status". Each carries the read's complete validated
 * reply frame as a comma-separated hex string (slave, FC, byte count, data and
 * CRC, nothing removed) — the same shape radar and the dispensers use. The
 * suffix (sfxGenset) is the slave ID the request is addressed to: "08" sends
 * 08 03/04 ... and only a controller set to slave 8 answers. The firmware
 * never interprets a genset register; the cloud owns
 * every register map and all scaling. Reads per model (all CRC-verified against
 * the spec's documented captures):
 *   GC1211        → Mahindra, FC04: registers 1..36 (72 B), Alarms 57..59
 *                   (6 B), status 67..68 (4 B)
 *   GC1111        → Mahindra, FC03: same three reads as GC1211 — the two differ
 *                   only in the function code, which the spec documents.
 *   SGC120        → FC03: one overall read, registers 1..78 (156 B). No Alarms
 *                   or status command; the single block holds both (alarms
 *                   65..76, status word 78).
 *   GC1201/02/03  → Ashok Leyland ("Bottling Plant DG", titled "DG 1202/03/04"
 *                   in the spec), FC04: registers 1..36 (72 B), status 67
 *                   (2 B — one register, unlike the Mahindras). No alarms.
 *   GC902         → NOT covered by the spec; mapped to the Ashok Leyland read
 *                   set, which the vendor register map lists as one shared
 *                   family map. Verify on real GC902 hardware before relying.
 * Adding a model: add one g_models[] row. Bump GENSET_MODEL_MAX. */
#define GENSET_MODEL_GC1211  0
#define GENSET_MODEL_GC1111  1
#define GENSET_MODEL_SGC120  2
#define GENSET_MODEL_GC1201  3
#define GENSET_MODEL_GC1202  4
#define GENSET_MODEL_GC1203  5
#define GENSET_MODEL_GC902   6
#define GENSET_MODEL_MAX     6   /* highest valid model id — update when adding */

/* ── Digital input FIFO ─────────────────────────────────────────────────────
 * Transitions waiting to be published — during an outage, every change since
 * the link dropped. Each carries the time it was seen (di.c), so a replay
 * after the outage still says WHEN the DG started or the mains failed. Full =
 * the OLDEST is dropped. 64 x ~124 B ≈ 8 KB of RAM (was 20 x 256 B = 5 KB). */
#define DI_FIFO_SIZE  64

/* ── Offline record queue (store.c) ────────────────────────────────────────
 * Backed by littlefs on the `storage` partition — 0x1e0000 = 1920 KB, per
 * partitions.csv. Every dispenser payload goes through here during an outage
 * (see the corrected note in store.h; it is no longer the rare
 * publish-failure path it was designed as).
 *
 * ── Why STORE_MAX_BYTES is 768 KB and not 1 MB ────────────────────────────
 * compact() rewrites the live log into a temporary file BEFORE deleting the
 * original, so the partition has to hold both copies at once. At the former
 * 1 MB ceiling that is 2 MB against a 1920 KB partition (less littlefs
 * metadata and the free blocks its wear levelling needs) — so compacting a
 * full log would fail with ENOSPC, leaving consumed space permanently
 * unreclaimed and the log growing until every write failed.
 *
 * The rule is: 2 x STORE_MAX_BYTES must fit inside the partition with room to
 * spare. 768 KB + 768 KB = 1.5 MB leaves ~400 KB of headroom. If the partition
 * is ever resized, resize this with it.
 *
 * 768 KB still holds ~3,900 records — about 6 hours of a two-nozzle site at
 * the 12 s cadence, and longer as sale volume rather than poll volume goes up. */
/* Longest single record. MUST stay strictly below DU_PAYLOAD_MAX, because
 * store_peek() reads into a DU_PAYLOAD_MAX buffer with fgets(), which stores at
 * most (size - 1) characters plus the NUL. At 1024/1024 a maximum-length record
 * came back TRUNCATED — the publish would carry a cut-off JSON object the cloud
 * silently drops, and the read cursor would still be correct so nothing looked
 * wrong locally. 1000 leaves clear headroom; a static assert in store.c pins the
 * relationship so it cannot drift again. */
#define STORE_REC_MAX     1000
#define STORE_MAX_BYTES   (768 * 1024)
#define STORE_COMPACT_AT  (256 * 1024)  /* rewrite to reclaim consumed space past this */
/* Slack freed when the queue hits STORE_MAX_BYTES. The trim drops down to
 * (STORE_MAX_BYTES - STORE_TRIM_BYTES) rather than to exactly the ceiling, so
 * the expensive drop+compact runs once per ~650 records instead of once per
 * record — see drop_oldest_until_fits(). */
#define STORE_TRIM_BYTES  (128 * 1024)
/* Wait for the queue lock. Generous because a littlefs compaction rewrites the
 * whole log and can take well over a second; a timeout here costs one dropped
 * record, so it should only fire if something is genuinely wedged. */
#define STORE_LOCK_TIMEOUT_MS  3000
/* Records replayed per poll cycle once the link returns. Deliberately small:
 * a backlog dumped all at once would hammer a link that has only just come
 * back, and each publish is a modem AT round-trip that blocks the poll loop.
 * Draining alongside live polling keeps current sales flowing while the
 * backlog clears in the background. */
/* Records drained from flash per cycle IN EXCESS of what one poll round
 * produces. The excess is what matters: the drain has to outpace production or
 * the backlog never clears. A fixed count does not — at 6 BlueSky + 5 Yenen a
 * round emits 22 payloads, so a flat 5/cycle would fall permanently behind and
 * the queue would grow without bound. The call site adds the live nozzle count
 * to this, guaranteeing net progress of STORE_FLUSH_PER_CYCLE per cycle at any
 * site size. */
#define STORE_FLUSH_PER_CYCLE  5
/* Hard ceiling on time spent draining per cycle. Each publish is a blocking
 * modem round-trip (~300-600 ms), so an unbounded drain would stall radar,
 * genset and DI for minutes after a long outage. The backlog then clears over
 * several cycles instead of one — slower, but the rest of the gateway keeps
 * working while it does.
 *
 * 9 s, not 15 s: this must stay under PUBLISH_INTERVAL_MS so radar/genset/DI
 * and inbound commands (polled inside the drain loop) still run every cycle,
 * and the cycle is now 12 s rather than 20 s. At ~0.6 s/publish, 9 s clears
 * ~15 records — still >= per_round + STORE_FLUSH_PER_CYCLE for a full 5-DU
 * Yenen site (10 + 5), so backlog recovery stays net positive there. At the
 * absolute maximum mixed config (22 + 5) it no longer is; that shape cannot
 * sustain a 12 s cadence on this link in the first place (see
 * PUBLISH_INTERVAL_MS), so raising this budget would not rescue it — message
 * batching is the real fix. Raising it above the cycle would only starve
 * radar/genset for the sake of a config that is already over its ceiling. */
#define STORE_FLUSH_BUDGET_MS  9000

/* ── Dispenser acquisition task (du_task.c) ────────────────────────────────
 * Dispenser polling runs on its own task on core 1 so it survives the
 * connectivity recovery ladder, which blocks the main task for minutes
 * (measured: 290 s with zero reads before a restart).
 *
 * DU_QUEUE_SLOTS is the RAM buffer between acquisition and publishing. At two
 * nozzles on a 12 s cycle it drains every round and never fills; it only backs
 * up once the main task is stuck in recovery, and 16 slots is about 1.5 min of
 * headroom before anything reaches flash. Sized against RAM cost: each slot is
 * a full DU_PAYLOAD_MAX buffer, so 16 x 1024 = 16 KB of the ~243 KB heap.
 * Raising it buys minutes of RAM-only buffering but does NOT remove the need
 * for flash — the ladder ends in a restart, and RAM does not survive that. */
#define DU_PAYLOAD_MAX     1024
#define DU_QUEUE_SLOTS     16
/* Floor on the gap between rounds, so a round that overruns PUBLISH_INTERVAL_MS
 * still yields the CPU instead of spinning the bus back-to-back. */
#define DU_TASK_MIN_GAP_MS 250

/* ── Absent-nozzle backoff ────────────────────────────────────────────────
 * A nozzle that does not answer costs a FULL retry budget every round — 6.6 s
 * (Yenen) or 15.8 s (BlueSky) by the measurements above. That dead time is
 * serial, so it stretches the cadence of the nozzles that DO work: a 4-nozzle
 * BlueSky site with two dead sides polls its live nozzles every ~52 s instead
 * of 20 s. Since the cloud differences consecutive totalizer readings, widening
 * that window is exactly what merges two sales into one transaction — the same
 * failure the bus-lock note above exists to prevent, arriving by a different
 * route. Single-sided DUs make this the normal case, not an edge case.
 *
 * So a nozzle that has been silent for AFTER_ROUNDS consecutive rounds is
 * demoted to a probe every RETRY_EVERY rounds, reclaiming its dead time for the
 * live nozzles.
 *
 * Both constants stay conservative, because the failure mode of being too
 * aggressive is worse than the dead time it saves — but AFTER_ROUNDS is tuned
 * for the fully-loaded site, where the OLD value hurt the survivors:
 *   - AFTER_ROUNDS 5 is ~1.75 min of UNBROKEN silence. A transient RS485 glitch
 *     or a noisy round (1-2 rounds) still cannot reach it. The trade vs the old
 *     10: at max config a mass dispenser power-loss (e.g. 6 BlueSky = 12 dead
 *     nozzles at ~15.8 s each = ~190 s rounds) reclaims the bus for the SURVIVING
 *     nozzles in half the time, so their totalizer cadence — and the sale
 *     resolution it protects — recovers twice as fast. A live nozzle demoted by
 *     a longer contention blip self-restores on its very next answer (see below),
 *     so the downside is bounded to ~RETRY_EVERY rounds on that one nozzle.
 *   - RETRY_EVERY 6 rediscovers a nozzle that comes back within ~2 min.
 *   - ANY valid response resets the counter immediately and restores full
 *     cadence on the very next round.
 * Critically, the counter advances on "did not respond at all", NOT on
 * "unhealthy": a BlueSky nozzle mid-sale reports busy rather than idle, and
 * treating that as absent would back off precisely the nozzle that is actively
 * dispensing — the worst possible moment to stop watching it. */
#define DU_ABSENT_AFTER_ROUNDS  5
#define DU_ABSENT_RETRY_EVERY    6

/* ── Waiting for the shared RS485 bus (rs485_bus.h) ────────────────────────
 * TWO timeouts, because the two waiters have opposite priorities.
 *
 * The single 8 s value these replace was based on "genset reads are the worst
 * case", which is simply not true — measured against the real retry budgets,
 * one nozzle can hold the bus for far longer than a genset poll:
 *
 *   genset  1 attempt = 3 reads x MODBUS_REPLY_TIMEOUT_MS ~3.1 s  (silent
 *                       Mahindra; the two-read models ~2 s)
 *   radar   1 tank    = 3 x 1 s reply window, 500 ms apart ~4 s
 *   Yenen   1 nozzle  = 3 reads x RS485_RECV_TIMEOUT_MS   ~6.6 s  (silent pump;
 *                       was ~13.2 s until Command 1 was dropped — yenen.h)
 *   BlueSky 1 nozzle  = 7 reads x RS485_RECV_TIMEOUT_MS  ~15.8 s  (answers
 *                       status, then every data read times out)
 *
 * So at 8 s a dispenser round routinely timed out radar/genset AND — worse —
 * a slow pump timed out the ACQUISITION task, silently dropping a whole
 * nozzle round. Dropping dispenser rounds is the one thing this firmware must
 * not do: the cloud reconstructs sales by differencing consecutive totalizer
 * readings, so a skipped round merges two sales into one.
 *
 * DU_TASK  waits long enough to never lose a round to bus contention. It can
 *          afford to: it is only ever waiting on radar/genset, both bounded
 *          at ~4 s (~7 s under a noise storm, see MODBUS_REPLY_TIMEOUT_MS),
 *          so this ceiling is headroom that should never be spent.
 * MAIN     waits about one worst-case nozzle. Radar and genset are sampled
 *          state on 10-minute and 5-minute cadences — skipping one and
 *          publishing "404" costs nothing, and the next reading is current.
 *          Keeping this bounded also keeps the main task responsive to an
 *          inbound panel-unlock, and keeps a 4-tank radar round (4 x this,
 *          worst case) comfortably inside WDT_TIMEOUT_MS. */
#define RS485_BUS_LOCK_TIMEOUT_MS  25000   /* du_task — acquisition, must not skip */
#define RS485_BUS_WAIT_MAIN_MS      9000   /* radar / genset / rsp tunnel          */

/* ── Thermal zones (°C) — tuned for 24/7 enclosed operation, no heatsink ─── */
#define THERMAL_NORMAL_MAX_C    65.0f
#define THERMAL_WARM_MAX_C      75.0f
#define THERMAL_CRITICAL_MIN_C  80.0f
