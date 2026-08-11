# VVM701 Gateway — Cloud Integration Reference

Firmware 1.0.0. Last updated 10 August 2026.

This document defines the MQTT message contract between the field gateways and the backend: every topic, the exact JSON of every payload, and the commands the device accepts. Throughout, `<base>` is the MQTT base topic (default `fuelsense`) and `<imei>` is the gateway's 15-digit modem IMEI.

## 1. Topics

All device-to-cloud messages publish to a single fleet-wide topic, `<base>/data`. It is not IMEI-scoped; every gateway's telemetry, health, and status all arrive here. Two things are read from the payload, never from the topic:

- Which device or unit sent it: the `HW_ID` field.
- What kind of message it is: which top-level key is present. Each message has exactly one of `nozzles`, `tanks`, `genset`, `di`, `gps`, `diagnostics`, `sysmon`, or `status`.

All traffic is QoS 1. The device never sets the MQTT retain flag. The ingestion pipeline stores each payload verbatim, so the schemas below are also the stored `raw_data` schemas.

| Topic | Direction | Payload |
|---|---|---|
| `<base>/data` | Device → Cloud | Telemetry, health, config acknowledgements, lifecycle status (JSON) |
| `<base>/<imei>/setconfig` | Cloud → Device | Configuration and `restart` / `factory_reset` / `getconfig` (JSON) |
| `<base>/<imei>/update` | Cloud → Device | FOTA trigger for one device (HTTPS URL string) |
| `<base>/update` | Cloud → Device | FOTA trigger for the whole fleet (HTTPS URL string) |
| `<base>/rsp/imei/<hw_id>/in` | Cloud → Device | Dispenser RSP command (hex) |
| `<base>/rsp/imei/<hw_id>/out` | Device → Cloud | RSP response (hex, or `404`) |
| `<base>/kfactor/imei/<hw_id>/in` | Cloud → Device | Dispenser K-factor command (hex) |
| `<base>/kfactor/imei/<hw_id>/out` | Device → Cloud | K-factor response (hex, or `404`) |

The `rsp` and `kfactor` `/out` responses are bare hex, not JSON, and carry no `HW_ID`. Match them by their topic, which embeds the unit's HW_ID and therefore changes if the unit's suffix is reconfigured. The inbound command topics stay per-IMEI so a command targets exactly one device.

## 2. Identity and HW_ID suffixes

The gateway's root identity is its IMEI, which appears as `device_id` in the pipeline. Each piece of equipment behind the gateway is tagged with a HW_ID:

```
HW_ID = IMEI + <2-character suffix>
```

The device assigns each unit a numeric 2-character suffix automatically, so a standard station needs no suffix configuration at all. Each equipment type has its own number range, so the ranges never overlap even on a fully-loaded gateway.

| Equipment | setconfig key | Auto suffix | Example HW_ID |
|---|---|---|---|
| BlueSky dispenser 1–6 | `sfxBs1` … `sfxBs6` | `01` … `06` | `86071008133250701` |
| Genset | `sfxGenset` | `gensetSlave` (default `10`) | `86071008133250710` |
| Ztech radar 1–4 | `sfxZtech1` … `sfxZtech4` | `21` … `24` | `86071008133250721` |
| Pantech radar 1–4 | `sfxPantech1` … `sfxPantech4` | `31` … `34` | `86071008133250731` |
| Digital inputs | `sfxDI` | `40` | `86071008133250740` |

The number is a cloud identifier only; it is not the unit's RS485 or Modbus address. For dispensers and radars the last digit is the unit number within its type (Ztech tank 2 is `22`, Pantech tank 2 is `32`). The genset is the one exception: it uses its configured `gensetSlave`, which is 10 by default.

A suffix must be exactly two alphanumeric characters. You can override any unit's auto value by sending its `sfx*` key in setconfig (section 6.1); an explicit value always wins. In a `config_report`, an empty `sfx*` means the unit is using its auto value from the table above.

Because the auto ranges are disjoint, a gateway carrying the maximum of everything — 6 dispensers, 4 Ztech tanks, 4 Pantech tanks, a genset and DI — still produces 16 distinct HW_IDs with no collision. A collision is only possible through a custom suffix that clashes with another unit (for example setting a dispenser to `10` while a genset is active); the device rejects such a configuration rather than publishing it, but validate uniqueness in the dashboard first so the operator gets an immediate error.

The suffix is per equipment unit, not per nozzle. A dispenser unit publishes both of its nozzles under one HW_ID; the two nozzles are the `"1"` and `"2"` keys inside the `nozzles` object (section 4.1).

Status, diagnostics, sysmon, and GPS messages use the bare IMEI (no suffix), because they describe the gateway itself rather than a unit behind it. No equipment ever uses the bare IMEI, so device-level messages never overlap with unit telemetry.

## 3. Conventions used across payloads

Dispensers and radars are transparent bridges: the device forwards the raw protocol frame as a comma-separated hex string and the backend decodes it. Decoding those frames needs the vendor references (the BlueSky dispenser command set and the Ztech/Pantech tank Modbus maps), which are separate from this guide. Genset is also raw, sent as Modbus register values rather than a frame, and its full decode maps are included here in Appendix A. Digital inputs and GPS arrive as ready-to-use JSON numbers.

The `404` marker. When a read times out or fails validation, the device publishes `404` for that item instead of dropping the message. This distinguishes "the gateway is alive but this unit did not answer" from "the gateway is offline." It is the string `"404"` for dispenser, radar, and genset, and the numeric `404` for digital inputs.

The `TS` field. Every telemetry payload carries a `TS` string formatted `YYYY-MM-DD HH:MM:SS`, taken from the cellular network clock with the timezone offset stripped (it is effectively UTC, and in production has run 5 hours 30 minutes behind the pipeline's IST `report_time`). Do not use `TS` for ordering or analytics; use the pipeline's `report_time`. `TS` is only a rough device-side reference.

Truncation fallback. Every payload must fit the modem's transmit ceiling of roughly 1.4 KB. If a message would exceed it, the device sends a minimal shape instead of a cut-off one. Genset and the other object payloads fall back to `{"...":{"data":"404"},...}`; diagnostics, sysmon, and the config read-back keep their wrapper key and add an `"error"` field. Treat any `error` field as "this cycle's data did not fit; the device is otherwise healthy."

## 4. Telemetry payloads (device to cloud)

All telemetry publishes to `<base>/data`, each message carrying the HW_ID of the unit it describes.

### 4.1 Dispenser

One message per nozzle, published as each read completes. A dispenser is polled every 20 seconds.

```json
{"HW_ID": "86071008133250701",
 "nozzles": {"1": {"data": "f5,01,...", "rsp": "f5,01,...", "kfact": "f5,01,..."}},
 "TS": "2026-08-08 05:27:59"}
```

The keys inside `nozzles` are global 1-based nozzle numbers as strings: dispenser 1 publishes `"1"` and `"2"`, dispenser 2 publishes `"3"` and `"4"`, and so on. `data` is the totalizer read and is sent every cycle. `rsp` is the RSP read, also every cycle. `kfact` is the K-factor read; it is refreshed every 15 minutes and the last value is repeated in between. All three are raw hex.

There are two failure shapes. A nozzle that is absent or busy collapses to `{"data": "404"}` with no other keys. A nozzle that responded but had one read fail carries `"404"` in that field alone, with real hex in the others.

The 20-second, per-nozzle cadence exists because the backend reconstructs each sale from the difference between consecutive totalizer reads. A tight cadence keeps two back-to-back sales on one nozzle from merging into a single delta.

### 4.2 Radar (tank level)

One message per tank, polled every 10 minutes.

```json
{"HW_ID": "86071008133250721", "tanks": {"1": {"data": "04,03,04,2f,13,00,00,56,22"}}, "TS": "2026-08-08 05:27:59"}
```

`data` is the raw Modbus RTU response frame in hex; the backend decodes the register bytes into level and volume. Ztech and Pantech number their tanks from `"1"` independently, and because they carry different HW_IDs the numbering does not collide. A failed or CRC-invalid read gives `{"data": "404"}`.

### 4.3 Generator set

One message every 5 minutes when a genset is configured. The device forwards the raw Modbus register values and the backend decodes them using the maps in Appendix A. All fields are raw integers.

```json
{"HW_ID": "86071008133250710",
 "genset": {"registers": [319, 62, 34, 137, 137, 1352, 1849, 1500, 499, ...],
            "alarms": [4368, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "status": 49152, "commandStatus": 1},
 "TS": "2026-08-08 05:27:59"}
```

- `registers`: raw measurement registers, index 0 being the model's first. Appendix A gives the index-to-field mapping and scale per model.
- `alarms`: raw alarm registers, each holding four alarm nibbles. Appendix A names them and gives the extraction.
- `status`: raw DG status word; `auxDiag` (GC902 only): digital I/O bitfield. Bit positions in Appendix A.
- `commandStatus`: last command result. 1 Started, 2 Stopped, 3 AlreadyON, 4 AlreadyOFF, 5 Manual, 6 NotStarted; 0 or anything else Unknown.

Which keys each model sends, and the read used to produce them:

| `gensetModel` | Controller | Function code | `registers` | `alarms` | `status` | `commandStatus` | `auxDiag` |
|---|---|---|---|---|---|---|---|
| 0, 1 | Sedemac GC1211, GC1111 | 04 | 36 | 10 | yes | yes | — |
| 2 | DEIF SGC 120 Mk II | 03 | 62 | 12 | yes | — | — |
| 3, 4, 5 | Sedemac GC1201, GC1202, GC1203 | 04 | 36 | — | yes | — | — |
| 6 | Sedemac GC902 | 04 | 47 | 9 | yes | yes | yes |

If the genset poll fails, the whole `genset` value is `{"data": "404"}`.

### 4.4 Digital inputs

Digital inputs are event-driven, not periodic. The device samples the inputs about every 2 seconds and publishes only when a value changes. A steady input produces no traffic, so do not treat silence as a fault or assume a fixed cadence.

```json
{"HW_ID": "86071008133250740", "di": {"DI1": 0, "DI2": 1}, "TS": "2026-08-08 05:27:59"}
```

Only the channels enabled in `diEnable` appear. Values are `0` or `1`, or the numeric `404` if a channel could not be read. Because a transition is queued and flushed on the next cycle, the `TS` on a DI message is the flush time, which can be up to a cycle after the transition.

### 4.5 GPS

GPS ships disabled and is off on every deployed gateway (`gpsEnable` defaults to false), so do not assume GPS data is present. When enabled, one message publishes roughly every 20 seconds using the bare IMEI:

```json
{"HW_ID": "860710081332507", "gps": {"lat": 12.887630, "lon": 77.641710}, "TS": "2026-08-08 05:27:59"}
```

`lat` and `lon` are decimal degrees, positive north and east. No fix yet or a failed query gives `{"gps": {"data": "404"}}`. No accuracy or confidence figure is sent.

### 4.6 Diagnostics and sysmon

These are two health reports, published back to back about every 15 minutes with the bare IMEI. `diagnostics` is the operational view for a fleet health dashboard; `sysmon` is a superset adding device internals, for investigating a specific gateway. They are told apart by their top-level key.

```json
{"HW_ID": "860710081332507",
 "diagnostics": {"fw": "1.0.0", "temp_c": 46.5, "thermal_zone": "normal",
                 "heap_free": 274160, "rssi": 26, "vbat_mv": 3820,
                 "operator": "Airtel", "uptime_s": 5231,
                 "pub_ok": 412, "pub_fail": 3, "loop_max_ms": 1840,
                 "du_ok": true, "radar_ok": true, "genset_ok": false,
                 "di_ok": true, "gps_ok": false},
 "TS": "2026-08-08 05:27:59"}
```

The `sysmon` message has the same envelope with a `"sysmon"` key, every field above (except `temp_c`, which it calls `cpu_temp_c`), plus: `cpu_freq_mhz`, `heap_min_ever`, `heap_largest_block`, `reset_reason`, `boot_count`, `ota_partition`, `main_stack_hwm`, `di_stack_hwm`.

- `thermal_zone`: `normal` below 65 °C, `warm` 65–75, `hot` 75–80, `critical` at or above 80.
- `rssi`: raw modem CSQ (0–31, or 99 for unknown), not dBm. To convert: dBm = -113 + 2 × rssi.
- `pub_ok` and `pub_fail`: publish counters for the reporting window, reset after a confirmed diagnostics publish. `loop_max_ms` is the worst poll-cycle duration in the window.
- The `_ok` flags reflect the most recent poll of each subsystem, as a logical AND across all units of that type. A subsystem that is not configured keeps its default and means nothing. `gps_ok` is false on virtually every gateway because GPS is off; do not alert on it alone.
- `reset_reason`: ESP-IDF reset-reason enum. 1 power-on, 3 software restart (normal), 4 crash/panic, 5 interrupt watchdog, 6 task watchdog, 7 other watchdog, 8 deep-sleep wake, 9 brownout. Recurring 4, 6, or 9 is a real fault (crash loop, hang, power instability). `boot_count` is persistent; a rising count with low `uptime_s` is a reboot loop and should alert.
- `ota_partition` (`ota_0` or `ota_1`) is the running firmware slot; it flips after each successful FOTA.

## 5. Status and lifecycle events

These publish on `<base>/data` with the bare IMEI. They are one-shot events.

Boot. `{"HW_ID":"...","status":"ready","fw":"1.0.0"}` means booted and online with at least one unit configured. `{"HW_ID":"...","status":"unconfigured","fw":"1.0.0"}` means booted and online but nothing is configured yet, which is the cue to onboard the gateway.

Configuration. Every well-formed `setconfig` produces an ordered sequence:

1. `{"status":"config_received"}` the instant the payload arrives, before validation.
2. Then one of: `{"status":"config_successful","fw":"1.0.0"}` (validated and saved; the device reboots immediately after) or `{"status":"config_failed","reason":"<reason>"}` (rejected, no reboot; nothing changed except in the `nvs_save_failed` case).

Do not treat a change as committed until `config_successful`. For connectivity changes, `config_successful` means the change was accepted for trial, not that it works (section 6.1).

Config failure reasons: `bad_payload`, `radar_mixed_style_use_radarType+tankCount_OR_ztechCount/pantechCount`, `radar_incomplete_send_radarType_and_tankCount_together`, `radar_type_invalid_use_ztech_pantech_or_none`, `gensetSlave <n> collides with radar slave range` (match on the prefix), `ztechCount <n> and pantechCount <n> produce overlapping radar slave ranges` (match on the suffix), `empty_apn`, `bad_mqtt_broker_fields`, `invalid_chars_in_connectivity_fields`, the suffix errors (`<Label> suffix must be exactly 2 alphanumeric characters`, `more than one equipment has no suffix assigned (bare IMEI)`, `<A> and <B> have duplicate suffix <sfx>`), and `nvs_save_failed`.

Remote reboot and reset. `{"status":"restarting"}` is sent just before a `restart` reboots, and `{"status":"factory_resetting"}` just before a `factory_reset` wipes and reboots.

FOTA. The sequence is `fota_initiated` (last message before the device drops offline to download), then `fota_successful` (flashed and reconnecting to report, not yet confirmed good), then `fota_completed` with `fw` (the new image survived a full poll cycle, which is the real success signal). Failure is `fota_failed` with a `reason`, or `fota_skip_known_bad` when a URL identical to one that already failed this session is refused. The most common reason is `http_<code>_err_<err>`: code 404 means a wrong URL, 403 means forbidden, and -1 means the request never reached the server. Other reasons include `non_https_url`, `get_timeout`, `bad_content_length`, `readfile_timeout`, `image_verify_failed`, and `network_rebuild_failed`.

Connectivity rollback. `{"status":"connectivity_rollback","reason":"new_settings_unreachable"}` is sent once, after the device reconnects on reverted settings, when a pushed APN or broker change could not reach a broker on the next boot (section 6.1).

## 6. Commands (cloud to device)

### 6.1 setconfig

Publish a JSON object to `<base>/<imei>/setconfig`. Only the keys you include change; anything omitted keeps its current value.

| Key | Type | Valid values | Controls |
|---|---|---|---|
| `duBlueskyCount` | int | 0–6 | Number of dispenser units |
| `radarType` + `tankCount` | string + int | `ztech`/`pantech`/`none`, 0–4 | Legacy radar config (sent as a pair) |
| `ztechCount` / `pantechCount` | int | 0–4 each | Radar config, explicit style (both types at once) |
| `gensetType` | int | 0 = off, 1 = on | Enable genset polling (not the family; see `gensetModel`) |
| `gensetModel` | int | 0–6 | Controller: 0 GC1211, 1 GC1111, 2 SGC120, 3 GC1201, 4 GC1202, 5 GC1203, 6 GC902 |
| `gensetSlave` | int | 1–247 | Genset Modbus address |
| `gensetBaud` | int | 4800, 9600, 19200 | Genset serial baud |
| `diEnable` | int (bitmask) | bit0 = DI1 … bit3 = DI4 | Which digital inputs are active |
| `gpsEnable` | bool | true / false | Enable GPS (leave false unless the GNSS antenna is confirmed connected) |
| `sfxBs1`…`sfxBs6`, `sfxZtech1`…`sfxZtech4`, `sfxPantech1`…`sfxPantech4`, `sfxGenset`, `sfxDI` | string | 2 alphanumeric, or empty | HW_ID suffix override (empty = auto, section 2) |
| `apn`, `apnUser`, `apnPass` | string | apn non-empty if any connectivity key is sent | Cellular APN |
| `mqttHost`, `mqttPort`, `mqttUser`, `mqttPass` | string / int | host non-empty; port 1–65535 | Broker |
| `mqttBaseTopic` | string | non-empty, no `#` or `+` | Topic prefix |
| `mqttTls` | bool | true / false | TLS on or off |

Out-of-range numeric values are coerced, not rejected, so validate them client-side: `gensetModel` above 6 becomes 0; an invalid `gensetBaud` becomes 9600; `gensetSlave` outside 1–247 becomes 10; `gensetType` other than 0 or 1 behaves as off. The resulting `gensetSlave` also cannot fall inside an active radar's Modbus range (Ztech from slave 4, Pantech from slave 6, each range growing with the configured count); a collision is rejected. Setting `ztechCount` to 3 or more together with any Pantech is also rejected, because the ranges would overlap. Prefer the explicit `ztechCount` / `pantechCount` style for new work.

Connectivity changes are a trial. If a `setconfig` touches any APN or broker field, the device saves the current values as a fallback and reboots to try the new ones. `config_successful` here means the change was accepted for trial. If the device then reaches a live broker on the new settings, they become permanent and a later `getconfig` shows `connPending: false`. If it cannot, the device reverts to the last known-good settings, reboots on those, and publishes `connectivity_rollback`. So for connectivity changes, wait for either a clean reconnect or a `connectivity_rollback` before concluding anything. When changing broker credentials, have the broker accept both the old and new credentials during the switch so the reconnect cannot fail whichever set it tries.

### 6.2 System commands

These also go to `<base>/<imei>/setconfig`:

- `{"cmd":"restart"}` acknowledges with `restarting`, then reboots.
- `{"cmd":"factory_reset"}` acknowledges with `factory_resetting`, erases stored config, and reboots `unconfigured`.
- `{"cmd":"getconfig"}` publishes the live configuration as a `config_report`, without changing anything or rebooting.

```json
{
  "HW_ID": "860710081332507", "status": "config_report", "fw": "1.0.0",
  "duBlueskyCount": 2, "ztechCount": 1, "pantechCount": 0,
  "gensetType": 1, "gensetModel": 0, "gensetSlave": 10, "gensetBaud": 9600,
  "diEnable": 3, "gpsEnable": false,
  "sfxGenset": "", "sfxDI": "", "sfxBs1": "", "sfxBs2": "", "sfxBs3": "", "sfxBs4": "",
  "sfxBs5": "", "sfxBs6": "", "sfxZtech1": "", "sfxZtech2": "", "sfxZtech3": "",
  "sfxZtech4": "", "sfxPantech1": "", "sfxPantech2": "", "sfxPantech3": "", "sfxPantech4": "",
  "apn": "airtelgprs.com", "apnUser": "", "apnPassSet": false,
  "mqttHost": "broker.example.com", "mqttPort": 8883, "mqttUser": "station01",
  "mqttPassSet": true, "mqttBaseTopic": "fuelsense", "mqttTls": true,
  "connPending": false
}
```

Every `sfx*` is empty here, which is the normal case: each unit uses its auto suffix from the table in section 2. For this example that gives HW_IDs `…01` and `…02` (the two dispensers), `…21` (the one Ztech tank), `…10` (genset), and `…40` (DI). A non-empty `sfx*` would mean that unit has been given a custom override.

Passwords are never echoed; `apnPassSet` and `mqttPassSet` only tell you whether one is stored. Treat the `config_report` as the authoritative equipment list for the dashboard: render a panel for every configured unit, even before it has reported.

### 6.3 FOTA trigger

Publish a plain HTTPS URL string (not JSON) to `<base>/<imei>/update` for one device, or to `<base>/update` for the whole fleet. The broadcast topic carries no IMEI, so every subscribed gateway will download; use it only for a deliberate fleet-wide rollout. Plain `http://` URLs are rejected. Leading and trailing whitespace, quotes, and newlines are stripped.

Do not publish FOTA URLs with the retain flag set. A retained URL is redelivered on every reconnect, so a bad one would be retried indefinitely. As a safeguard, a URL byte-identical to one that already failed this session is refused with `fota_skip_known_bad`; that guard is in RAM only and is forgotten on reboot. Clear or replace a retained message once the update lands or the URL is known bad.

### 6.4 Dispenser rsp and kfactor commands

Send a hex string to a unit's `in` topic and read the reply from its `out` topic. Separators are optional and ignored, and the input is case-insensitive. The reply is the raw dispenser response as hex, or the literal `404` if the dispenser did not answer. Expect a response within about a second.

## 7. Operational notes

- Nightly reboot. Each gateway reboots once a day around 01:30 IST (20:00 UTC), producing a silent gap of roughly 60–90 seconds. This is intentional; do not alert on it.
- Deriving offline. There is no explicit offline message; infer it from silence, allowing for the nightly reboot, FOTA downloads, and connectivity recovery. A reasonable starting threshold is 10 minutes of total silence, tuned against real fleet behaviour.
- Message correlation. Status and config acknowledgements carry no request ID, so serialize pushes to a given device and wait for a terminal response before sending the next. QoS 1 can redeliver a message; every status describes a state rather than a delta, so reprocessing a duplicate is safe.
- Offline queueing. The device uses a persistent MQTT session, so the broker queues commands while it is offline and delivers them on reconnect. Broker-side session or message expiry can drop a command sent during a long outage.
- Suffix changes. A unit's HW_ID changes when its suffix changes; historical data stays under the old HW_ID. Track the rename on the backend if continuity matters.

## 8. Security posture

- Transport is TLS with certificate-chain validation on both the broker and the FOTA server. The trusted-root bundle carries ISRG Root X1 (Let's Encrypt) and Amazon Root CA 1 (S3); an endpoint on any other CA fails closed until its root is added to the firmware. The chain is validated but the hostname is not, so the production certificate must match the hostname the device connects to.
- All gateways currently authenticate to the broker with a shared username. Move to per-device or per-batch credentials before fleet size makes rotation impractical.
- Secure boot and flash encryption are not yet enabled. Flash can be read or rewritten with physical access. Both are supported by the chip but are a one-way, provisioning-time decision (they burn eFuses and cannot be applied over the air). Enable Secure Boot V2 before turning on remote FOTA in the field.
- Access control is at the broker. `setconfig` has no authentication beyond the MQTT connection, so use broker ACLs to restrict which clients may publish to a device's topics.

## Appendix A — Genset register maps and decoding

Every value in `registers`, `alarms`, `status`, `commandStatus`, and `auxDiag` is a plain decimal integer from 0 to 65535 — the raw 16-bit Modbus register, with no scaling applied on the device. The cloud decodes it using the table for the configured `gensetModel`:

- Models 0, 1, 3, 4, 5 (GC1211, GC1111, GC1201, GC1202, GC1203) use table A.1.
- Model 6 (GC902) uses table A.2.
- Model 2 (DEIF SGC 120) uses table A.3.

Decoding rules:

- **Measurement:** `real value = register × scale`, using the scale in the table.
- **Signed fields** (marked "signed" in a table): if the value is greater than 32767, subtract 65536 before applying the scale (16-bit two's complement). Unmarked fields are unsigned.
- **32-bit fields** (cumulative energy, and SGC120 run time): combine the two listed registers as `(registers[lower] << 16) | registers[higher]` — the lower array index holds the high word. This word order follows the standard Modbus convention but has not been confirmed against a live controller, so verify it against a known reading before trusting the absolute value.
- **`alarms`:** each array entry is one register holding four alarm nibbles. For an entry value `V`, the four names listed left to right occupy the nibbles from most to least significant: `state(pos) = (V >> (12 - 4 × pos)) & 0xF` for `pos` 0 to 3. A state of 2–5 is an active alarm (2 Warning, 3 Shutdown, 4 ElecTrip, 5 Notification); 0 and 1 are inactive and should be skipped.
- **`status`, `auxDiag`:** read individual bits with `(value >> bit) & 1`, using the 0-indexed bit positions listed under each table.

Worked example (Sedemac, model 0 or 1):

```json
"registers": [420, 78, 64, 138, 134, 152, 4830, 1500, 500, 2401, "..."],
"status": 49152,
"alarms": [12816, 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

- Measurements: `registers[0]` 420 × 0.01 = 4.20 Bar oil pressure; `[1]` 78 = 78 °C; `[8]` 500 × 0.1 = 50.0 Hz; `[9]` 2401 × 0.1 = 240.1 V.
- `status` 49152 = 0xC000: bit 15 = 1 available, bit 14 = 1 running, bit 11 = 0 mode Manual.
- `alarms[0]` 12816 = 0x3210: nibbles 3, 2, 1, 0 left to right, so RadiatorWaterLevel = 3 Shutdown and LowFuelLevel = 2 Warning are active; the remaining two are inactive.

### A.1 Sedemac GC1211, GC1111, GC1201, GC1202, GC1203 — `registers` (36 values)

```
0      Oil pressure (0.01 Bar)          9-11   Gen L1/L2/L3-N voltage (0.1 V)
1      Engine temperature (1 C)         12-14  Gen L1-L2/L2-L3/L3-L1 voltage (0.1 V)
2      Fuel level (1 %)                 15-17  Load L1/L2/L3 current (0.01 A)
3      Charging alternator volt (0.1 V) 18-21  Gen L1/L2/L3/total active power (0.1 kW)
4      Battery voltage (0.1 V)          22-25  Gen L1/L2/L3/total apparent power (0.1 kVA)
5      Number of starts                 26-29  Gen L1/L2/L3/total reactive power (0.1 kVAr)
6      Engine run time (hours)          30-33  Gen L1/L2/L3/average power factor (0.01)
7      Engine speed (RPM)               34-35  Cumulative energy, 32-bit (0.1 kWh)
8      Genset frequency (0.1 Hz)
```

Status word bits: 15 available, 14 running, 13 stoppedNormally, 12 stoppedWithFault, 11 mode (1 = Auto, 0 = Manual). For example, `0xC000` is available and running; `0x2000` is stoppedNormally.

GC1201, GC1202, and GC1203 send `registers` and `status` only. GC1211 and GC1111 also send `alarms` and `commandStatus`.

`alarms` names, per index, most significant nibble first:

```
0  RadiatorWaterLevel, LowFuelLevel, HighEngTemp, LOP
1  FailToStop, FailToStart, OverSpeed, UnderSpeed
2  GensetHighFreq, GensetLowFreq, GensetHighVolt, GensetLowVolt
3  EmergencyStop, UnbalancedLoad, Overload, HighCurrent
4  AirFilterMaint, FuelFilterMaint, OilFilterMaint, ChargingAltFail
5  EngTempOpenCkt, OilPressOpenCkt, BattHighVolt, BattLowVolt
6  (reserved), (reserved), MPULoss, FuelTheft
7  AuxInputD, AuxInputC, AuxInputB, AuxInputA
8  AuxInputH, AuxInputG, AuxInputF, AuxInputE
9  (reserved), (reserved), (reserved), AuxInputI
```

### A.2 Sedemac GC902 — `registers` (47 values)

The measurement fields match A.1 but shifted by one (index 0 is protocol version), with additional registers at the end.

```
0      Protocol version                 16-18  Load L1/L2/L3 current (0.01 A)
1      Oil pressure (0.01 Bar)          19-22  Gen L1/L2/L3/total active power (0.1 kW)
2      Coolant temperature (1 C)        23-26  Gen L1/L2/L3/total apparent power (0.1 kVA)
3      Fuel level (1 %)                 27-30  Gen L1/L2/L3/total reactive power (0.1 kVAr)
4      Charging alternator volt (0.1 V) 31-34  Gen L1/L2/L3/average power factor (0.01)
5      Battery voltage (0.1 V)          35-36  Cumulative active energy, 32-bit (0.1 kWh)
6      Number of starts                 37-38  Cumulative apparent energy, 32-bit (0.1 kVAh)
7      Engine run time (hours)          39-40  Cumulative reactive energy, 32-bit (0.1 kVArh)
8      Engine speed (RPM)               41-46  Mains L1-N/L2-N/L3-N/L1-L2/L2-L3/L3-L1 volt (0.1 V)
9      Genset frequency (0.1 Hz)
10-12  Gen L1/L2/L3-N voltage (0.1 V)
13-15  Gen L1-L2/L2-L3/L3-L1 voltage (0.1 V)
```

Status word bits: 15 loadOnDG, 14 running, 13 stoppedNormally, 12 stoppedWithFault, 11 mode (1 = Auto, 0 = Manual). `auxDiag` bits: 15 down to 8 are Digital Input A through H; 7 down to 2 are Digital Output A through F. `commandStatus` uses the same values as A.1.

`alarms` names, per index, most significant nibble first:

```
0  LowOilPressure, HighCoolantTemp, LowFuelLevel, RadiatorWaterLevel
1  UnderSpeed, OverSpeed, FailToStart, FailToStop
2  GenLowVolt, GenHighVolt, GenLowFreq, GenHighFreq
3  GenHighCurrent, GenOverload, UnbalancedLoad, EmergencyStop
4  ChargeAltFail, OilFilterMaint, FuelFilterMaint, AirFilterMaint
5  BattLowVolt, BattHighVolt, OilPressOpenCkt, EngTempOpenCkt
6  FuelTheft, MPUFault, (reserved), (reserved)
7  AuxInputA, AuxInputB, AuxInputC, AuxInputD
8  AuxInputE, AuxInputF, AuxInputG, AuxInputH
```

### A.3 DEIF SGC 120 Mk II — `registers` (62 values, sparse)

```
0      Protocol revision                     52     Coolant temperature (0.1 C, signed)
1-3    Gen L1/L2/L3-N voltage (0.1 V)        53     Fuel level (divide by 10, inferred)
7      Generator frequency (0.1 Hz)          55     Charging alternator voltage (0.1 V)
23-25  Gen L1/L2/L3 current (0.1 A)          56     Battery voltage (0.1 V)
26-29  Gen L1/L2/L3/total power (0.1 kW, signed)  57  Engine speed (RPM)
51     Lube oil pressure (0.1 Bar, unverified)   58  Number of starts
                                             60-61  Engine run time, 32-bit (hours)
```

All other indices are reserved and read zero. Status word bits: 4 available, 8 running, 7 stoppedNormally, 6 stoppedWithFault, 11–13 mode (5 = Auto, 4 = Manual, 6 = Scheduler, otherwise Unknown). There is no `commandStatus`.

`alarms` names, per index, most significant nibble first:

```
0   LOP, HighCoolantTemp, LowFuelLevel, WaterLevelSwitch
1   UnderSpeed, OverSpeed, FailToStart, FailToStop
2   ReversePower, LowLoad, GenLowFreq, GenHighFreq
3   GenHighCurrent, GenOverload, UnbalancedLoad, EmergencyStop
4   ChargeAltFail, MaintenanceDue, AFTActTimeout, AshLoadMaint
5   BattLowVolt, BattHighVolt, TempCktOpen, LOPShortBatt
6   FuelTheft, MPUFault, OilPressOpenCkt, AuxInputI
7   AuxInputA, AuxInputB, AuxInputC, AuxInputD
8   AuxInputE, AuxInputF, AuxInputG, AuxInputH
9   GenL1LowVolt, GenL1HighVolt, GenL2LowVolt, GenL2HighVolt
10  GenL3LowVolt, GenL3HighVolt, DGPhaseRotation, MainsPhaseRotation
11  FuelLevelOpenCkt, VBeltBroken, ECUPreheatFail, HighOilPress
```

The SGC 120 alarm mapping and status bits are transcribed from DEIF documentation and have not yet been confirmed against a live fault. Sanity-check the first real alarm before relying on it.
