# VVM701 Gateway — Cloud Integration Reference

Firmware 1.0.0. Last updated 13 August 2026.

> **Format change in this revision.** Three things changed: (1) the genset now sends **raw register bytes as hex**, split across **two messages** per poll — `registers` and `status` — instead of decoded `registers`/`alarms` arrays; see section 4.3 and Appendix A. (2) The separate `diagnostics` and `sysmon` health messages are now a **single `diagnostics` message** — see section 4.6. (3) The **on-device nightly reboot has been removed** — maintenance reboots are now entirely cloud-scheduled via `{"cmd":"restart"}`; see sections 6.2 and 7.

This document defines the MQTT message contract between the field gateways and the backend: every topic, the exact JSON of every payload, and the commands the device accepts. Throughout, `<base>` is the MQTT base topic (default `fuelsense`) and `<imei>` is the gateway's 15-digit modem IMEI.

## 1. Topics

All device-to-cloud messages publish to a single fleet-wide topic, `<base>/data`. It is not IMEI-scoped; every gateway's telemetry, health, and status all arrive here. Two things are read from the payload, never from the topic:

- Which device or unit sent it: the `HW_ID` field.
- What kind of message it is: which top-level key is present. Each message has exactly one of `nozzles`, `tanks`, `genset`, `di`, `gps`, `diagnostics`, or `status`.

All traffic is QoS 1. The device never sets the MQTT retain flag. The ingestion pipeline stores each payload verbatim, so the schemas below are also the stored `raw_data` schemas.

MQTT connection parameters (what a broker admin needs):

- **Client ID:** `iotronics-<imei>` — for example `iotronics-860710081332507`. It is stable per device, so the broker can key ACLs and per-device rules on it.
- **Session:** persistent (not a clean session). The broker must queue QoS-1 messages for an offline device and deliver them on reconnect; this is what lets a `setconfig` or FOTA push reach a device that was briefly offline. Broker-side session or message expiry that is too short will silently drop commands sent during an outage.
- **Keepalive:** 60 seconds. A device that has genuinely dropped will miss its keepalive within ~1.5× that; do not declare a device offline on keepalive alone, though — use the silence-based rule in section 7, which also tolerates reboots and FOTA.
- **TLS:** on by default (port and CA details in section 8); can be disabled per device via `mqttTls` for a plaintext broker.
- **Payload ceiling:** the device never publishes a message larger than ~1.4 KB (the modem's transmit limit). Every payload either fits or is replaced by its `404`/`error` fallback (section 3), so the broker will never receive a truncated message.

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

Status, diagnostics, and GPS messages use the bare IMEI (no suffix), because they describe the gateway itself rather than a unit behind it. No equipment ever uses the bare IMEI, so device-level messages never overlap with unit telemetry.

## 3. Conventions used across payloads

Dispensers, radars, and the genset are transparent bridges: the device forwards raw protocol bytes as a comma-separated hex string and the backend decodes them. Decoding the dispenser and tank frames needs the vendor references (the BlueSky dispenser command set and the Ztech/Pantech tank Modbus maps), which are separate from this guide. Dispenser and radar values are the complete protocol frame; the genset value is the Modbus register payload with framing and CRC already stripped, and its full register maps are in Appendix A. Digital inputs and GPS arrive as ready-to-use JSON numbers.

The `404` marker. When a read times out or fails validation, the device publishes `404` for that item instead of dropping the message. This distinguishes "the gateway is alive but this unit did not answer" from "the gateway is offline." It is the string `"404"` for dispenser, radar, and genset, and the numeric `404` for digital inputs.

The `TS` field. Every telemetry payload carries a `TS` string formatted `YYYY-MM-DD HH:MM:SS`, taken from the cellular network clock with the timezone offset stripped (it is effectively UTC, and in production has run 5 hours 30 minutes behind the pipeline's IST `report_time`). Do not use `TS` for ordering or analytics; use the pipeline's `report_time`. `TS` is only a rough device-side reference.

Truncation fallback. Every payload must fit the modem's transmit ceiling of roughly 1.4 KB. If a message would exceed it, the device sends a minimal shape instead of a cut-off one. Object payloads fall back to their `"404"` shape — for the genset that is the same key with the value `"404"` (for example `{"genset":{"registers":"404"}}`); diagnostics and the config read-back keep their wrapper key and add an `"error"` field. Treat any `error` field as "this cycle's data did not fit; the device is otherwise healthy."

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

Every 5 minutes when a genset is configured, the device issues up to **two Modbus reads** and publishes **each as its own message** — a measurement block under `registers` and a DG status block under `status`. Both carry the same genset HW_ID and the same `TS`, and arrive back to back.

```json
{"HW_ID": "86071008133250710",
 "genset": {"registers": "01,3f,00,3e,00,22,00,89, ... ,1b,08"},
 "TS": "2026-08-05 18:29:41"}

{"HW_ID": "86071008133250710",
 "genset": {"status": "c0,00,00,00"},
 "TS": "2026-08-05 18:29:41"}
```

The value is a comma-separated lowercase hex string of the **register data bytes only** — the same transparent-bridge shape radar uses. The Modbus framing (slave address, function code, byte count) and the trailing CRC are stripped by the device, which has already validated them, so what arrives is purely the register payload. Extract register k (0-based) as:

```
register[k] = data[2*k] << 8 | data[2*k + 1]
```

The reads per model, and the Appendix A table that decodes them:

| `gensetModel` | Controller | FC | `registers` read | `status` read | Appendix |
|---|---|---|---|---|---|
| 0 | Mahindra GC1211 | 04 | offsets 1–36 (72 B) | offsets 67–68 (4 B) | A.1 |
| 1 | Mahindra GC1111 | 03 | offsets 1–36 (72 B) | offsets 67–68 (4 B) | A.1 |
| 2 | SGC 120 Mk II | 03 | offsets 1–78 (156 B) | *(none)* | A.2 |
| 3 | Ashok Leyland GC1201 | 04 | offsets 1–36 (72 B) | offset 67 (2 B) | A.1 |
| 4 | Ashok Leyland GC1202 | 04 | offsets 1–36 (72 B) | offset 67 (2 B) | A.1 |
| 5 | Ashok Leyland GC1203 | 04 | offsets 1–36 (72 B) | offset 67 (2 B) | A.1 |
| 6 | Sedemac GC902 | 04 | offsets 1–36 (72 B) | offset 67 (2 B) | A.1 |

Notes:

- **GC1211 and GC1111 differ only in the function code** (04 vs 03); the register blocks are identical.
- **The SGC 120 sends only the `registers` message** — its single overall read already covers the status registers, so no second message is published for it. Do not wait for one.
- **The Mahindra status read returns two registers** (DG status word, then command status); the Ashok Leyland status read returns one (DG status word only).
- **Alarms are not published.** The controllers expose a separate alarm command, which the device deliberately does not issue — it keeps the genset off the shared RS485 bus for one fewer transaction per poll.
- If a read fails every retry, that message's value is the string `"404"` (for example `{"genset": {"status": "404"}}`). The key is always present, so routing never changes on failure.

#### 4.3.1 Worked examples per model

Real payloads, published to `<base>/data`, produced from the reference controller captures. The Modbus command the device sends is shown above each message (slave `0A` = `gensetSlave` 10).

**`gensetModel` 0 — Mahindra GC1211** (two messages)

```
TX  0A 04 00 01 00 24 A0 AA          (FC04, offsets 1–36)
```
```json
{"HW_ID": "86071008665145510", "genset": {"registers": "01,3f,00,3e,00,22,00,89,00,89,05,48,07,39,05,dc,01,f3,09,69,09,5c,09,52,10,51,10,3d,10,48,00,00,00,00,00,04,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,04,00,01,00,00,1b,08"}, "TS": "2026-08-05 18:29:41"}
```
```
TX  0A 04 00 43 00 02 81 64          (FC04, offsets 67–68)
```
```json
{"HW_ID": "86071008665145510", "genset": {"status": "c0,00,00,00"}, "TS": "2026-08-05 18:29:41"}
```

**`gensetModel` 1 — Mahindra GC1111** (two messages; identical blocks to the GC1211, function code 03)

```
TX  0A 03 00 01 00 24 15 6A          (FC03, offsets 1–36)
TX  0A 03 00 43 00 02 34 A4          (FC03, offsets 67–68)
```
```json
{"HW_ID": "86071008665145510", "genset": {"registers": "01,3f,00,3e,00,22,00,89,00,89,05,48,07,39,05,dc,01,f3,09,69,09,5c,09,52,10,51,10,3d,10,48,00,00,00,00,00,04,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,04,00,01,00,00,1b,08"}, "TS": "2026-08-05 18:29:41"}

{"HW_ID": "86071008665145510", "genset": {"status": "c0,00,00,00"}, "TS": "2026-08-05 18:29:41"}
```

**`gensetModel` 3, 4, 5 — Ashok Leyland GC1201/1202/1203, and 6 — Sedemac GC902** (two messages; the status read returns a single register, so its value is 2 bytes rather than 4)

```
TX  0A 04 00 01 00 24 A0 AA          (FC04, offsets 1–36)
TX  0A 04 00 43 00 01 C1 65          (FC04, offset 67)
```
```json
{"HW_ID": "86071008665145510", "genset": {"registers": "01,e0,00,59,00,28,00,89,00,8b,14,fb,0e,80,05,dd,01,f3,09,68,09,65,09,67,10,43,10,49,10,4e,01,31,01,20,01,1d,00,3c,00,37,00,34,00,a7,00,49,00,45,00,44,00,d3,00,29,00,29,00,2c,00,80,00,52,00,4f,00,4b,00,4f,00,09,6d,3a"}, "TS": "2026-08-05 18:29:41"}

{"HW_ID": "86071008665145510", "genset": {"status": "40,00"}, "TS": "2026-08-05 18:29:41"}
```

**`gensetModel` 2 — SGC 120 Mk II** (one message only — no `status` message is ever published for this model)

```
TX  0A 03 00 01 00 4E 95 45          (FC03, offsets 1–78)
```
```json
{"HW_ID": "86071008665145510", "genset": {"registers": "09,61,09,59,08,ee,10,34,0f,d8,0f,db,01,f8,01,f8,01,f8,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,06,00,00,00,0a,00,00,00,08,00,00,00,00,00,00,00,00,00,00,00,00,00,31,01,90,01,18,00,1c,00,92,00,91,05,e7,00,2d,00,22,00,00,00,01,00,03,00,00,00,00,11,00,11,11,00,11,11,01,01,00,11,00,00,00,01,10,00,00,11,11,11,00,00,f1,00,01,23,10"}, "TS": "2026-08-05 18:29:41"}
```

Message sizes: the 36-register models produce a ~303-byte `registers` message and a ~90–96-byte `status` message; the SGC 120 produces a single ~555-byte message. All are well inside the ~1.4 KB ceiling.

### 4.4 Digital inputs

Digital inputs are event-driven, not periodic. The device samples the inputs about every 2 seconds and publishes only when a value changes. A steady input produces no traffic, so do not treat silence as a fault or assume a fixed cadence.

All four channels enabled (`diEnable` = 15):

```json
{"HW_ID": "86071008133250740", "di": {"DI1": 0, "DI2": 1, "DI3": 0, "DI4": 1}, "TS": "2026-08-08 05:27:59"}
```

Only the channels enabled in `diEnable` appear, so a station with just DI1 and DI2 enabled (`diEnable` = 3) publishes `{"DI1": 0, "DI2": 1}` with no `DI3`/`DI4` keys at all. `diEnable` is a bitmask: bit 0 = DI1, bit 1 = DI2, bit 2 = DI3, bit 3 = DI4.

Values are `0` or `1`, or the numeric `404` if a channel could not be read — DI is the one place where `404` is a number rather than the string `"404"`. Because a transition is queued and flushed on the next cycle, the `TS` on a DI message is the flush time, which can be up to a cycle after the transition.

### 4.5 GPS

GPS ships disabled and is off on every deployed gateway (`gpsEnable` defaults to false), so do not assume GPS data is present. When enabled, one message publishes roughly every 20 seconds using the bare IMEI:

```json
{"HW_ID": "860710081332507", "gps": {"lat": 12.887630, "lon": 77.641710}, "TS": "2026-08-08 05:27:59"}
```

`lat` and `lon` are decimal degrees, positive north and east. No fix yet or a failed query gives `{"gps": {"data": "404"}}`. No accuracy or confidence figure is sent.

### 4.6 Diagnostics

One health report every ~15 minutes, with the bare IMEI. This is a single combined message — the former separate `diagnostics` and `sysmon` reports were merged into it. It carries the fleet-dashboard view plus the device internals needed to investigate a specific gateway.

```json
{"HW_ID": "860710081332507",
 "diagnostics": {"fw": "1.0.0", "uptime_s": 5231,
                 "reset_reason": 1, "boot_count": 7,
                 "temp_c": 46.5, "heap_free": 274160, "heap_min_ever": 251200,
                 "main_stack_hwm": 2400, "di_stack_hwm": 3800,
                 "rssi": 26, "vbat_mv": 3820, "operator": "Airtel",
                 "pub_ok": 412, "pub_fail": 3, "loop_max_ms": 1840,
                 "du_ok": true, "radar_ok": true, "genset_ok": false,
                 "di_ok": true, "gps_ok": false},
 "TS": "2026-08-08 05:27:59"}
```

- `rssi`: raw modem CSQ (0–31, or 99 for unknown), not dBm. Convert with dBm = -113 + 2 × rssi. `temp_c` is the ESP32 core temperature in °C. The device no longer sends a `thermal_zone` label — derive it if needed: normal below 65 °C, warm 65–75, hot 75–80, critical at or above 80.
- `heap_free` is the current free heap; `heap_min_ever` is the lowest free heap since boot — a `heap_min_ever` that falls steadily across reports is a memory leak. `main_stack_hwm` and `di_stack_hwm` are the minimum free stack (in bytes) ever seen on the main and DI tasks; a value approaching 0 warns of a stack overflow (`di_stack_hwm` is 0 when DI is disabled).
- `pub_ok` and `pub_fail`: publish counters for the reporting window, reset after a confirmed diagnostics publish. `loop_max_ms` is the worst poll-cycle duration in the window.
- The `_ok` flags reflect the most recent poll of each subsystem, as a logical AND across all units of that type. A subsystem that is not configured keeps its default and means nothing. `gps_ok` is false on virtually every gateway because GPS is off; do not alert on it alone.
- `reset_reason`: ESP-IDF reset-reason enum. 0 unknown, 1 power-on, 2 external pin, 3 software restart, 4 crash/panic, 5 interrupt watchdog, 6 task watchdog, 7 other watchdog, 8 deep-sleep wake, 9 brownout. Values 10 and above (10 SDIO, 11 USB, 12 JTAG, 13 eFuse error, 14 power glitch, 15 CPU lockup/double-exception) are hardware-debug edge cases that should not occur on a deployed unit; treat any of them as worth investigating if seen. `2` (external pin) is also unexpected in the field — nothing in normal operation asserts an external reset, so it would point to a hardware-level event (e.g. someone pressing EN) rather than firmware behavior. Recurring 4, 6, or 9 is a real fault (crash loop, hang, power instability). **`3` is not exclusively "user requested a restart"** — it is whatever `esp_restart()` was called for, which also covers the low-heap guard and other internal safety reboots (see the note in section 5); do not assume every `3` was expected. `boot_count` is persistent; a rising count with low `uptime_s` is a reboot loop and should alert.

Removed from the older two-message format: the separate `sysmon` message, and the fields `thermal_zone`, `cpu_freq_mhz` (CPU is pinned at 160 MHz), `heap_largest_block`, and `ota_partition` (the running firmware version is in `fw` here and in the `fota_completed` status).

## 5. Status and lifecycle events

These publish on `<base>/data` with the bare IMEI. They are one-shot events.

Boot. `{"HW_ID":"...","status":"ready","fw":"1.0.0"}` means booted and online with at least one unit configured. `{"HW_ID":"...","status":"unconfigured","fw":"1.0.0"}` means booted and online but nothing is configured yet, which is the cue to onboard the gateway.

Configuration. Every well-formed `setconfig` produces an ordered sequence:

1. `{"status":"config_received"}` the instant the payload arrives, before validation.
2. Then one of: `{"status":"config_successful","fw":"1.0.0"}` (validated and saved; the device reboots immediately after) or `{"status":"config_failed","reason":"<reason>"}` (rejected, no reboot; nothing changed except in the `nvs_save_failed` case).

Do not treat a change as committed until `config_successful`. For connectivity changes, `config_successful` means the change was accepted for trial, not that it works (section 6.1).

**Exception — a no-op push does not reboot.** If a `setconfig` payload resolves to a config byte-for-byte identical to what is already live (most likely because a retained message was redelivered on reconnect), the device replies `config_successful` immediately but does **not** save to flash or reboot. This exists specifically so a retained/duplicate `setconfig` cannot loop the device. It means `config_successful` alone does not guarantee a reboot happened; if you need to confirm a reboot occurred, wait for the follow-on `ready`/`unconfigured` as well.

Config failure reasons: `bad_payload`, `radar_mixed_style_use_radarType+tankCount_OR_ztechCount/pantechCount`, `radar_incomplete_send_radarType_and_tankCount_together`, `radar_type_invalid_use_ztech_pantech_or_none`, `gensetSlave <n> collides with radar slave range` (match on the prefix), `ztechCount <n> and pantechCount <n> produce overlapping radar slave ranges` (match on the suffix), `empty_apn`, `bad_mqtt_broker_fields`, `invalid_chars_in_connectivity_fields`, the suffix errors (`<Label> suffix must be exactly 2 alphanumeric characters`, `more than one equipment has no suffix assigned (bare IMEI)`, `<A> and <B> have duplicate suffix <sfx>`), and `nvs_save_failed`.

Remote reboot and reset. `{"status":"restarting"}` is sent just before a `restart` reboots, and `{"status":"factory_resetting"}` just before a `factory_reset` wipes and reboots.

FOTA. The sequence is `fota_initiated` (last message before the device drops offline to download), then `fota_successful` (flashed and reconnecting to report, not yet confirmed good), then `fota_completed` with `fw` (the new image survived a full poll cycle, which is the real success signal). Failure is `fota_failed` with a `reason`, or `fota_skip_known_bad` when a URL identical to one that already failed this session is refused. The most common reason is `http_<code>_err_<err>`: code 404 means a wrong URL, 403 means forbidden, and -1 means the request never reached the server. Other reasons include `non_https_url`, `get_timeout`, `bad_content_length`, `readfile_timeout`, `image_verify_failed`, and `network_rebuild_failed`.

Connectivity rollback. `{"status":"connectivity_rollback","reason":"new_settings_unreachable"}` is sent once, after the device reconnects on reverted settings, when a pushed APN or broker change could not reach a broker on the next boot (section 6.1).

**Not every reboot self-announces.** `restarting`/`factory_resetting`/`config_successful`/the FOTA sequence all precede a reboot because MQTT is still up at that point. Several internal safety reboots have no MQTT session to publish on when they fire, so they are completely silent — the device simply drops off and comes back with `ready`/`unconfigured`: the low-heap guard (free heap below ~20 KB), exhausting the connectivity recovery ladder, and any boot-time failure to power on the modem, bring up the network, or reach the broker. Do not treat a `ready` with no preceding status as an anomaly; use `reset_reason` and `boot_count` in the next `diagnostics` report (section 4.6) to tell a silent safety reboot apart from a normal power cycle. Note that `reset_reason` cannot distinguish a user-requested `restart` from one of these internal safety reboots — both report `3` (software restart), since both go through the same `esp_restart()` call; only a `boot_count` that rises with a low `uptime_s`, or a `heap_min_ever` that was trending down beforehand, points to the difference.

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

**This is also the mechanism for fleet maintenance reboots.** The device has no on-device scheduled reboot — that decision, and its timing, belongs entirely to the cloud, which pushes `{"cmd":"restart"}` to each device's own `<base>/<imei>/setconfig` topic whenever it decides that device should reboot. See section 7 for the batching approach.

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

- Maintenance reboot is cloud-scheduled, not automatic. Earlier firmware rebooted every device on a fixed on-device timer (01:30 IST); that has been removed. There is now no reboot the device initiates on its own for maintenance — if the fleet is to be rebooted periodically, the cloud must schedule and push it via `{"cmd":"restart"}` (section 6.2).

  The reason for removing the on-device timer was fleet-wide synchronisation: every gateway re-attaching to the cellular network and the MQTT broker at the same instant becomes a real load spike as the fleet grows, for a mechanism this firmware doesn't structurally need (there is no heap allocation on the device, so fragmentation cannot occur, and a device reboot does not power-cycle the modem, so it cannot clear a wedged SIM/PDP either — see the low-heap guard and the connectivity recovery ladder for the mechanisms that actually handle those failure modes).

  If a periodic fleet reboot is still wanted as unknown-unknowns insurance, implement it cloud-side as a scheduler that divides the fleet into batches (for example 10% of devices per time slot) and pushes `{"cmd":"restart"}` to each batch at its assigned time, ideally during a low-traffic window for that site (late night local time). Each reboot produces a silent gap of roughly 60–90 seconds while the device re-attaches and reconnects — the same downtime the old on-device timer produced, since it is the identical boot path.
- Deriving offline. There is no explicit offline message; infer it from silence, allowing for a cloud-scheduled reboot (if configured), FOTA downloads, and connectivity recovery. A reasonable starting threshold is 10 minutes of total silence, tuned against real fleet behaviour.
- Message correlation. Status and config acknowledgements carry no request ID, so serialize pushes to a given device and wait for a terminal response before sending the next. QoS 1 can redeliver a message; every status describes a state rather than a delta, so reprocessing a duplicate is safe.
- Offline queueing. The device uses a persistent MQTT session, so the broker queues commands while it is offline and delivers them on reconnect. Broker-side session or message expiry can drop a command sent during a long outage.
- Suffix changes. A unit's HW_ID changes when its suffix changes; historical data stays under the old HW_ID. Track the rename on the backend if continuity matters.

## 8. Security posture

- Transport is TLS with certificate-chain validation on both the broker and the FOTA server. The trusted-root bundle carries ISRG Root X1 (Let's Encrypt) and Amazon Root CA 1 (S3); an endpoint on any other CA fails closed until its root is added to the firmware. The chain is validated but the hostname is not, so the production certificate must match the hostname the device connects to.
- All gateways currently authenticate to the broker with a shared username. Move to per-device or per-batch credentials before fleet size makes rotation impractical.
- Secure boot and flash encryption are not yet enabled. Flash can be read or rewritten with physical access. Both are supported by the chip but are a one-way, provisioning-time decision (they burn eFuses and cannot be applied over the air). Enable Secure Boot V2 before turning on remote FOTA in the field.
- Access control is at the broker. `setconfig` has no authentication beyond the MQTT connection, so use broker ACLs to restrict which clients may publish to a device's topics.

## Appendix A — Genset register maps and decoding

Each genset field is a raw 16-bit Modbus register (0–65535) extracted from the hex data bytes as in section 4.3 — `register[k] = data[2k] << 8 | data[2k+1]` — with no scaling applied on the device.

Register indices below are **0-based positions within the message**, and every read starts at Modbus offset 1, so `register[k]` is protocol offset `k + 1`.

- Models 0, 1, 3, 4, 5, 6 (GC1211, GC1111, GC1201/02/03, GC902) use table A.1.
- Model 2 (SGC 120 Mk II) uses table A.2.

Decoding rules:

- **Measurement:** `real value = register[k] × scale`, using the scale in the table.
- **Signed fields** (marked "signed"): if `register[k]` is greater than 32767, subtract 65536 before applying the scale (16-bit two's complement). Unmarked fields are unsigned.
- **32-bit fields** (cumulative energy, SGC 120 run time): combine the two listed registers as `(register[lower] << 16) | register[higher]` — the lower index holds the high word. This follows the standard Modbus convention but has not been confirmed against a live controller; verify against a known reading before trusting the absolute value.
- **Status words:** read individual bits with `(register >> bit) & 1`, using the 0-indexed bit positions listed under each table.

### A.1 Mahindra GC1211 / GC1111, Ashok Leyland GC1201/1202/1203, Sedemac GC902

**`registers` message** — 36 registers, `register[0]` … `register[35]` (Modbus offsets 1–36):

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

**`status` message** — the DG status word, and on the Mahindra models a command-status register:

| Model | `status` contents |
|---|---|
| GC1211, GC1111 | 2 registers: `register[0]` DG status word (offset 67), `register[1]` command status (offset 68) |
| GC1201/02/03, GC902 | 1 register: `register[0]` DG status word (offset 67) |

DG status word bits: 15 available / loadOnDG, 14 running, 13 stoppedNormally, 12 stoppedWithFault, 11 mode (1 = Auto, 0 = Manual). For example `0xC000` is available and running; `0x4000` is running; `0x2000` is stoppedNormally.

Command status (Mahindra only): 1 Started, 2 Stopped, 3 AlreadyON, 4 AlreadyOFF, 5 Manual, 6 NotStarted; 0 or anything else Unknown.

Worked example — a GC1211 `registers` message beginning `01,3f,00,3e,00,22,...` and a `status` message of `c0,00,00,00`:

- `register[0]` = 0x013F = 319 → 319 × 0.01 = **3.19 Bar** oil pressure
- `register[1]` = 0x003E = 62 → **62 °C** engine temperature
- `register[2]` = 0x0022 = 34 → **34 %** fuel level
- `status register[0]` = 0xC000 → bit 15 and bit 14 set = **available and running**; `status register[1]` = 0x0000 → command status **Unknown/none**

### A.2 SGC 120 Mk II

**`registers` message** — 78 registers, `register[0]` … `register[77]` (Modbus offsets 1–78). Sparse: indices not listed are reserved and read zero. There is no separate `status` message for this model.

```
0-2    Gen L1/L2/L3-N voltage (0.1 V)        50     Coolant temperature (0.1 C, signed)
6      Generator frequency (0.1 Hz)          51     Fuel level (divide by 10, inferred)
22-24  Gen L1/L2/L3 current (0.1 A)          53     Charging alternator voltage (0.1 V)
25-28  Gen L1/L2/L3/total power (0.1 kW, signed)  54  Battery voltage (0.1 V)
49     Lube oil pressure (0.1 Bar, unverified)   55  Engine speed (RPM)
                                             56     Number of starts
                                             58-59  Engine run time, 32-bit (hours)
```

Worked example — an SGC 120 `registers` message beginning `09,61,09,59,08,ee,...`:

- `register[0]` = 0x0961 = 2401 → 2401 × 0.1 = **240.1 V** Gen L1-N
- `register[1]` = 0x0959 = 2393 → **239.3 V** Gen L2-N
- `register[6]` = 0x01F8 = 504 → 504 × 0.1 = **50.4 Hz** generator frequency

The SGC 120 field positions are transcribed from DEIF documentation and only partly confirmed against a live controller. Sanity-check the first real reading of each field before relying on it.
