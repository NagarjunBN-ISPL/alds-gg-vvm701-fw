# VVM701 Gateway — Cloud Integration Reference

Firmware 1.0.0 · Last updated 20 July 2026

## 0. What changed

If you worked from an earlier copy of this document, read this first. Everything here is already live in firmware 1.0.0 — the sections listed have the detail.

**The outbound topic is now flat: `<base>/data`.** It used to be `<base>/<imei>/data`, with separate `<base>/diagnostics` and `<base>/sysmon` topics alongside it. All three are gone; every device→cloud message from every gateway now lands on one shared, fleet-wide `<base>/data`. You identify the sender from the `HW_ID` field in the payload and the message kind from which top-level key is present — never from the topic. Inbound command topics are unchanged and still per-IMEI. Sections 6 and 8.

**Diagnostics and sysmon payloads are nested.** Their fields now sit inside a `"diagnostics"` / `"sysmon"` object rather than at the top level. If you briefly saw a flat `"type":"diagnostics"` field, that shape is dead — it existed for part of one day and never shipped. Section 8.5.

**Genset field names are short mnemonics.** `PREV`, `OPR`, `CT`, `FLVL`, `CAV`, `BV`, `NOS`, `RHR`, `ENGSP`, `HZ`, `L1N`/`L2N`/`L3N`, `I1`/`I2`/`I3`, `KW1`/`KW2`/`KW3`/`KW`. Anything using the old descriptive names (`lubeOilPressure`, `engineTemp`) is reading a dead schema. `KWH` and `ALR1`–`ALR3` are new. Section 8.3.

**New — GPS.** A `gps` key carrying `lat`/`lon`, plus a `gps_ok` flag in the health reports. **Disabled on every gateway**; do not build anything that assumes it is present. Section 8.6.

**New — DEIF SGC 120 genset support** (`gensetModel: 2`), and IDs 3–6 reserved for Sedemac variants that deliberately report `404` rather than risk wrong data. Dispenser and radar caps were also raised. Sections 5 and 10.1.

**Stricter setconfig validation.** HW_ID suffixes must now be exactly 2 *alphanumeric* characters — previously only the length was checked, so a suffix containing a quote or space would silently corrupt that unit's JSON and topic strings. Connectivity fields containing a quote, backslash, or control character are now rejected up front with a new reason, `invalid_chars_in_connectivity_fields`, instead of burning a reboot and a failed trial. Sections 5, 9.2, 10.1.

No other topic, payload, or setconfig key changed.

## 1. Purpose and scope

This document describes the MQTT message contract between the VVM701 field gateways and the backend, and gives the backend, dashboard, and cloud/infrastructure teams what they each need to build against it. It covers every topic the device uses, the exact shape of every payload it publishes, the commands it accepts, the runtime behaviours that are not obvious from a payload sample alone, and practical guidance for the dashboard and for whoever operates the broker.

It deliberately does not cover firmware internals, hardware wiring, the RS485/Modbus decoding of dispenser and radar frames, or the modem's recovery state machine, except where those things change what the cloud actually observes on the wire.

Throughout, `<base>` is the configurable MQTT base topic (default `fuelsense`) and `<imei>` is the gateway's 15-digit modem IMEI.

## 2. System overview

Each fuel station has one cellular gateway built around an ESP32-S3. A single gateway sits in front of several pieces of equipment — fuel dispensers, tank-level radars, a generator set, and discrete digital inputs — and is the only thing that talks to the broker. The equipment behind it does not have its own network identity; the gateway polls each unit and republishes the readings.

The device holds one MQTT session and publishes everything through it. The ingestion pipeline already stores each message as `{id, device_id, creation_time, report_time, raw_data}`. The important point for this document: `raw_data` is the payload exactly as the device sent it. Nothing in the path adds, renames, or reformats fields. Every schema below is therefore also the schema of the `raw_data` you already store.

All publishing and subscribing is at QoS 1. The device never sets the MQTT retain flag on anything it publishes; any retained message you see on a device-facing topic was retained by whoever published it, not by the device (this matters for FOTA — see section 10.3).

## 3. Quick start — validating your integration

Before working through the full reference below, this is the fastest way to confirm a pipeline is wired correctly end to end. It assumes a gateway is already provisioned with a known IMEI and broker access.

1. Subscribe to `<base>/data` — a single fleet-wide topic that carries all device→cloud messages (telemetry, status, and the diagnostics/sysmon health reports).
2. Trigger a reboot — publish `{"cmd":"restart"}` to `<base>/<imei>/setconfig`, or power-cycle the gateway.
3. Within about 20 seconds, expect a boot status message on `<base>/data`: either `ready` or `unconfigured`, both carrying `fw`. If nothing arrives within a minute, the gateway did not reconnect — check signal and APN before looking at the pipeline itself.
4. Publish `{"cmd":"getconfig"}` to `<base>/<imei>/setconfig`. Expect a `config_report` back on `<base>/data` within a couple of seconds. This confirms both the subscribe path and the device's inbound command path.
5. Push a small, low-risk config change — enabling a digital input is a safe first test since it carries no connectivity risk: `{"diEnable":1,"sfxDI":"QS"}`. Expect, in order: `config_received` immediately, `config_successful` within about a second, then silence for 15–20 seconds while the device reboots, then a fresh `ready`.
6. Confirm telemetry now arrives on `<base>/data` with `"HW_ID":"<imei>QS"` and a `di` object.

If all six steps produce the expected sequence, the subscribe path, the publish path, and the config lifecycle are all correctly wired. Everything else in this document is reference detail on top of that same message flow.

## 4. Conventions used across payloads

A few patterns repeat across most message types. They are described once here and referenced later.

**Raw hex frames.** For dispensers and radars, the gateway is a transparent bridge. It does not decode meter or tank-level values; it forwards the raw protocol frame as a comma-separated hex string, for example `f5,01,a2,c5,00,7b`. Decoding those bytes into litres, volumes, or prices is the backend's responsibility. Genset and digital-input data, by contrast, are decoded on the device and arrive as normal JSON numbers.

**The `404` marker.** When a sensor read times out or fails validation, the device substitutes the value `404` rather than dropping the message. This lets you distinguish "the gateway is alive and tried, but this unit didn't answer" from "the gateway is offline." `404` can appear as a whole-object value or as a single field, depending on the message type; each section below says which. Note one inconsistency worth handling in your parser: dispenser, radar and genset use the **string** `"404"`, while digital inputs use the **numeric** `404` (unquoted).

**Truncation fallback.** Every payload has to fit the modem's safe transmit size (roughly 1.4 KB). If a message would exceed its buffer, the device sends a minimal fallback instead of a malformed or silently-cut payload. For diagnostics and sysmon this is `{"HW_ID":...,"diagnostics":{"fw":...,"error":"payload_truncated"},"TS":...}` (or `"sysmon"` for the sysmon report) — the wrapping key is kept even in the fallback so the message kind is still identifiable; for genset it falls back to the `404` shape (section 8.3); for the config read-back it is `{"...","status":"config_report","error":"truncated"}`. Treat any `error` field as "this cycle's data did not fit, the device is otherwise healthy."

**The `TS` field.** Every telemetry payload carries a `TS` string formatted `YYYY-MM-DD HH:MM:SS`. It comes from the cellular network clock (`AT+QLTS`), and the firmware strips the timezone-offset portion while parsing, so `TS` carries no zone information. In observed production data `TS` ran exactly 5 hours 30 minutes behind the pipeline's `report_time`, i.e. the device was reporting UTC while the pipeline recorded IST. That offset is not guaranteed — it depends on what the operator's network delivers and can differ across SIMs and attach states. Do not use `TS` as an authoritative timestamp for analytics or ordering. Use the pipeline's `report_time` / `creation_time`. `TS` is useful only as a rough device-side sanity check.

## 5. Identity and addressing

The gateway's root identity is its **IMEI**. This is what appears as `device_id` in the pipeline and in your sentry queries.

Because one gateway fronts several pieces of equipment, individual equipment telemetry is tagged with a **HW_ID**, constructed as:

```
HW_ID = IMEI + <2-character suffix>
```

The suffix identifies which physical unit a reading belongs to. Suffixes are set remotely through `setconfig` (section 10.1) and must be exactly 2 alphanumeric characters (letters/digits only — anything else is rejected, since the suffix is embedded verbatim into JSON payloads and MQTT topic strings). If a unit has no custom suffix configured, the device falls back to a generated one:

| Equipment | Custom suffix key | Auto fallback | Example HW_ID |
|---|---|---|---|
| BlueSky dispenser 1–6 | `sfxBs1` … `sfxBs6` | `B1` … `B6` | `860710081332507B1` |
| Ztech radar 1–4 | `sfxZtech1` … `sfxZtech4` | `Z1` … `Z4` | `860710081332507Z1` |
| Pantech radar 1–4 | `sfxPantech1` … `sfxPantech4` | `P1` … `P4` | `860710081332507P1` |
| Genset | `sfxGenset` | none | `860710081332507GS` (custom) or bare IMEI |
| Digital inputs | `sfxDI` | none | `860710081332507DI` (custom) or bare IMEI |

These are upper limits, not what every station reports — a gateway only ever publishes telemetry for the equipment it is actually configured with (`duBlueskyCount`/`ztechCount`/`pantechCount`, section 10.1).

Two things to note. First, genset and DI have no auto-fallback — if you leave their suffix unset, their HW_ID is the bare IMEI with nothing appended. Second, device-level status, diagnostics, sysmon, and GPS (section 8.6) messages always use the bare IMEI as their `HW_ID`, never a suffixed one — there is no suffix key for any of these four; only equipment telemetry (dispenser, radar, genset, DI) carries a suffixed HW_ID.

This is the same value your dashboard already keys on. Querying the sentry endpoint with a suffixed `device_id` (for example `...GS` or `...DI`) returns the readings for that one piece of equipment on that gateway; that is exactly how the HW_ID is meant to be used downstream.

**Uniqueness is enforced on the device.** No two active units may resolve to the same HW_ID. The firmware rejects any `setconfig` that would cause a collision, including the case where two units are both left without a suffix and would both fall back to the bare IMEI. The reason: two units publishing under one HW_ID are indistinguishable in the pipeline, and one would silently overwrite the other on the dashboard. Validate suffix uniqueness in the dashboard before sending a config too, so the operator gets an immediate error instead of a rejected round-trip.

## 6. Topic map

| Topic | Direction | Cadence | Contents |
|---|---|---|---|
| `<base>/data` | Device → Cloud | 30 s and on-event | **Everything** device→cloud: equipment telemetry, status/lifecycle events, and the diagnostics + sysmon health reports (each self-identified — see the note below). Not IMEI-scoped. |
| `<base>/<imei>/setconfig` | Cloud → Device | on demand | Configuration and system commands (JSON) |
| `<base>/<imei>/update` | Cloud → Device | on demand | FOTA trigger for one device (payload: HTTPS URL) |
| `<base>/update` | Cloud → Device | on demand | FOTA trigger broadcast to the whole fleet |
| `<base>/rsp/imei/<hw_id>/in` | Cloud → Device | on demand | Dispenser RSP command (payload: hex) |
| `<base>/rsp/imei/<hw_id>/out` | Device → Cloud | ~1 s | Dispenser RSP response (hex, or `404`) |
| `<base>/kfactor/imei/<hw_id>/in` | Cloud → Device | on demand | Dispenser K-factor command (payload: hex) |
| `<base>/kfactor/imei/<hw_id>/out` | Device → Cloud | ~1 s | Dispenser K-factor response (hex, or `404`) |

**There is a single outbound topic, `<base>/data`, shared across the whole fleet — not IMEI-scoped.** Every device publishes everything it sends to the cloud onto this one topic (e.g. every gateway's telemetry, health, and status all land on one `<base>/data`). Two questions your consumer answers from the payload, never from the topic:

- **Which device / which equipment?** — the `HW_ID` field. Always read it from the payload; it is the only way to attribute a message. Equipment telemetry carries a suffixed HW_ID (IMEI + 2-char suffix); status, diagnostics and sysmon carry the bare IMEI.
- **What kind of message?** — which top-level key is present in the payload: `nozzles` → dispenser, `tanks` → radar, `genset` → genset, `di` → digital inputs, `status` → a lifecycle/status event, `diagnostics` → the operational health report, `sysmon` → the low-level system-health report. Check for these to route each message; a payload has exactly one of them.

The inbound command topics (`setconfig`, per-device `update`, rsp/kfactor `in`) remain per-IMEI so a command targets exactly one device. The rsp/kfactor `out` responses also stay on their own per-HW_ID topics: they are bare hex strings rather than JSON, so they have nowhere to carry an `HW_ID` and must be matched by their topic instead. Those topics embed the per-unit HW_ID (IMEI + suffix), so they change if a dispenser's suffix is reconfigured.

## 7. Onboarding a new gateway

The typical sequence to bring a station online, from the cloud side:

1. The gateway powers on, attaches to the network, connects to the broker, and publishes a boot message on `<base>/data` with its bare IMEI. A factory-fresh unit with nothing configured publishes `status: "unconfigured"`; a unit that already has equipment configured publishes `status: "ready"`. Either way, this first message is how you learn the gateway's IMEI and that it is online.
2. Send a `setconfig` to `<base>/<imei>/setconfig` with the equipment counts and the suffixes you want (section 10.1). This is also where you assign the human-facing HW_ID suffixes that the dashboard will display.
3. The device replies `config_received` immediately, then `config_successful` once the config validates and persists (or `config_failed` with a reason if it does not — section 9.2).
4. After `config_successful` the device reboots. Allow roughly 15–20 seconds. It comes back up, publishes `ready`, and begins publishing equipment telemetry under the HW_IDs you just assigned (IMEI + each suffix).
5. From that point the dashboard should expect telemetry from one HW_ID per configured unit. If you need to confirm what the device actually stored, send `{"cmd":"getconfig"}` and read the `config_report` it publishes back (section 10.2).

## 8. Telemetry payloads (device → cloud)

All equipment telemetry publishes to `<base>/data`. Each message carries the HW_ID of the specific unit it describes.

### 8.1 Dispenser (BlueSky)

One message per dispenser unit per poll cycle (every 30 s). Each dispenser occupies two nozzle positions.

```json
{
  "HW_ID": "860710081332507B1",
  "nozzles": {
    "1": {"data": "f5,01,...", "rsp": "f5,01,...", "kfact": "f5,01,...", "rfid": "f5,01,..."},
    "2": {"data": "f5,02,...", "rsp": "f5,02,...", "kfact": "f5,02,...", "rfid": "f5,02,..."}
  },
  "TS": "2026-07-10 05:27:59"
}
```

The keys inside `nozzles` are **global 1-based nozzle numbers as strings**, not per-unit: dispenser 1 publishes nozzles `"1"` and `"2"`, dispenser 2 publishes `"3"` and `"4"`, dispenser 3 publishes `"5"` and `"6"`. `data` is the totalizer read, `rsp` and `kfact` are the RSP and K-factor reads, `rfid` is the RFID read — all raw hex.

There is no idle/busy state in the payload. The only presence signal is whether a nozzle returned data. Two failure shapes:

- Nozzle absent (no valid status reply): the whole nozzle collapses to `{"data": "404"}` with no `rsp`/`kfact`/`rfid` keys at all.
- Nozzle present but one read failed: that field alone is the string `"404"` while the others carry real hex.

So `"data": "404"` on its own (no sibling keys) means "nozzle not responding," whereas `"404"` in one field alongside real data in others means "nozzle is there, that one read failed this cycle."

### 8.2 Radar (tank level)

Radars are polled on a slower cadence than the main loop — roughly every 60 seconds, not every 30. One message per tank.

```json
{
  "HW_ID": "860710081332507Z1",
  "tanks": {"1": {"data": "04,03,04,2f,13,00,00,56,22"}},
  "TS": "2026-07-10 05:27:59"
}
```

The `tanks` key is the 1-based tank index for that radar type, as a string. Ztech tanks number from `"1"`, and Pantech tanks number from `"1"` independently (they carry different HW_IDs, so the numbering does not collide). `data` is the raw Modbus RTU response frame in hex; the backend decodes the register bytes into level/volume. On a failed or CRC-invalid read the whole thing is `{"data": "404"}`.

### 8.3 Generator set (genset)

One message per poll cycle when a genset is configured. Unlike dispenser and radar, these values are decoded on the device. Numeric values are delivered pre-scaled. Three controller families currently have a working decode (`gensetModel` in section 10.1 selects which): Sedemac GC1211, Sedemac GC1111, and DEIF SGC 120 Mk II. Four more model IDs (3–6) are reserved for other Sedemac variants in the field whose exact register map isn't verified yet — see the `gensetModel` note in section 10.1, they report `404` rather than data until that's resolved. The firmware normalizes each controller's own register map into the same key names, and the core measurement/`alarms`/`status` block (everything from `PREV` through `KW`, plus `alarms` and `status`) is common to all three working models — but the example below is a maximal one: `KWH`, `ALR1`/`ALR2`/`ALR3`, `commandStatus`, and `holdingRegs` are Sedemac-only and never appear in a DEIF SGC 120 payload (see the conditional-fields note below the table). The alarm-name keys (Appendix A) and the set of possible `mode` values also differ by model, noted where relevant below.

```json
{
  "HW_ID": "860710081332507GS",
  "genset": {
    "PREV": 1,
    "OPR": 4.20,
    "CT": 78,
    "FLVL": 64,
    "CAV": 13.8,
    "BV": 13.4,
    "NOS": 152,
    "RHR": 4830,
    "ENGSP": 1500,
    "HZ": 50.0,
    "L1N": 240.1, "L2N": 239.8, "L3N": 240.5,
    "I1": 12.30, "I2": 11.90, "I3": 12.10,
    "KW1": 2.9, "KW2": 2.8, "KW3": 2.9, "KW": 8.6,
    "KWH": 481200,
    "ALR1": 0, "ALR2": 0, "ALR3": 0,
    "alarms": {"LOP": "Shutdown", "HighEngTemp": "Warning"},
    "status": {
      "available": true, "running": true,
      "stoppedNormally": false, "stoppedWithFault": false,
      "mode": "Auto"
    },
    "commandStatus": "Started",
    "holdingRegs": {"command": 0, "operatingMode": 1}
  },
  "TS": "2026-07-10 05:27:59"
}
```

Every measurement field uses a short mnemonic key rather than a descriptive name — the full key reference is the table immediately below. This is a deliberate, deployed change (as of 2026-07-16); older captures or documentation using names like `lubeOilPressure`/`engineTemp` reflect a prior version of this same schema and are no longer what the device sends.

The register numbers in the last column are the **Sedemac** (GC1211/GC1111) source registers. The DEIF SGC 120 reads a completely different internal register map to produce the same keys — so those numbers are meaningful for a Sedemac Modbus dump only, not a DEIF one. The cloud never sees registers either way; the column is reference for anyone debugging against a raw bus capture.

| Key | Meaning | Sedemac register |
|---|---|---|
| `PREV` | Protocol revision | 0 |
| `OPR` | Lube oil pressure (Bar) | 1 |
| `CT` | Coolant / engine temperature (°C) | 2 |
| `FLVL` | Fuel level (%) | 3 |
| `CAV` | Charging alternator voltage (V) | 4 |
| `BV` | Battery voltage (V) | 5 |
| `NOS` | Number of starts | 6 |
| `RHR` | Engine run time (hours) | 7 |
| `ENGSP` | Engine speed (RPM) | 8 |
| `HZ` | Generator frequency (Hz) | 9 |
| `L1N`, `L2N`, `L3N` | Generator L1/L2/L3-to-neutral voltage (V) | 10, 11, 12 |
| `I1`, `I2`, `I3` | Generator L1/L2/L3 load current (A) | 16, 17, 18 |
| `KW1`, `KW2`, `KW3` | Generator L1/L2/L3 power (kW) | 19, 20, 21 |
| `KW` | Generator total power (kW) | 22 |
| `KWH` | Cumulative generated energy (kWh) — optional, Sedemac only, see caveat below | 35–36 |
| `ALR1`, `ALR2`, `ALR3` | Raw value of the first three alarm registers — optional, Sedemac only, alongside (not instead of) the decoded `alarms` object below | 57, 58, 59 |

Field types: `PREV`, `CT`, `FLVL`, `NOS`, `RHR`, `ENGSP` are integers. `OPR`, the voltages, currents, powers, and `HZ` are floats. `KWH`, `ALR1`, `ALR2`, `ALR3` are integers. `status` booleans are as shown.

**`KWH` is not wire-verified.** It reads registers 35-36 as a single 32-bit value assuming standard high-word-first Modbus register order, but neither that word order nor its scale factor has been confirmed against a live controller or vendor documentation. Treat the raw integer as provisional until confirmed — do not assume a decimal scale (e.g. ÷10 or ÷100) without checking against a real reading. `KWH` and `ALR1`/`ALR2`/`ALR3` are present only on the Sedemac families (GC1211/GC1111); the DEIF SGC 120 driver does not populate them and they're omitted entirely from its payload, same as `commandStatus`/`holdingRegs`.

`status.mode` has more possible values than the example shows. Both Sedemac families report only `Auto` or `Manual`. The DEIF SGC 120 reports one of `Auto`, `Manual`, `Scheduler`, or `Unknown` — its controller has a third operating mode Sedemac doesn't, and `Unknown` is a documented fallback for an unrecognized mode value. Treat `mode` as an open string set, not a fixed two-value enum.

Four parts of this payload are conditional and your schema must treat them as optional:

- **`alarms`** is an object of only the currently active alarms, keyed by alarm name with a severity string as the value. Inactive alarms are absent, not reported as false, so `alarms` can be `{}`. The value is one of `Warning`, `Shutdown`, `ElecTrip`, `Notification` — those are the only four states that ever appear on the wire (two further internal states, `Disabled` and `Inactive`, are filtered out and never published). The full set of possible alarm-name keys is listed in Appendix A.
- **`alarmsTruncated`** appears, set to `true`, only when there were so many simultaneously active alarms that the list was cut short to keep the payload under the modem's transmit ceiling. When present it means "more alarms may be active than are shown here" — surface it rather than treating the alarm list as complete. When not truncated, the key is omitted entirely (it is never `false`).
- **`commandStatus`** appears on both Sedemac families (GC1211 and GC1111) — always present when the poll succeeds. It is absent only on the DEIF SGC 120, which has no equivalent readback register (its command interface is write-only).
- **`holdingRegs`** is narrower: present only on GC1211, and even then only when its separate holding-register read succeeds (some GC1211 variants don't expose that space). Absent on GC1111 (no separate holding space to read) and on the DEIF SGC 120 (same write-only reason as `commandStatus`).

If the genset poll fails outright (controller not answering), the entire `genset` value collapses to `{"data": "404"}` rather than the object above. The same `404` shape is used if a fully-built payload would still overflow the transmit ceiling.

Value tables for `alarms` states and `commandStatus` are in Appendix A.

### 8.4 Digital inputs (DI)

Digital inputs behave differently from everything else and this is the single most important behavioural note in the telemetry section: **DI is event-driven, not periodic.** The device samples the inputs every ~2 seconds internally but only publishes when a value actually changes. A steady input produces no traffic. When a transition occurs it is queued and published on the next poll cycle. This is deliberate — it stops an unchanging input from flooding the pipeline every 30 seconds. Do not assume a fixed DI cadence, and do not treat the absence of DI messages as a fault; it usually just means nothing changed.

```json
{
  "HW_ID": "860710081332507DI",
  "di": {"DI1": 0, "DI2": 1, "DI3": 0, "DI4": 1},
  "TS": "2026-07-10 05:27:59"
}
```

This example shows all four channels enabled (`diEnable`'s bitmask set to `15`). Only the digital-input channels that are actually enabled in the configuration appear — a station with only DI1/DI2 enabled would publish just `{"DI1": 0, "DI2": 1}`, with no `DI3`/`DI4` keys at all. Values are the integer `0` or `1`, or the numeric `404` (unquoted) if that channel could not be read. As noted in section 4, DI is the one place where `404` is numeric rather than a string.

Because a transition is queued and drained on the next cycle, the `TS` on a DI message is the time of the poll cycle that flushed it, which can be up to ~30 seconds after the transition was actually detected.

### 8.5 Diagnostics and sysmon

These are the two fleet-wide health reports. Both publish about every 5 minutes (once every 10 poll cycles at 30 s each), back to back, onto the same `<base>/data` topic as everything else. They are told apart from equipment telemetry — and from each other — the same way as every other message type: by which top-level key is present, `"diagnostics"` or `"sysmon"`. They answer different questions: `diagnostics` is the operational/SLA view you would put on a fleet health dashboard, while `sysmon` is the lower-level view you would consult when a specific gateway is misbehaving. Both carry the bare IMEI as `HW_ID`.

Diagnostics payload:

```json
{
  "HW_ID": "860710081332507",
  "diagnostics": {
    "fw": "1.0.0",
    "temp_c": 46.5, "thermal_zone": "normal",
    "heap_free": 274160, "rssi": 26, "vbat_mv": 3820,
    "operator": "Airtel", "uptime_s": 5231,
    "pub_ok": 412, "pub_fail": 3, "loop_max_ms": 1840,
    "du_ok": true, "radar_ok": true, "genset_ok": false, "di_ok": true, "gps_ok": false
  },
  "TS": "2026-07-10 05:27:59"
}
```

Sysmon payload (a superset with system internals):

```json
{
  "HW_ID": "860710081332507",
  "sysmon": {
    "fw": "1.0.0",
    "cpu_temp_c": 46.5, "thermal_zone": "normal", "cpu_freq_mhz": 160,
    "heap_free": 274160, "heap_min_ever": 261200, "heap_largest_block": 262144,
    "uptime_s": 5231, "reset_reason": 1, "boot_count": 37,
    "rssi": 26, "vbat_mv": 3820, "operator": "Airtel",
    "ota_partition": "ota_0", "main_stack_hwm": 3200, "di_stack_hwm": 2600,
    "pub_ok": 412, "pub_fail": 3, "loop_max_ms": 1840,
    "du_ok": true, "radar_ok": true, "genset_ok": false, "di_ok": true, "gps_ok": false
  },
  "TS": "2026-07-10 05:27:59"
}
```

Notes on specific fields:

- `thermal_zone` is derived from temperature: `normal` below 65 °C, `warm` 65–75, `hot` 75–80, `critical` at or above 80.
- `rssi` is the raw modem CSQ value (0–31, or 99 for "unknown"), not dBm. If you want dBm: `dBm = -113 + 2 × rssi`. A value of 99 means the modem could not measure signal (usually antenna or attach issue).
- `pub_ok` / `pub_fail` are publish counters accumulated over the reporting window and reset after a confirmed diagnostics publish. `loop_max_ms` is the worst poll-cycle duration in the window.
- `du_ok` / `radar_ok` / `genset_ok` / `di_ok` / `gps_ok` reflect the most recent poll of each subsystem (a logical AND across all units of that type — one dead dispenser out of three makes `du_ok` false). A subsystem that is not configured keeps its default and means nothing. Two specific traps: `gps_ok` is `false` on essentially every gateway simply because GPS is disabled (section 8.6), and even on a GPS-enabled gateway it stays `false` for the first few minutes after boot while the receiver acquires its first fix — that is normal, not a fault. Do not alert on `gps_ok: false` alone.
- `reset_reason` is the raw ESP-IDF reset-reason enum. The values you will actually care about are `1` (power-on), `3` (clean software restart, e.g. after a config change or FOTA), `4` (crash/panic), `6` (task watchdog), and `9` (brownout / power problem). A gateway cycling on `4`, `6`, or `9` is unhealthy. The full enum is in Appendix B.
- `boot_count` is a persistent counter that survives reboots. A rapidly climbing `boot_count` with a low `uptime_s` is a reboot loop and should alert.
- `ota_partition` (`ota_0` / `ota_1`) tells you which firmware slot is currently running; it flips after each successful FOTA.

### 8.6 GPS

**GPS ships disabled and is off on every deployed gateway.** `gpsEnable` defaults to `false` and has only ever been turned on for bench testing. This section documents the payload shape so you can build against it now, ahead of any real deployment — but do not write dashboard or alerting logic that assumes GPS data is present until specific gateways are confirmed enabled. For the foreseeable future, treat "no `gps` messages from this device" as the normal case.

Every gateway's modem (Quectel EC200U-CN) has GNSS built into the same module used for cellular, with its own dedicated antenna connector separate from the main LTE antenna. Whether a given gateway can actually get a fix depends only on whether that antenna is physically connected on that unit — this is a hardware population difference, not a firmware capability difference, so the feature is gated by config (`gpsEnable`, section 10.1) rather than compiled in or out.

When enabled, one message publishes per poll to `<base>/data`, using the bare IMEI as `HW_ID` — like `status`/`diagnostics`/`sysmon`, GPS describes the gateway's own fixed location, not a specific piece of equipment behind it, so there is no suffix key for it.

```json
{
  "HW_ID": "860710081332507",
  "gps": {"lat": 12.887630, "lon": 77.641710},
  "TS": "2026-07-17 15:38:35"
}
```

`lat` and `lon` are decimal degrees — positive north/east, negative south/west. The coordinates above are a real captured fix from bench testing, not a made-up example.

That is the whole payload. The modem's location query also returns altitude, HDOP, satellite count, speed, course and fix time, but none of those are published — they were dropped deliberately as noise for an asset that is bolted to the ground. Worth knowing: that means there is **no quality or confidence signal** attached to a coordinate. If you later need to tell a solid fix from a marginal one — say, to raise a "this gateway has moved" alert without false-positiving on a bad fix — the firmware has to start sending HDOP or satellite count first. Ask before designing against it.

If the GNSS engine has no fix yet (normal for a while after a gateway with GPS enabled boots — a cold fix can take minutes) or the modem query fails, the whole `gps` value collapses to `{"data": "404"}`, same as every other subsystem's failure shape:

```json
{"HW_ID": "860710081332507", "gps": {"data": "404"}, "TS": "2026-07-17 05:30:22"}
```

Polled roughly every 10 minutes — far slower than other telemetry — because the gateway is a stationary asset; its position essentially never changes after installation, so there's no value in re-querying on a 30s/60s cadence.

## 9. Status and lifecycle events (device → cloud)

These are state announcements, not sensor readings, and they all publish on `<base>/data` with the bare IMEI as `HW_ID`. They are one-shot events.

### 9.1 Boot

`{"HW_ID":"...","status":"ready","fw":"1.0.0"}` — booted, online, and at least one piece of equipment is configured.

`{"HW_ID":"...","status":"unconfigured","fw":"1.0.0"}` — booted and online but nothing is configured yet. This is what a factory-fresh gateway sends, and it is your cue to onboard it.

### 9.2 Configuration lifecycle

Every `setconfig` that is well-formed produces this ordered sequence. Automated tooling should key off it explicitly. Every message carries the gateway's bare IMEI as `HW_ID` — on the shared `<base>/data` topic that field is how you match an ack to the device you configured (there is no per-device topic to route on; see the correlation note in section 12):

1. `{"HW_ID":"<imei>","status":"config_received"}` — sent the instant a recognised config payload arrives, before any validation. This only confirms delivery.
2. Then exactly one of:
   - `{"HW_ID":"<imei>","status":"config_successful","fw":"1.0.0"}` — validated and saved. The device reboots immediately after this. **Do not treat a change as committed until you see this message.**
   - `{"HW_ID":"<imei>","status":"config_failed","reason":"<reason>"}` — rejected, and the device does not reboot. For every reason except one, nothing changed at all: the device is still running exactly the config it had before. The exception is `nvs_save_failed`, where the config passed validation and *is* live in RAM but could not be written to flash — see that row in the table below.

Two caveats on `config_successful`. First, for connectivity changes it means "accepted for trial," not "confirmed working" — see section 10.1. Second, it is published immediately before the device reboots, so treat it as "the device took this and is restarting now," not as a promise that the reboot succeeded; the `ready` that follows is what confirms that.

Config rejection reasons:

| reason | Meaning |
|---|---|
| `bad_payload` | Not valid JSON, or contained none of the recognised keys. A stray message on this topic (for example a FOTA URL sent to the wrong topic) lands here and is ignored without a reboot. |
| `radar_mixed_style_use_radarType+tankCount_OR_ztechCount/pantechCount` | Payload combined the legacy `radarType`+`tankCount` style with the newer `ztechCount`/`pantechCount` style. Use one or the other. |
| `radar_incomplete_send_radarType_and_tankCount_together` | Sent only one of the paired `radarType` / `tankCount` keys. |
| `radar_type_invalid_use_ztech_pantech_or_none` | `radarType` was not one of `ztech`, `pantech`, `none`. |
| `gensetSlave <n> collides with radar slave range` | The genset Modbus address overlaps an active radar range. This reason is a freeform string containing the offending slave number, so match on the `gensetSlave ... collides` prefix, not the whole string. |
| `ztechCount <n> and pantechCount <n> produce overlapping radar slave ranges` | `ztechCount` >= 3 together with any `pantechCount` > 0 — Ztech's range would reach slave 6, Pantech tank 1's fixed address. Freeform string with both counts embedded; match on the `produce overlapping radar slave ranges` suffix. Field-standard 2 Ztech + 2 Pantech never triggers this. |
| `empty_apn` | A connectivity change was requested but the resulting APN was empty. |
| `bad_mqtt_broker_fields` | Broker host or base topic empty, port out of range, or base topic contained an MQTT wildcard (`#` or `+`). |
| `invalid_chars_in_connectivity_fields` | An APN or broker field (host, user, password, base topic) contained a double quote, backslash, or control character. These are embedded verbatim into the modem's AT commands and can never form a working configuration, so they are rejected upfront rather than burning a reboot + connectivity trial. |
| suffix errors | One of: `<Label> suffix must be exactly 2 alphanumeric characters`; `more than one equipment has no suffix assigned (bare IMEI)`; `<A> and <B> have duplicate suffix <sfx>`. |
| `nvs_save_failed` | Config validated and is running in RAM, but writing it to flash failed. A reboot would lose it, so this is reported as a failure rather than a false success. |

A read-back of the live config is available on demand via `{"cmd":"getconfig"}`; its `config_report` shape is in section 10.2.

### 9.3 FOTA lifecycle

A firmware update produces this sequence on `<base>/data`. Each message carries the gateway's bare IMEI as `HW_ID` (omitted from the shapes below for brevity, but always present — it is how you attribute the update progress to a device on the shared topic):

- `{"HW_ID":"<imei>","status":"fota_initiated"}` — the download is starting. This is the last message before the device drops MQTT to run the HTTP download; it goes silent until the update finishes one way or the other.
- `{"HW_ID":"<imei>","status":"fota_successful"}` — download and flash succeeded; the device reconnects briefly to send this, then reboots into the new image. This does **not** yet mean the new firmware is confirmed good.
- `{"HW_ID":"<imei>","status":"fota_completed","fw":"<new version>"}` — sent after the reboot, once the new image has survived a full poll cycle. This is the real "the update worked" signal. If it never arrives, the new image failed to run and the bootloader will fall back to the previous one on its own.
- `{"HW_ID":"<imei>","status":"fota_failed","reason":"<token>"}` — the download or flash failed; the device reconnects and reports why. Reason tokens are listed below.
- `{"HW_ID":"<imei>","status":"fota_skip_known_bad"}` — the device was handed a URL byte-identical to one that already failed this session and refused to retry it. See the retained-message note in section 10.3.

FOTA failure reason tokens:

| Category | Tokens |
|---|---|
| URL / request setup | `non_https_url`, `url_setup_no_connect`, `url_setup_no_ok`, `get_rejected`, `get_timeout` |
| HTTP response | `http_<code>_err_<err>` (e.g. `http_404_err_0`), `bad_content_length` |
| Saving to modem storage | `readfile_rejected`, `readfile_timeout`, `readfile_err_<n>`, `file_open_failed`, `file_open_parse` |
| Flashing | `no_ota_partition`, `ota_begin_failed`, `flash_timeout`, `read_corrupt_length`, `read_short`, `read_cmd_too_long`, `ota_write_failed`, `download_read_failed`, `image_verify_failed`, `set_boot_failed` |
| Network recovery | `network_rebuild_failed` |

The most common one you will see in practice is `http_<code>_err_<err>`. A `code` of `404` means the URL is wrong or the file is missing; `403` means the object is not readable; a `code` of `-1` with a non-zero `err` means the request never reached the server at all (DNS, TLS, or connectivity).

### 9.4 Connectivity rollback

`{"HW_ID":"<imei>","status":"connectivity_rollback","reason":"new_settings_unreachable"}` is published on `<base>/data` when a previously pushed APN or broker change failed to reach a live broker on the next boot and the device reverted to its last known-good connectivity settings. See section 10.1 for the full trial mechanism — this message is the signal that a connectivity change did not stick.

It arrives shortly after the device reconnects on the reverted settings, not at the moment of the rollback itself — the device has to get back online before it can tell you anything. The device keeps the notice pending until it has been published successfully, so if the link is still unstable on that first attempt, expect the message on a subsequent boot instead. You will get it exactly once, whenever the link is good enough to carry it.

## 10. Commands (cloud → device)

### 10.1 setconfig

Publish a JSON object to `<base>/<imei>/setconfig`. Only the keys you include are changed; anything you omit keeps its current value. Accepted keys:

| Key | Type | Valid values | Controls |
|---|---|---|---|
| `duBlueskyCount` | int | 0–6 | Number of dispenser units |
| `radarType` + `tankCount` | string + int | `ztech`/`pantech`/`none`, 0–4 | Legacy radar config (must be sent as a pair) |
| `ztechCount` / `pantechCount` | int | 0–4 each | Radar config, explicit style (lets both types run at once) |
| `gensetType` | int | 0 = off, 1 = on | Enable genset polling — despite the name, `1` just means "genset polling is on," not "controller is Sedemac." `gensetModel` is what actually selects the controller family, including DEIF. |
| `gensetModel` | int | 0–6, see note below | Controller family |
| `gensetSlave` | int | 1–247 | Genset Modbus address |
| `gensetBaud` | int | 4800, 9600, 19200 | Genset serial baud |
| `diEnable` | int (bitmask) | bit0 = DI1 … bit3 = DI4 | Which digital inputs are active |
| `gpsEnable` | bool | `true` / `false` | Enable GPS position reporting (section 8.6) — leave `false` unless this specific gateway's GNSS antenna is confirmed physically connected. Defaults to `false`; most gateways should never have this set. |
| `sfxGenset`, `sfxDI` | string | exactly 2 alphanumeric chars, or empty | HW_ID suffix for genset / DI |
| `sfxBs1` … `sfxBs6` | string | exactly 2 alphanumeric chars, or empty | HW_ID suffix per dispenser |
| `sfxZtech1` … `sfxZtech4` | string | exactly 2 alphanumeric chars, or empty | HW_ID suffix per Ztech tank |
| `sfxPantech1` … `sfxPantech4` | string | exactly 2 alphanumeric chars, or empty | HW_ID suffix per Pantech tank |
| `apn` | string | non-empty if any connectivity key is sent | Cellular APN |
| `apnUser`, `apnPass` | string | optional | APN credentials |
| `mqttHost` | string | non-empty if any connectivity key is sent | Broker host |
| `mqttPort` | int | 1–65535 | Broker port |
| `mqttUser`, `mqttPass` | string | optional | Broker credentials |
| `mqttBaseTopic` | string | non-empty, no `#` or `+` | Topic prefix |
| `mqttTls` | bool | `true` / `false` | TLS on/off |

**`gensetModel` values, and an important distinction between them:**

| Value | Controller | Has a working driver? |
|---|---|---|
| 0 | Sedemac GC1211 | Yes |
| 1 | Sedemac GC1111 | Yes |
| 2 | DEIF SGC 120 Mk II | Yes |
| 3 | Sedemac GC1201 | **No — reserved ID, always reports genset data as `404`** |
| 4 | Sedemac GC1202 | **No — reserved ID, always reports genset data as `404`** |
| 5 | Sedemac GC1203 | **No — reserved ID, always reports genset data as `404`** |
| 6 | Sedemac GC902 | **No — reserved ID, always reports genset data as `404`** |

IDs 3–6 correspond to stations already running that specific hardware in the field, but there's no verified register map for them yet (their protocol documentation isn't available as of this writing). They are deliberately **not** aliased to GC1211/GC1111 — despite being the same Sedemac GenControl family, their alarm/status register block is documented one address higher than GC1211/GC1111's wire-verified offset, so reusing that driver against this hardware would silently misread every alarm rather than fail cleanly. Selecting 3–6 today gets you a clean, honest `{"data":"404"}` instead. If a station has one of these controllers, configure it with the matching ID now so it reports 404 (not wrong data) until a real driver ships — don't fall back to 0 or 1 as a workaround.

**Out-of-range numeric values are coerced, not rejected — validate them client-side.** This is the one area where a typo will not come back as `config_failed`; it will be quietly accepted as something else, so your dashboard should be the thing that catches it:

- `gensetModel` above 6 → silently becomes 0 (GC1211). A typo here means quietly polling the wrong controller family.
- `gensetBaud` other than 4800/9600/19200 (including 0) → silently becomes 9600.
- `gensetSlave` outside 1–247 → silently becomes the default, 10.
- `gensetType` other than 0 or 1 → behaves as off (no genset polling), and is normalised to 0 on the next reboot.

Whatever slave address results also cannot fall inside an active radar's address range — if Ztech is configured (up to 4 units), slaves 4–7 are reserved; if Pantech (up to 4 units), 6–9 — and a collision is rejected. This range grows with however many Ztech/Pantech units are actually configured, not the full 4-unit cap, so a station with 1 Ztech tank only reserves slave 4. Radar config accepts either the legacy paired style or the explicit-count style but not both in one message.

Ztech and Pantech's own ranges sit right next to each other (4.. and 6..), not spaced apart, so they can collide with **each other** too if a station runs more than the normal 2 Ztech tanks alongside any Pantech: `ztechCount` ≥ 3 pushes Ztech's range onto slave 6, Pantech tank 1's fixed address. That combination is rejected the same way a genset/radar collision is — `config_failed` with a reason naming both counts. The field-standard 2 Ztech + 2 Pantech never hits this.

Two radar styles exist for backward compatibility. Prefer the explicit `ztechCount` / `pantechCount` style for new work; it is the only way to run both radar types on one gateway.

**Connectivity changes are a trial, and this is the most important behaviour in this section.** If a `setconfig` touches any APN or broker field, the device stages the change, saves the previous values as a fallback, and reboots to try the new settings. The `config_successful` you receive means "the change was accepted and the device is about to try it" — it does not mean the new settings work. What happens next:

- If the device reaches a live broker session on the new settings, they become permanent. You will simply see the device reconnect (on the new broker, if you changed it) and resume publishing. There is no separate "confirmed" message; a later `getconfig` will show `connPending: false`.
- If the new settings fail — bad APN, wrong broker host, wrong credentials, unreachable port — the device automatically reverts to the last known-good settings, reboots on those, and publishes `connectivity_rollback`. A bad connectivity push cannot permanently strand a gateway.

The practical consequence: for connectivity changes, wait for either a clean reconnect on the new settings or a `connectivity_rollback` before you conclude anything. Do not treat `config_successful` alone as confirmation. If you are changing the broker's own credentials, make sure the broker accepts the new credentials before or at the moment you push them; the safest approach is to have the broker accept both old and new during the switch (for example by provisioning a second account) so the device's reconnect cannot fail whichever set it happens to try.

### 10.2 System commands

These also go to `<base>/<imei>/setconfig`:

- `{"cmd":"restart"}` — reboot the gateway. No acknowledgement is sent; the device just restarts.
- `{"cmd":"factory_reset"}` — erase stored configuration and reboot. The device comes back up `unconfigured`.
- `{"cmd":"getconfig"}` — publish the live configuration back on `<base>/data` as a `config_report`. This does not change anything or reboot.

The `config_report` shape:

```json
{
  "HW_ID": "860710081332507", "status": "config_report", "fw": "1.0.0",
  "duBlueskyCount": 2, "ztechCount": 1, "pantechCount": 0,
  "gensetType": 1, "gensetModel": 0, "gensetSlave": 10, "gensetBaud": 9600,
  "diEnable": 3, "gpsEnable": false,
  "sfxGenset": "GS", "sfxDI": "DI",
  "sfxBs1": "", "sfxBs2": "", "sfxBs3": "", "sfxBs4": "", "sfxBs5": "", "sfxBs6": "",
  "sfxZtech1": "", "sfxZtech2": "", "sfxZtech3": "", "sfxZtech4": "",
  "sfxPantech1": "", "sfxPantech2": "", "sfxPantech3": "", "sfxPantech4": "",
  "apn": "airtelgprs.com", "apnUser": "", "apnPassSet": false,
  "mqttHost": "broker.example.com", "mqttPort": 8883, "mqttUser": "station01",
  "mqttPassSet": true, "mqttBaseTopic": "fuelsense", "mqttTls": true,
  "connPending": false
}
```

Passwords are never echoed. `apnPassSet` and `mqttPassSet` are booleans telling you only whether a password is stored, not what it is. `connPending: true` means a connectivity change is currently on trial and has not yet been confirmed by a successful broker session — under normal steady state this is `false`.

### 10.3 FOTA trigger

Publish a plain HTTPS URL string (not JSON) to `<base>/<imei>/update` for one device, or to `<base>/update` for the whole fleet. Plain `http://` URLs are rejected; the URL must start with `https://`.

The payload is the URL and nothing else — the device does not parse JSON on this topic. Leading and trailing whitespace, quotes, and newlines are stripped before use, so a trailing newline from your publishing tool is harmless.

**The broadcast topic hits every gateway.** `<base>/update` carries no IMEI, so every device subscribed to that base topic will start downloading. Use `<base>/<imei>/update` for anything other than a deliberate fleet-wide rollout.

**Do not publish FOTA URLs with the retain flag set.** The device never retains anything itself, but a retained URL is redelivered by the broker on every single reconnect — so a bad URL means the device is handed the same failing update forever, going offline for a download attempt each time. The firmware limits the damage: after a URL fails, an *exactly identical* URL is refused with `fota_skip_known_bad` rather than retried. Two things to know about that guard:

- It matches the URL exactly. A different URL — including one that merely starts with the failed one, like `…/fw.bin.bak` after `…/fw.bin` failed — is treated as a new attempt and will be tried.
- It lives in RAM only, so it is forgotten on reboot. Retrying a previously-failed URL without an intervening reboot gets you `fota_skip_known_bad`; after any reboot the same URL will be attempted again.

If you do publish retained, clear or replace the retained message once the update lands or once you know the URL is bad.

### 10.4 Dispenser rsp / kfactor commands

These are raw pass-through commands to a specific dispenser. Send a hex string to the unit's `in` topic and read the response from its `out` topic.

- Command in: `<base>/rsp/imei/<hw_id>/in` or `<base>/kfactor/imei/<hw_id>/in`, where `<hw_id>` is the dispenser's HW_ID (IMEI + suffix).
- Payload: a hex string. Separators are optional and ignored, so `f5015ab6d7` and `f5,01,5a,b6,d7` are equivalent. It is case-insensitive.
- Response out: the raw dispenser reply as a hex string on the matching `/out` topic, or the literal string `404` if the dispenser did not answer. The response is not JSON.
- Latency: the device services these between poll steps, so expect a response within about a second. Set your request timeout accordingly.

## 11. Dashboard and frontend integration guidance

This section is for whoever builds the station dashboard rather than the ingestion pipeline. It assumes the dashboard is consuming already-parsed `raw_data` rows from the backend, not raw MQTT.

**Rendering the equipment list.** There is no discovery message that announces what equipment exists on a gateway — the first sign of a piece of equipment is normally its first telemetry message. Do not build the equipment list purely by watching telemetry arrive; a unit that has been configured but has not yet reported, or has been offline since configuration, would then simply be missing from the UI with no explanation. Instead, treat the `config_report` from `{"cmd":"getconfig"}` (section 10.2) as the authoritative equipment list — it gives the exact count and suffix, and therefore the exact HW_ID, of every configured unit whether or not it has reported yet. Render a panel for every entry in the config report immediately, in a "waiting for data" state, and fill it in as HW_ID-tagged telemetry arrives.

**Suffix changes and history.** If an operator edits a unit's suffix, the HW_ID that unit publishes under changes from that point on. Historical data recorded under the old HW_ID is not retroactively relabeled — it stays attached to the old HW_ID. A dashboard keyed purely on HW_ID will see what looks like a brand-new piece of equipment with no history immediately after a suffix edit, while the old HW_ID simply stops receiving new data. If continuity across a suffix change matters to the product, that mapping needs to be tracked on the backend ("this HW_ID is a rename of that HW_ID as of this timestamp") — the firmware has no concept of it.

**Distinguishing `404` from a real reading.** Several fields use `404` (string or numeric — see section 4) to mean "no reading this cycle," not zero. This matters most for digital inputs and genset values, where `0` is a legitimate real reading. Render `404` explicitly — a dash, an "unavailable" label, a distinct visual state — rather than folding it into the same display as a real `0`. Showing `404` as `0` misrepresents a sensor fault as a normal reading.

**Alarm severity display.** The four genset alarm states (Appendix A) map naturally onto a standard severity scale: `Shutdown` and `ElecTrip` both mean the genset has actually stopped or tripped — treat both as critical. `Warning` is a middle severity. `Notification` is informational, the lowest priority of the four. If `alarmsTruncated` is present, show an indicator that the alarm list may be incomplete rather than silently displaying only what fit.

**Deriving online/offline.** There is no explicit "device offline" message — it has to be inferred from silence. Base the threshold on the gateway's own known silent windows so normal behaviour does not get flagged: the nightly reboot (60–90 seconds, once daily near 02:00), FOTA downloads (offline for the duration, usually under a minute on a decent connection but longer on a poor one), and connectivity recovery after a signal drop (silent while it retries before reconnecting or rebooting). A reasonable starting point is to treat a gateway as offline after 10 minutes with no message of any kind on any of its topics. Treat this as a starting recommendation to tune against real fleet behaviour, not a fixed value defined by the firmware.

**Setconfig from the UI.** Since `config_received` / `config_successful` / `config_failed` carry no identifier tying them to a specific request (section 12), disable the save action on a configuration form until a terminal response has been seen for the in-flight request, rather than allowing a second push to queue up behind the first.

## 12. Operational notes

**Nightly reboot.** Each gateway reboots once a day, when its network clock reaches roughly 02:00, and re-arms after 03:00. This produces a silent window of about 60–90 seconds every night while it restarts and reconnects. This is intentional and healthy. Do not alert on the resulting gap.

**Offline message queueing and expiry.** The device uses a persistent MQTT session, so while it is offline the broker queues QoS-1 messages addressed to it (setconfig, FOTA, rsp/kfactor) and delivers them on reconnect. If your broker is configured with message or session expiry, a command published during a long device outage can expire before the device comes back, and it will never see it. Keep this in mind when scripting config pushes to gateways that may currently be offline.

**Message correlation and idempotency.** Status and lifecycle messages carry no request ID. A `config_successful` or `config_failed` cannot be tied back to a specific `setconfig` push except by being the next terminal response received for that device. If you need to correlate a response to a particular change, serialize your pushes to a given device — wait for a terminal response before sending the next one. Separately, QoS 1 can redeliver a message the device already sent; every status and lifecycle message describes a state, not a delta, so reprocessing a duplicate is always safe.

**Silence during recovery and FOTA.** When cellular connectivity drops, the gateway works through several recovery steps on its own and stays silent until it is back. It also disconnects entirely during a FOTA download. Short silences are normal — see the online/offline guidance in section 11 for a concrete threshold.

**Validate suffixes before sending.** The device enforces HW_ID suffix uniqueness and will reject a colliding config, but catching the collision in the dashboard before the round-trip gives the operator a clearer, faster error.

## 13. Security and production readiness

A short, factual note on the current security posture, relevant to whoever operates the broker and plans the production rollout.

**Transport encryption with certificate chain validation (as of 2026-07-17).** The gateway uploads a trusted root CA bundle to the modem at boot and validates both the MQTT broker's and the FOTA server's certificates against it (`seclevel=1` on both TLS contexts — previously `seclevel=0`, no validation at all, plus FOTA specifically had signature and chain-verification checks explicitly disabled). This closes what was the most severe gap in the system: previously, a network-position attacker (DNS hijack, rogue AP, compromised APN routing) could present any certificate, including a self-signed one, and the device would accept it — for FOTA specifically, that meant an attacker in that position could have served malicious firmware and had it flashed and run.

The bundle carries two roots, because the endpoints are issued by different CAs (each confirmed by inspecting its live TLS handshake): **ISRG Root X1** (Let's Encrypt) covers the MQTT broker, and **Amazon Root CA 1** covers the S3-hosted FOTA server. Both TLS contexts trust both roots, so either endpoint can move between these two CAs without a firmware change. If an endpoint ever moves to a CA outside this list, its handshake fails closed — connection refused, not silently accepted, which is the correct direction to fail — but it does mean the new root has to be added to the firmware first. This was verified end-to-end on real hardware: MQTT connects with validation active, and FOTA downloads completed successfully from both S3 (Amazon chain) and a GitHub-hosted URL (a four-level Let's Encrypt chain), so both roots and the full chain-verification path are known-good.

One caveat remains: this validates the certificate **chain**, not the **hostname**. The dev broker's certificate is issued for a different hostname than the device connects to (a known, separate issue tracked for the production cutover), and the modem accepts it — so impersonation by a party holding *any* certificate validly issued by these two CAs is not closed by this change. Fixing that requires the production certificate to actually match the hostname the firmware connects to.

**Shared credentials.** Every gateway currently authenticates to the broker with the same MQTT username. Simple to operate at pilot scale, but a single leaked credential affects the whole fleet rather than one device, and rotating it means touching every device rather than one. Worth moving to per-device or per-batch credentials before fleet size makes a shared-credential rotation impractical to coordinate.

**No secure boot / flash encryption.** The ESP32-S3's flash is neither encrypted nor signature-verified at boot — physical access to a device (opening the enclosure, using the onboard USB programming port) allows reading the complete firmware image, including the compiled-in fallback broker/APN credentials, and writing arbitrary firmware back. Both are supported by the chip and are a planned follow-up, but enabling them is a one-way, per-chip decision (burns eFuses; there is no factory reset back to unencrypted/unsigned) and cannot be applied retroactively to already-fielded devices over the air — it only takes effect from first boot of a device provisioned with it enabled. Treat this as a manufacturing/provisioning-process decision, not a firmware patch.

**Access control lives at the broker, not the device.** Setconfig has no authentication beyond the MQTT connection itself — anything arriving on a gateway's subscribed topics is treated as authorized, including connectivity changes. Broker-side ACLs restricting which clients may publish to which device's topics are the actual access control here.

## 14. Quick reference

| Topic | Direction | Payload |
|---|---|---|
| `<base>/data` | Device → Cloud | Everything device→cloud: telemetry, config acks, lifecycle status, and `diagnostics`/`sysmon` health reports (JSON) |
| `<base>/<imei>/setconfig` | Cloud → Device | Config, `restart`, `factory_reset`, `getconfig` (JSON) |
| `<base>/<imei>/update` | Cloud → Device | HTTPS URL (string) |
| `<base>/update` | Cloud → Device | HTTPS URL, broadcast (string) |
| `<base>/rsp/imei/<hw_id>/in` | Cloud → Device | Hex command |
| `<base>/rsp/imei/<hw_id>/out` | Device → Cloud | Hex response, or `404` |
| `<base>/kfactor/imei/<hw_id>/in` | Cloud → Device | Hex command |
| `<base>/kfactor/imei/<hw_id>/out` | Device → Cloud | Hex response, or `404` |

### Glossary

- **IMEI** — the modem's 15-digit identity; the gateway's root identifier and the `device_id` in the pipeline.
- **HW_ID** — a specific piece of equipment's address: IMEI plus a 2-character suffix.
- **BlueSky** — the fuel dispenser protocol/vendor.
- **Ztech / Pantech** — the two tank-level radar vendors.
- **Genset** — generator set. Controller is Sedemac (GC1211/GC1111) or DEIF (SGC 120 Mk II), selected via `gensetModel`.
- **DI** — discrete digital inputs on the gateway.
- **FOTA** — firmware-over-the-air update.

## Appendix A — Genset value tables

Three controller families are supported (`gensetModel`: 0=GC1211, 1=GC1111, 2=SGC120 — section 10.1). The alarm severity states below are universal across all three. The alarm **name** keys are not — each family has its own list, and a device only ever emits keys from the list matching its configured model, never a mix of both.

Alarm severity states (values inside the `alarms` object). Only these four ever appear on the wire, on any model:

| State | Meaning |
|---|---|
| `Warning` | Active, warning level (non-shutdown) |
| `Shutdown` | Active and has shut the genset down |
| `ElecTrip` | Active, electrical trip |
| `Notification` | Active, lowest-priority notice |

`commandStatus` values (GC1211 and GC1111 only — the DEIF SGC 120 never sends this key, see section 8.3):

| Value | Meaning |
|---|---|
| `Unknown` | Unknown / out of expected range |
| `Started` | Last start command started the genset |
| `Stopped` | Last stop command stopped it |
| `AlreadyON` | Start issued while already running |
| `AlreadyOFF` | Stop issued while already stopped |
| `ManualMode` | Command ignored because controller is in manual |
| `NotStarted` | Start commanded but genset did not start |

### A.1 Sedemac (GC1211 / GC1111) alarm-name keys

The possible keys inside `alarms` when `gensetModel` is 0 or 1. Five of the 40 underlying register bits are unused on this controller and never produce a key; they are left out of the list below rather than shown as placeholders:

```
RadiatorWaterLevel  LowFuelLevel     HighEngTemp      LOP
FailToStop          FailToStart      OverSpeed        UnderSpeed
GensetHighFreq      GensetLowFreq    GensetHighVolt   GensetLowVolt
EmergencyStop       UnbalancedLoad   Overload         HighCurrent
AirFilterMaint      FuelFilterMaint  OilFilterMaint   ChargingAltFail
EngTempOpenCkt      OilPressOpenCkt  BattHighVolt     BattLowVolt
MPULoss             FuelTheft
AuxInputA  AuxInputB  AuxInputC  AuxInputD
AuxInputE  AuxInputF  AuxInputG  AuxInputH  AuxInputI
```

### A.2 DEIF SGC 120 alarm-name keys

The possible keys inside `alarms` when `gensetModel` is 2. All 48 underlying register bits are named on this controller — no unused slots:

```
LOP               HighCoolantTemp   LowFuelLevel      WaterLevelSwitch
UnderSpeed        OverSpeed         FailToStart       FailToStop
ReversePower      LowLoad           GenLowFreq        GenHighFreq
GenHighCurrent    GenOverload       UnbalancedLoad    EmergencyStop
ChargeAltFail     MaintenanceDue    AFTActTimeout     AshLoadMaint
BattLowVolt       BattHighVolt      TempCktOpen       LOPShortBatt
FuelTheft         MPUFault          OilPressOpenCkt   AuxInputI
AuxInputA  AuxInputB  AuxInputC  AuxInputD
AuxInputE  AuxInputF  AuxInputG  AuxInputH
GenL1LowVolt      GenL1HighVolt     GenL2LowVolt      GenL2HighVolt
GenL3LowVolt      GenL3HighVolt     DGPhaseRotation   MainsPhaseRotation
FuelLevelOpenCkt  VBeltBroken       ECUPreheatFail    HighOilPress
```

**This list has not yet been confirmed against a live fault.** It's transcribed correctly from DEIF's own Modbus documentation, and the surrounding measurement fields (oil pressure, battery voltage, run hours, etc.) have been cross-checked against real hardware and match exactly — but every alarm register has read all-zero (no active alarms) in every capture and every field test so far, so the specific name-to-bit mapping and the `status.mode`/`running`/`available` bit positions haven't been proven against a real trip or a running unit. If you're building alerting logic on top of DEIF alarm names specifically, treat the first real one you see as worth a manual sanity check against the physical genset before trusting it fully.

## Appendix B — reset_reason values

The `reset_reason` field in sysmon maps to the ESP-IDF reset-reason enum:

| Value | Meaning | Value | Meaning |
|---|---|---|---|
| 0 | Unknown | 8 | Wake from deep sleep |
| 1 | Power-on | 9 | Brownout (power problem) |
| 2 | External pin | 10 | SDIO |
| 3 | Software restart (normal) | 11 | USB |
| 4 | Crash / panic | 12 | JTAG |
| 5 | Interrupt watchdog | 13 | eFuse error |
| 6 | Task watchdog | 14 | Power glitch |
| 7 | Other watchdog | 15 | CPU lockup |

Values 4, 6, and 9 recurring on a gateway indicate a real fault (crash loop, hang, or power instability respectively).
