# alds-gg VVM701 gateway: cloud integration guide

Firmware 1.0.0, product id `alds-gg-vvm701/1`. Updated 19 October 2026.

Everything the gateway publishes and every command it accepts, copied from the
firmware source. If a payload differs, the firmware is right and this doc is wrong.

## 1. This build, and what changed

The 25 September 2026 build of 1.0.0 is an internal reliability pass — flash wear
on a backlog drain, repair of a queue record torn by a power cut, and catching an
inbound command that lands while the modem is mid-diagnostics. Nothing on the wire
changed: no new topic, payload, key or status, and nothing in the pipeline needs
touching.

**Wire change after that build:** genset messages now carry the complete Modbus
reply as received (`slave, fc, byte count, data, crc`), like radar and the
dispensers, instead of the data bytes alone. Register decoding moves by 3 bytes:
register k is `frame[3+2k]<<8 | frame[3+2k+1]`. Section 6.4.

**Wire change, 8 October 2026:** the diagnostics report (health, `why`, forensics
`last_stop`, all counters) is published on `<base>/logs`, no longer on
`<base>/data`. Payload unchanged. Subscribe to `<base>/logs`; the broker account
must be allowed to publish there. Same as the CNG and Senseflow firmware.
Section 7. Also: before a FOTA hands the gateway to another product, the modem's
GNSS engine, autogps and AGPS are switched off; `fota_completed` is re-sent until
delivered if the new image confirmed itself while offline.

**Wire change, 8 October 2026: setconfig is strict.** A push with a typo'd or
unknown key, a value out of range or of the wrong type, a string too long for its
field, or a cut-short JSON is refused whole with `config_failed` and a `reason`;
it is never clamped or partly applied. Send only documented keys. Section 8.1.

**Wire changes, 16 October 2026 — act on all three:**

1. **`<base>/data` carries polled readings only** (nozzles, tanks, genset, DI,
   GPS). Everything else — every `status` (`ready`, `unconfigured`,
   `config_*`, `config_report`, `fota_*`, `restarting`, ...) — moved to
   `<base>/logs`, next to diagnostics. Payloads unchanged. Lock events stay on
   `.../lock/out`, rsp/kfactor replies on their `.../out`. Section 3.
2. **Dispenser slots.** `du<N>Sfx` set = slot N ON, polling dispenser `<sfx>`;
   `du<N>Sfx` `""` = slot N OFF. An empty suffix no longer means "the position".
   `duCount` must equal the slots ON or the push is refused
   (`duCount 3 but 2 dispenser suffixes set ...`). A unit configured before this
   build keeps polling exactly the dispensers it polled (converted on boot; its
   `config_report` now shows the suffixes). Sections 4 and 8.1.
3. **Diagnostics `sys.cpu`**: per-core CPU load (`core0_avg`, `core1_avg`,
   `core0_peak`, `core1_peak` in %, `window_s`). Section 7.

**Wire change, 18 October 2026: rsp and kfactor are a plain tunnel, and
`setrsp` is gone.** The cloud builds every dispenser command (RSP write,
k-factor write, any read); the gateway sends the frame from
`.../rsp/imei/<hw_id>/in` (BlueSky and Yenen) or `.../kfactor/imei/<hw_id>/in`
(BlueSky) to the pump once, unchanged, and answers `.../out` with the pump's
reply exactly as received, or `404`. It no longer checks the frame, its address
or that the nozzle is idle, never retries and never reads back — don't write
mid-sale: read the status through the tunnel first. Removed:
`<base>/setrsp/imei/<hw_id>/in` and its `RSP_*` acks (a Yenen price is now a
cloud-built frame on the rsp topic, as in the pilot), and the `busy`,
`rsp_rejected`/`kfactor_rejected` and `rsp_readback`/`kfactor_readback`
statuses. Yenen has no kfactor topic. Section 8.5.

Against the previous GG firmware the ingestion still works unchanged — same
telemetry topic, `HW_ID` + `TS` envelope and payload shapes. Its deltas, each
documented in its own section:

- Suffixes are digits only and moved: Ztech `04`.., Pantech after the Ztech tanks,
  DI `11`; genset stays `10`, dispensers `01`..`06`. Section 4.
- setconfig keys are `duCount` + `du<N>Make` + `du<N>Sfx` (were `duBlueskyCount`/
  `sfxBs<N>`), and a bad value is now refused with a reason instead of changed
  silently; `gensetSlave` alone moves the genset. Section 8.1.
- Genset Mahindra GC1211/GC1111 add a third message, `Alarms` (capital A, per the
  DG spec), between `registers` and `status`. Section 6.4.
- Outage readings replay from flash oldest-first, at-least-once — expect bursts of
  old-`TS` records afterwards. Section 5.
- Also, each optional to consume: Yenen dispensers beside BlueSky (6.2), grouped
  diagnostics (7), the fleet-FOTA/lock topics (3, 8.3), GPS
  `hdop`/`nsat`/`spd` (6.6), compiled credentials overriding stored config (2), `rfid` always `"404"`.

## 2. Connection

| | Value |
|---|---|
| Broker (this image) | `mqtt.iotronsys.com:8777`, user `gogas`, base topic `gogas` (production, the same set as alds-ggy) |
| Client id | `iotronics-<imei>` |
| Protocol | MQTT 3.1.1 over TLS, QoS 1 everywhere, persistent session, keepalive 60 s |
| Retain | the device never sets it; the cloud must not either (section 8.3) |

The broker, user, password, base topic and TLS flag are compiled in and enforced
over the unit's stored config on every boot: a unit reflashed for another
deployment comes up on the new broker and announces `firmware_credentials_applied`.
`setconfig` can still change them until the next firmware with different compiled
values. GG and GGY share one production broker and base topic; tell them apart by
`diagnostics.product` and the fleet FOTA topic each listens on. Subscribe with the
base topic from `config_report.mqttBaseTopic`, not one guessed from the hostname.

## 3. Topics

`<base>` is the base topic, `<imei>` the 15-digit modem IMEI, `<hw_id>` the
IMEI plus a 2-digit suffix.

| Topic | Direction | Payload |
|---|---|---|
| `<base>/data` | device to cloud | polled readings only: `nozzles`, `tanks`, `genset`, `di`, `gps` (JSON, section 6) |
| `<base>/logs` | device to cloud | everything else: diagnostics (section 7) and every `status` message (section 8) (JSON) |
| `<base>/<imei>/setconfig` | cloud to device | config, `restart`, `factory_reset`, `getconfig` (JSON) |
| `<base>/<imei>/update` | cloud to device | FOTA for one gateway (URL string or JSON) |
| `<base>/fleet/gg/update` | cloud to device | FOTA for every GG gateway on this base topic |
| `<base>/<imei>/lock` | cloud to device | panel box lock: string containing `unlock` or `lock` |
| `<base>/<imei>/lock/out` | device to cloud | lock ack/events (`unlocked`/`locked`/`ignored_retained`/`bad_command`/`safety_restart`) |
| `<base>/rsp/imei/<hw_id>/in` / `out` | both | dispenser command tunnel, BlueSky and Yenen: cloud-built frame as hex in, the pump's reply as hex or `404` out (section 8.5) |
| `<base>/kfactor/imei/<hw_id>/in` / `out` | both | the same tunnel, BlueSky only |

Readings arrive on `<base>/data` (`nozzles`, `tanks`, `genset`, `di`, `gps`);
diagnostics and every `status` message arrive on `<base>/logs` — subscribe to
both (firmware before 16 October 2026 sent status on `/data`, and before
8 October diagnostics too, so accept them on either during a mixed fleet).
Identify the sender from `HW_ID` and the message kind from the one top-level key
present: `nozzles`, `tanks`, `genset`, `di`, `gps`, `diagnostics`, `status` or
`event`.

## 4. HW_ID and suffixes

`HW_ID` is the IMEI followed by a 2-digit suffix. Every suffix is a number
(`01`..`99`) and every suffix is also the unit's address on the bus.

| Unit | setconfig key | Default | Meaning of the number |
|---|---|---|---|
| Dispenser slot 1-6 | `du1Sfx`..`du6Sfx` | `""` = slot OFF (no default) | the dispenser number. BlueSky dispenser N is nozzle addresses 2N-1 and 2N; Yenen dispenser N is head N. Range `01`..`06`, Yenen `01`..`05` |
| Ztech tank 1-4 | `sfxZtech1`..`4` | `04`, `05`, ... | Modbus slave |
| Pantech tank 1-4 | `sfxPantech1`..`4` | continues after the last Ztech tank | Modbus slave |
| Genset | `sfxGenset` (= `gensetSlave`) | `10` | Modbus slave |
| Digital inputs | `sfxDI` | `11` | label only, not on the bus |

Dispensers use slots (the same rule as the bottling-plant meters): slot N is ON
only when `du<N>Sfx` is set, and the suffix picks the pump. A station whose
dispensers sit at slave ids 1, 2 and 3, with dispenser 1 out of service:

```json
{"duCount":2,"du1Sfx":"","du2Make":"bluesky","du2Sfx":"02","du3Make":"bluesky","du3Sfx":"03"}
```

Only dispensers 2 and 3 are polled; they publish under `<imei>02` (nozzles
`"3"`/`"4"`) and `<imei>03` (nozzles `"5"`/`"6"`). Nothing is changed at the pump.
Turning dispenser 1 back on is `{"du1Sfx":"01","duCount":3}`. `duCount` is a
cross-check: it must equal the number of slots with a suffix (after merging this
push over what is stored), else `config_failed`. Send every slot key each time
to be safe. Two units can never share a suffix (the device refuses it, section
8.1). Status, diagnostics, GPS and lock messages use the bare IMEI.

## 5. Parsing rules

- One equipment unit per message, one top-level key per message.
- Hex is lowercase, 2 digits per byte, comma separated, and is the complete
  frame the device received, for every device: validated, then published with
  nothing removed (Modbus: slave, function code, byte count, data and CRC).
- `"404"` in a field means that read failed. A unit that did not answer at all
  publishes a compact `"data":"404"`; a genset message keeps its own key
  (`{"genset":{"Alarms":"404"}}`); DI uses the number `404`.
- `TS` is `YYYY-MM-DD HH:MM:SS` UTC from the device clock. Readings taken before
  the clock is set are held, not sent with a 1970 timestamp (BlueSky/Yenen).
- No message exceeds 1400 bytes; a larger payload is replaced by its `404` shape,
  never truncated.
- Dispenser readings are queued during an outage (16 in RAM, then up to 768 KB on
  flash) and replayed oldest-first — flash, then RAM, then live — so a newer
  reading never precedes an older one from the same gateway. At-least-once; dedupe
  on `HW_ID` + `TS` + `data`. Radar, genset, DI and GPS are not queued.

## 6. Telemetry

### 6.1 BlueSky dispenser, one message per nozzle, every 20 s

```json
{"HW_ID":"86071008133255601",
 "nozzles":{"1":{"data":"f5,01,ac,00,02,15,18,26,23,30,19,35,34,c5,3f",
                 "rsp":"f5,01,a4,68,86,b6,08",
                 "kfact":"f5,01,a4,23,60,e2,71",
                 "rfid":"404"}},
 "TS":"2026-09-21 12:55:06"}
```

`data` is the totalizer, `rsp` the unit price, both every cycle; `kfact` is read
every 15 min and repeated in between; `rfid` stays but is always `"404"` (not
read). A failed field becomes `"404"`. An absent or busy nozzle publishes
`{"nozzles":{"1":{"data":"404"}}}` and nothing else. Only an idle nozzle is read,
so every `data` is a settled totalizer. A nozzle silent for 5 rounds is probed
every 6th round until it answers.

### 6.2 Yenen dispenser, one message per nozzle, every 10 s

```json
{"HW_ID":"86071008133255603","nozzles":{"5":{"data":"02,34,39,30,32,30,..."}},
 "TS":"2026-09-21 12:55:16"}
```

Only `data`: the raw Command 2 reply as hex. Absent is `{"data":"404"}`. The
frame decode is in the appendix. Status `00` in the frame means idle; only
record from idle frames.

Nozzle labels (both makes): `label = (N-1)*2 + side + 1` with N the dispenser
number, so dispenser `01` publishes `"1"`/`"2"` and dispenser `03` `"5"`/`"6"`,
up to `"12"`. The label is the key inside `nozzles`.

### 6.3 Radar, one message per tank, every 10 min

```json
{"HW_ID":"86071008133255604","tanks":{"1":{"data":"04,03,04,2f,13,00,00,56,22"}},
 "TS":"2026-09-21 12:55:06"}
```

`data` is the raw Modbus RTU reply. Ztech and Pantech both number tanks from
`"1"`; tell them apart by HW_ID. A failed tank gives `{"data":"404"}` and is
re-polled on its own 60 s later, up to 3 times; the 10-minute cadence itself
does not move. Take the latest reading as current.

### 6.4 Genset, every 5 min, up to three messages

```json
{"HW_ID":"86071008133255610","genset":{"registers":"0a,04,48,01,3f,00,3e,00,22,...,1b,08,5e,2c"},"TS":"..."}
{"HW_ID":"86071008133255610","genset":{"Alarms":"0a,04,06,11,10,11,11,11,11,48,44"},"TS":"..."}
{"HW_ID":"86071008133255610","genset":{"status":"0a,04,04,c0,00,00,00,7d,44"},"TS":"..."}
```

One message per Modbus read, in this order, same `TS`. Each is the complete
reply exactly as the controller sent it: `slave, fc, byte count, data..., crc lo,
crc hi`. The gateway has already checked the CRC, the slave and function-code
echo and the length; a reply that fails any check is a `"404"`, never published.
Register k is `frame[3+2k]<<8 | frame[3+2k+1]`. The slave byte is the genset
suffix (`10` → `0a`, `08` → `08`). `Alarms` has a capital A, as the DG spec does.
A failed read gives `"404"` for that message (the key stays), with the same 60 s
re-poll.

| `gensetModel` | Controller | FC | `registers` | `Alarms` | `status` |
|---|---|---|---|---|---|
| 0 | Mahindra GC1211 | 04 | offsets 1-36 | offsets 57-59 (6 B) | offsets 67-68 (4 B) |
| 1 | Mahindra GC1111 | 03 | offsets 1-36 | offsets 57-59 (6 B) | offsets 67-68 (4 B) |
| 2 | SGC 120 Mk II | 03 | offsets 1-78 | none, inside `registers` | none, inside `registers` |
| 3, 4, 5 | Ashok Leyland (spec: "DG 1202/03/04") | 04 | offsets 1-36 | none | offset 67 (2 B) |
| 6 | Sedemac GC902 (not in the spec) | 04 | offsets 1-36 | none | offset 67 (2 B) |

Register maps and scaling are in `GENSET_REGISTER_MAPS.md` in this repo.

### 6.5 Digital inputs, on change only

```json
{"HW_ID":"86071008133255611","di":{"DI1":0,"DI2":1},"TS":"2026-09-21 12:55:06"}
```

Only channels enabled in `diEnable` appear. Sampled every 2 s, published on
change, so silence is normal. An unreadable channel is the number `404`.

### 6.6 GPS, every 5 min, off by default

```json
{"HW_ID":"860710081332556","gps":{"lat":12.887360,"lon":77.641840,"hdop":5.9,"nsat":8,"spd":0.0},"TS":"..."}
```

`lat`/`lon` in decimal degrees. `hdop`, `nsat` (satellites used) and `spd` (km/h)
come whenever the modem reports them; a message without them is still a fix, so key
on `lat`/`lon`. `spd` 0.0 is real (stopped), not missing. No fix gives
`{"gps":{"data":"404"}}`. Off unless `gpsEnable` is set. Same payload as the bowser
gateway (bwsr).

## 7. Diagnostics, every 15 min, on `<base>/logs`

One message on `<base>/logs` (not `/data`) carrying everything about the
gateway's own health: the `health` verdict and its `why` list, the forensics
record of the last stop (`last_stop`) and every counter.

```json
{"HW_ID":"860710081332556","diagnostics":{
  "health":"ok","why":[],
  "fw":"1.0.0","product":"alds-gg-vvm701/1","cred_fp":"3a91c2e7",
  "uptime_s":86412,"boot_count":14,"reset_reason":1,
  "last_stop":{"fault":false,"reset_reason":3,"task":"main","phase":"idle","detail":0,"stalled_ms":0,"marks":1200,"ran_ms":3600000},
  "records":{"q_ram":0,"q_flash":0,"q_bytes":0,"q_pct":0,"spilled":0,"dropped":0,"trimmed":0,"store_ok":true,"rtc_synced":true},
  "link":{"rssi":22,"operator":"Airtel","connected_s":86300,"recov_l1":0,"recov_l2":0,"recov_l3":0,"bearer_distrust":0,"pub_ok":1450,"pub_fail":0},
  "bus":{"nozzles_absent":[],"lock_timeouts":0,"cycle_max_ms":4210,"cycle_avg_ms":3100,"cycles":90},
  "sys":{"heap_free":248000,"heap_min_ever":236000,"stack_min":2400,"temp_c":46.5,"vbat_mv":3820,
         "cpu":{"window_s":900,"core0_avg":3,"core1_avg":1,"core0_peak":9,"core1_peak":2}},
  "subsystems":{"du":true,"radar":true,"genset":false,"di":true,"gps":false}},
 "TS":"2026-09-21 12:55:06"}
```

`health` is the device's own verdict (`ok`, `degraded`, `fault`) with the
reasons in `why`: `store_unmounted`, `records_dropped`, `records_trimmed`,
`last_stop_fault` mean fault; `clock_unsynced`, `nozzle_absent`,
`queue_filling`, `link_rebuilt`, `publish_failures`, `temp_high`,
`link_unstable` mean degraded. Alert on fault, show degraded.

- `records.dropped` and `.trimmed` are readings lost for good; any non-zero
  value is data loss. `q_pct` is how full the flash queue is.
- `link.rssi` is CSQ 0-31 (99 unknown), dBm = -113 + 2 x rssi. `recov_l1..l3`
  count reconnects, network rebuilds and modem power cycles since boot.
  `pub_ok`/`pub_fail` are for this 15-minute window.
- `bus.nozzles_absent` lists the nozzle labels currently not answering.
- `sys.cpu` is each core's load in % over `window_s` (since the last report
  that was delivered): `core0_avg`/`core1_avg` the average, `*_peak` the busiest
  minute. Single digits are normal (the firmware mostly waits on serial lines);
  the key is absent if the build cannot measure it.
- `last_stop` appears once after a reboot: what the device was doing when it
  stopped. `fault:true` is a crash, watchdog or brownout; `false` a planned
  restart. `phase:"recover"`, `detail:4` means the connectivity ladder gave up.
- `reset_reason`: 1 power-on, 3 software restart, 4 panic, 5/6/7 watchdog,
  9 brownout. `boot_count` rising with a low `uptime_s` is a reboot loop.
- `cred_fp` identifies the compiled credential set (same on every unit of a build).

## 8. Status messages and events

All on `<base>/logs` (on `<base>/data` before 16 October 2026), bare IMEI unless
noted, one-shot.

| Payload | When |
|---|---|
| `{"status":"ready","fw":"1.0.0"}` | booted with equipment configured |
| `{"status":"unconfigured","fw":"1.0.0"}` | booted with nothing configured; repeated every 3 min until a setconfig arrives |
| `{"status":"firmware_credentials_applied","broker_changed":B,"config_erased":B,"fw":".."}` | compiled credentials were written over the stored ones. `config_erased:true` means the unit was carrying another product's config (a CNG or GGY unit reflashed as GG) and is now `unconfigured` |
| `{"status":"config_received"}` then `config_successful` or `config_failed` | setconfig handling, section 8.1 |
| `{"status":"config_report",...}` | answer to `getconfig` |
| `{"status":"restarting"}` / `{"status":"factory_resetting"}` | acks before the reboot |
| `{"status":"cmd_ignored_retained","cmd":".."}` | a `restart`/`factory_reset` arrived within 15 s of subscribing, treated as retained and dropped |
| `{"status":"connectivity_rollback","reason":"new_settings_unreachable"}` | a pushed APN/broker change did not reach a broker; reverted |
| `fota_initiated`, `fota_successful` (`"handover":true` on a product swap), `fota_completed` | section 8.3 |
| `fota_failed` (`reason`), `fota_skip_known_bad`, `fota_skip_already_applied`, `fota_skip_reverted`, `fota_ignored_retained`, `fota_reverted` (`image`) | section 8.3 |
| lock events on `<base>/<imei>/lock/out` (not `<base>/logs`): `unlocked` (+`window_ms`), `locked`, `ignored_retained`, `bad_command`, and `safety_restart` on the next boot if an unlock window never closed | panel lock, section 8.4 |

rsp/kfactor replies go to their own `.../out` topic, never to `<base>/logs`
(section 8.5).

### 8.1 setconfig

Publish JSON to `<base>/<imei>/setconfig`. Only the keys sent are changed.

| Key | Type | Values |
|---|---|---|
| `duCount` | int | 0-6; must equal the dispenser slots with a suffix (cross-check, not a switch) |
| `du1Make`..`du6Make` | str | `bluesky` or `yenen` |
| `du1Sfx`..`du6Sfx` | str | `01`..`06` (Yenen `01`..`05`) = slot ON, polling that dispenser; `""` = slot OFF |
| `ztechCount`, `pantechCount` | int | 0-4 each |
| `radarType` + `tankCount` | str + int | `ztech`/`pantech`/`none`, 0-4. Alternative to the counts, never mixed with them |
| `sfxZtech1..4`, `sfxPantech1..4` | str | `01`..`99` = Modbus slave, or `""` |
| `gensetType`, `gensetModel`, `gensetSlave`, `gensetBaud` | int | 0/1; 0-6 (table 6.4); 1-247; 4800/9600/19200 |
| `sfxGenset` | str | the genset's suffix = its `gensetSlave`; send either one, and if both are sent they must agree |
| `diEnable` | int | bitmask 0-15, bit0 = DI1 .. bit3 = DI4 |
| `sfxDI` | str | `01`..`99`, default `11` |
| `gpsEnable` | bool | |
| `apn`, `apnUser`, `apnPass` | str | `apn` non-empty |
| `mqttHost`, `mqttPort`, `mqttUser`, `mqttPass`, `mqttBaseTopic`, `mqttTls` | str/int/bool | host and base topic non-empty, port 1-65535, no `#` or `+` in the base topic |

**A bad push is refused whole; nothing in it is applied** — `config_failed` with
a `reason`, the current configuration keeps running, no reboot. Nothing is
clamped, defaulted or ignored (changed 8 October 2026; counts, `gensetModel`,
`gensetBaud` and switches used to be clamped or skipped and still acked
`config_successful`):

| Refused | `reason` |
|---|---|
| Not one complete JSON object (cut short, two objects, a bare URL) | `bad_payload` |
| A key this firmware does not know (a typo such as `du1make`) | `unknown_key_<key>` |
| The same key twice | `duplicate_key_<key>` |
| Whitespace between a key and its `:` (`"duCount" :2`) | `bad_payload` |
| An object or array as a value | `<key> must be a plain value` |
| A number that is not a whole number, or out of range (`"duCount":9`, `-1`, `"two"`, `2.5`) | `<key> must be a whole number <lo>..<hi>` / `<key> must be <lo>..<hi>` |
| `gensetBaud` other than 4800, 9600, 19200 | `gensetBaud must be 4800, 9600 or 19200` |
| A switch that is not `true`/`false`/`1`/`0` | `<key> must be true or false` |
| A string key sent as a number | `<key> must be a string` |
| A string longer than its field (never cut to size) | `<key> is longer than <n> characters` |
| `cmd` together with config keys | `cmd_must_be_sent_alone` |
| An unknown `cmd` | `unknown_cmd` |
| `duCount` not equal to the dispenser slots with a suffix (since 16 October 2026) | `duCount <n> but <m> dispenser suffixes set (a dispenser slot is ON only with its duNSfx)` |

A `config_report` may be edited and sent back as a setconfig: its read-only keys
(`HW_ID`, `status`, `fw`, `apnPassSet`, `mqttPassSet`, `connPending`) are accepted
and ignored. Numbers and switches may be quoted (`"duCount":"2"`, `"gpsEnable":"true"`). Field
limits: `apn` 47, `apnUser`/`apnPass` 31, `mqttHost`/`mqttPass` 63,
`mqttUser`/`mqttBaseTopic` 31 characters. `gensetType`, `gensetSlave` and
`diEnable` out of range read `gensetType must be 0..1`, `gensetSlave must be
1..247`, `diEnable must be 0..15`. A suffix that is not a 2-character string
is refused (`sfxZtech1 must be 2 digits as a string, e.g. 05`), so `"045"` never
becomes `04`; a `sfxGenset`/`gensetSlave` pair must agree (`sfxGenset 03 and
gensetSlave 4 disagree, send one of them`). Suffix/address clashes name the units:
`DU2 names dispenser 07, but a BlueSky dispenser must be 01..06`, `DU1 and DU2 have
duplicate suffix 02`, `DU4 and Ztech1 have duplicate suffix 04`, `Ztech1 and Genset
both resolve to Modbus slave 4`, `Ztech1 suffix must be exactly 2 digits (01..99)`.
Other reasons: `bad_payload`,
`du_make_invalid_use_bluesky_or_yenen`, `radar_mixed_style_use_radarType+tankCount_OR_ztechCount/pantechCount`,
`radar_incomplete_send_radarType_and_tankCount_together`,
`radar_type_invalid_use_ztech_pantech_or_none`, `empty_apn`,
`bad_mqtt_broker_fields`, `invalid_chars_in_connectivity_fields`, `nvs_save_failed`.

A successful setconfig saves and reboots; a push that changes nothing (a retained
duplicate) answers `config_successful` without rebooting. Any APN or broker field is
a trial — old values kept as fallback, device reboots, reverts with
`connectivity_rollback` if it cannot reach a broker. Change one thing at a time.

### 8.2 restart, factory_reset, getconfig

`{"cmd":"restart"}`, `{"cmd":"factory_reset"}` (wipes config, comes back
`unconfigured`), `{"cmd":"getconfig"}` (no change, no reboot). There is no
scheduled reboot on the device; if you want one, push `restart` in batches.

`config_report` keys, in order: `HW_ID`, `status`, `fw`, `duCount`,
`ztechCount`, `pantechCount`, `gensetType`, `gensetModel`, `gensetSlave`,
`gensetBaud`, `diEnable`, `gpsEnable`, `sfxGenset`, `sfxDI`, `du1Make`,
`du1Sfx` .. `du6Make`, `du6Sfx`, `sfxZtech1..4`, `sfxPantech1..4`, `apn`,
`apnUser`, `apnPassSet`, `mqttHost`, `mqttPort`, `mqttUser`, `mqttPassSet`,
`mqttBaseTopic`, `mqttTls`, `connPending`. Passwords are never echoed; an
empty suffix means the default (for a dispenser slot: OFF). `duCount` is the
number of dispenser slots ON. Use it as the equipment list for the dashboard.

### 8.3 FOTA

Per gateway on `<base>/<imei>/update`, or every GG gateway on
`<base>/fleet/gg/update`. Two payload forms:

```
https://devfota-fuelsense.s3.ap-south-1.amazonaws.com/fota/gg/alds-gg-vvm701-fw-1.0.1.bin
{"url":"https://.../alds-gg-vvm701-fw-1.0.1.bin","target":"gg","force":true}
```

HTTPS only, URL up to 511 bytes; S3 and GitHub raw both work (their CA roots are in
the image). `target` is optional; anything but `gg`/`alds-gg`/`alds-gg-vvm701` is
ignored, keeping a GGY fleet payload off GG units on the shared broker. Fleet pushes
stagger 0-29 s. Sequence: `fota_initiated` (MQTT closes), `fota_successful` (flashed,
rebooting), then `fota_completed` with the new `fw` after the first poll cycle on the
new image — the real success. Bounded 15 min, 5 tries.

Failure is `fota_failed` with `reason`: `http_<code>_err_<n>` (404 wrong URL,
403 forbidden, -1 never reached the server), `non_https_url`, `no_url`,
`url_too_long`, `url_setup_no_connect`, `url_setup_no_ok`, `get_timeout`,
`get_rejected`, `bad_content_length`, `bad_length_<n>` (image size outside
64 KB-3 MB, e.g. an error page), `ufs_short_<n>`, `file_open_failed`,
`file_open_parse`, `readfile_timeout`, `readfile_rejected`, `readfile_err_<n>`,
`read_cmd_too_long`, `read_short`, `read_corrupt_length`, `size_mismatch_<n>`
(bytes written to flash), `download_read_failed`, `flash_timeout`,
`ota_begin_failed`, `ota_write_failed`, `image_verify_failed`, `set_boot_failed`,
`no_ota_partition`, `budget_exhausted`.

Publish with retain off — a retained URL re-delivers on every reconnect. The device
drops one within 15 s of subscribing (`fota_ignored_retained`), skips a URL it
already applied (`fota_skip_already_applied`, or `fota_skip_reverted` if that image
was rolled back) or that already failed this boot (`fota_skip_known_bad`);
`"force":true` overrides all three, and a fresh file name per release avoids them.
An image that never confirms itself is rolled back and reported `fota_reverted` with
the discarded `image` name; config is untouched.

Product swap: the same board can be sent a CNG or GGY image on its own
`<imei>/update` topic and vice versa. `fota_successful` then carries
`"handover":true`, the offline queue is dropped, and the unit comes up on the
other product `unconfigured` with that product's compiled credentials.

### 8.4 Panel lock

Publish a string with `unlock` or `lock` to `<base>/<imei>/lock`, retain off.
Every ack/event replies on `<base>/<imei>/lock/out` (never `<base>/data` or the
command topic): `unlock` opens the box 8 s then self-relocks
(`{"event":"unlocked","window_ms":8000}`), `lock` closes it now (`locked`), a bad
payload is `bad_command`, an unlock within 5 s of subscribing is `ignored_retained`,
and a `safety_restart` next boot means an unlock window never closed (deadman
forced a restart). `unlocked` = command reached the relay (bolt not sensed); every reboot re-locks.

### 8.5 rsp / kfactor tunnel

The cloud builds every dispenser command; the gateway only carries it.

| Publish to | Dispensers |
|---|---|
| `<base>/rsp/imei/<hw_id>/in` | BlueSky and Yenen |
| `<base>/kfactor/imei/<hw_id>/in` | BlueSky only (Yenen has no k-factor) |

The payload is the frame as hex, retain off. These all send the same bytes:

```
f5,02,a4,68,80,b2,09        F5 02 A4 68 80 B2 09        F502A46880B209
0xF5,0x02,0xA4,...          f5,2,a4,68,80,b2,9          "f5,02,a4,68,80,b2,09"
{"Status":"lock","NZ1":"02,30,38,30,31,30,4B,36,35,33,32,03,37,31"}
```

The last is the pilot's wrapper: the first `NZ<n>` string is the frame, other
keys are ignored. Anything else (a letter that is not hex, an odd digit run, a
JSON object without `NZ<n>`) is refused: nothing is sent, `404`.

The gateway sets the line (BlueSky 9600 8E1, Yenen 9600 8N1), sends the frame
once and reads until the line has been quiet for 2 s. The same topic with `out`
for `in` then gets the reply exactly as received, lowercase comma hex — an ack
(`f5,02,a3,59,b2,3f`, `06,30,31`), a NAK (`15`), a damaged frame, whatever came.
It gets `404` when the payload is not a frame of 1-64 bytes, 16 commands are
already waiting, the bus stayed busy for 9 s, nothing came back within 2 s, more
than 128 bytes came, or the line was still talking after 6 s. Nothing is
checked, retried or read back, and nothing goes to `<base>/logs`.

Commands run one at a time in arrival order, ~2 s each, so replies arrive in
the order the commands were sent. There is no request id: to pair a reply with
its command, keep one command in flight per dispenser.

Safety is the cloud's job now. A write is sent even mid-sale, so read the nozzle
first and write only when it is idle:

- BlueSky status: `F5 <nz> A2 D5 <cs>` (`cs = nz ^ D5 ^ D7`, e.g. `f5,01,a2,d5,03`);
  idle when the 4th reply byte is `80` (or `00`).
- Yenen: Command 2 (appendix); idle when the status field is `00`.

Then confirm the new value from telemetry: `rsp` (BlueSky) or `data[6:10]`
(Yenen) on the next message. After any BlueSky tunnel command the gateway
re-reads the k-factor on the next round, so `kfact` shows a change within one
round instead of 15 min.

## 9. Operating notes

- There is no offline message. Treat ~10 min of silence as offline (allowing for
  reboots and FOTA); `diagnostics.link.connected_s` is the current session age.
- Commands have no request id: one per device at a time, wait for its terminal
  status. Retain off on every command — a retained `restart` is a reboot loop, a
  retained `unlock` opens the box on reconnect.
- A gateway with a dead SIM or no coverage keeps polling and buffering; it restarts
  for network reasons at most 6 times in a row, then stays up.
- Broker outage: the device stays on the network, buffers to flash and probes the
  broker every 30 s, so the fleet drains within minutes of its return.
- TLS roots in the image: ISRG Root X1, X2, YE, YR and Amazon Root CA 1. A
  broker or FOTA host on another CA fails to connect.

## Appendix: Yenen frame decode

RS485 ASCII, 9600 8N1. A nozzle is head (1-5) plus side: side A `'0'+(head-1)`,
side B `'5'+(head-1)`. `data` is the raw Command 2 reply; drop the leading STX and
the trailing ETX + 2 checksum bytes, then read 47 ASCII characters:

| Offset | Width | Field |
|---|---|---|
| 0 | 2 | reply code, `49` |
| 2 | 3 | address echo |
| 5 | 1 | command echo, `A` |
| 6 | 4 | unit price, /100 |
| 10 | 4 | preset amount |
| 14 | 4 | preset volume |
| 18 | 3 | delivery litres, /100 |
| 21 | 6 | delivery amount, /100 |
| 27 | 6 | totalizer litres, lifetime (add 1,000,000 to match the pump display) |
| 33 | 8 | totalizer amount, lifetime |
| 41 | 2 | status: `00` idle, anything else dispensing |
| 43 | 4 | trailing, opaque |

Example `49010A67720260000035902431187433322182451110990`: head 1 side A, price
67.72, 3.59 L for 243.11, totalizer 874,333 L, status `11` so mid-sale. Record only
from `00` frames; derive sales by differencing totalizers. The device verifies the
checksum and address echo; anything else is `404`.

RSP write (Command K) — built by the cloud, sent through the rsp tunnel (8.5):
`02 30 38 30 <'0'+head> <side code> 4B <price as 4 digits, rupees x100> 03 <ck1> <ck2>`,
the checksum being the XOR of STX..ETX with each nibble + `0x30` (head 1 side A,
65.32: `02,30,38,30,31,30,4b,36,35,33,32,03,37,31`). The pump answers `06,30,31`
(accepted) or `15` (refused). The ack covers the frame, not the stored price —
confirm from the next `data`.
