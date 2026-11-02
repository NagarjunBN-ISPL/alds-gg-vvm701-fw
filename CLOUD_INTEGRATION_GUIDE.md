# ALDS GG gateway (VVM701): cloud integration guide

Firmware 1.0.0, product id `alds-gg-vvm701/1`. Updated 21 October 2026.

What the gateway sends, what it accepts, and how to configure it. Payloads are
taken from the firmware source; if the two ever disagree, the firmware is right.

## 1. Connection and topics

| | |
|---|---|
| Broker | `mqtt.iotronsys.com:8777`, TLS, user `gogas`, base topic `gogas` (same as alds-ggy) |
| Client id | `iotronics-<imei>` |
| MQTT | 3.1.1, QoS 1, persistent session, keepalive 60 s |
| Retain | never used by the gateway. The cloud must not set it either |

The broker settings are compiled into the firmware and re-applied on every
boot, so a reflashed unit always lands on the right broker. GG and GGY share
the broker and base topic; tell them apart by `diagnostics.product`.

`<base>` is the base topic, `<imei>` the 15-digit modem IMEI, `<hw_id>` the
IMEI plus a 2-digit suffix (2.3).

| Topic | Direction | What |
|---|---|---|
| `<base>/data` | gateway to cloud | readings only: nozzles, tanks, genset, DI, GPS |
| `<base>/logs` | gateway to cloud | every `status` message, `config_report`, diagnostics |
| `<base>/<imei>/setconfig` | cloud to gateway | configuration and commands (section 2) |
| `<base>/<imei>/update` | cloud to gateway | FOTA for one gateway |
| `<base>/fleet/gg/update` | cloud to gateway | FOTA for every GG gateway |
| `<base>/<imei>/lock`, `/lock/out` | both | panel box lock and its replies |
| `<base>/rsp/imei/<hw_id>/in`, `/out` | both | dispenser command tunnel, BlueSky and Yenen |
| `<base>/kfactor/imei/<hw_id>/in`, `/out` | both | same tunnel, BlueSky only |

Subscribe to both `/data` and `/logs`. Besides `HW_ID` and `TS`, each message
has one top-level key that says what it is: `nozzles`, `tanks`, `genset`,
`di`, `gps`, `diagnostics`, `status` or `event`.

## 2. Configuring a gateway

### 2.1 How it works

Publish one JSON object to `<base>/<imei>/setconfig`, retain off. Only the keys
you send change; the rest stay as stored. The gateway answers on `<base>/logs`:

```
{"HW_ID":"860710081332556","status":"config_received"}
{"HW_ID":"860710081332556","status":"config_successful","fw":"1.0.0"}
```

It then saves, reboots and, once back online, announces
`{"status":"ready","fw":"1.0.0"}`. A wrong push gets `config_failed` with a
`reason` (2.10), for example
`{"status":"config_failed","reason":"DU1 and DU2 have duplicate suffix 02"}`.
Nothing from a refused push is applied and the gateway carries on with its old
settings, no reboot. Values are never trimmed, rounded or swapped for a
default. A push identical to what is stored answers `config_successful`
without a reboot.

A new gateway has nothing switched on. It publishes
`{"status":"unconfigured","fw":"1.0.0"}` at boot and every 3 minutes until it
gets a setconfig.

### 2.2 A complete example

Three BlueSky dispensers, two Ztech tanks, a Mahindra GC1211 genset at slave 10
and two digital inputs:

```json
{"du1Make":"bluesky","du1Sfx":"01","du2Make":"bluesky","du2Sfx":"02",
 "du3Make":"bluesky","du3Sfx":"03","du4Sfx":"","du5Sfx":"","du6Sfx":"","duCount":3,
 "radarType":"ztech","tankCount":2,
 "gensetType":1,"gensetModel":0,"gensetSlave":10,"gensetBaud":9600,
 "diEnable":3,"gpsEnable":false}
```

The dispensers publish as `<imei>01`..`<imei>03`, the tanks as `<imei>04` and
`<imei>05`, the genset as `<imei>10`, the inputs as `<imei>11`.

### 2.3 Suffixes

Every unit publishes under `HW_ID` = IMEI + a 2-digit suffix. In this product
the suffix is also the unit's bus address, so the number you set is the device
that gets polled. Send it as a 2-digit string (`"04"`, not `4` or `"4"`),
`01`..`99`. No two units on a gateway may share a suffix, and no two Modbus
units (tanks, genset) may share a slave; the gateway refuses the push and
names both units.

| Unit | Key | If left `""` |
|---|---|---|
| Dispenser slot 1-6 | `du1Sfx`..`du6Sfx` | slot is OFF |
| Ztech tank 1-4 | `sfxZtech1`..`sfxZtech4` | `04`, `05`, `06`, `07` |
| Pantech tank 1-4 | `sfxPantech1`..`sfxPantech4` | carries on after the last Ztech tank |
| Genset | `gensetSlave` or `sfxGenset` | `10` |
| Digital inputs | `sfxDI` | `11` (a label only, not a bus address) |

Status, diagnostics, GPS and lock messages use the bare IMEI. Dispenser `04`
and up clashes with the default Ztech numbers; in that case give the tanks
their real slave ids (`sfxZtech1`...).

### 2.4 Dispensers

Six slots. A slot is ON only when its `du<N>Sfx` is set, and the suffix is the
dispenser number on the bus.

| Key | Values |
|---|---|
| `du<N>Make` | `bluesky` (default) or `yenen` |
| `du<N>Sfx` | `"01"`..`"06"` (Yenen `"01"`..`"05"`), or `""` for OFF |
| `duCount` | must equal the number of slots ON. It is a check, not a switch |

BlueSky dispenser N has nozzle addresses 2N-1 and 2N; Yenen dispenser N is head
N. Telemetry nozzle labels follow the same rule, so dispenser `03` publishes
nozzles `"5"` and `"6"`. BlueSky and Yenen can be mixed on one gateway.

Taking dispenser 1 out of service at a 3-dispenser site:

```json
{"duCount":2,"du1Sfx":"","du2Sfx":"02","du3Sfx":"03"}
```

Dispensers 2 and 3 keep publishing as `<imei>02` and `<imei>03`; nothing
changes at the pumps. `{"du1Sfx":"01","duCount":3}` puts it back.

### 2.5 Radar tanks

Usually `radarType` (`ztech`, `pantech` or `none`) with `tankCount` (0-4),
always sent together; picking one vendor turns the other off. A site with both
kinds sends `ztechCount` and `pantechCount` (0-4 each) instead. Never mix the
two styles in one push. Per-tank suffixes are the Modbus slave ids; leave them
`""` for the defaults in 2.3. `{"radarType":"none","tankCount":0}` turns radar off.

### 2.6 Genset, digital inputs, GPS

| Key | Values |
|---|---|
| `gensetType` | `0` off (default), `1` on |
| `gensetModel` | `0` GC1211 (default), `1` GC1111, `2` SGC 120 Mk II, `3`/`4`/`5` Ashok Leyland GC1201/GC1202/GC1203, `6` Sedemac GC902 (not yet checked on hardware) |
| `gensetSlave` | Modbus slave 1-247, default `10` |
| `sfxGenset` | the same number as a 2-digit string. Send this or `gensetSlave`; if both, they must agree |
| `gensetBaud` | `4800`, `9600` (default), `19200` |
| `diEnable` | bitmask 0-15: 1 = DI1, 2 = DI2, 4 = DI3, 8 = DI4 (`3` = DI1 + DI2, `0` = off) |
| `sfxDI` | `"01"`..`"99"`, default `"11"` |
| `gpsEnable` | `true` / `false` (default). Off means no GPS messages |

### 2.7 Network and broker

Leave these alone normally; the firmware carries the production values.
`apn` (1-47 characters), `apnUser`, `apnPass` (up to 31), `mqttHost` (1-63),
`mqttPass` (up to 63), `mqttUser`, `mqttBaseTopic` (up to 31; base topic
non-empty, no `#` or `+`), `mqttPort` (1-65535), `mqttTls` (`true`/`false`).
No quotes, backslashes or control characters.

Any change here is a trial. The gateway reboots onto the new values; if it
cannot reach a broker with them it goes back to the old ones and reports
`{"status":"connectivity_rollback","reason":"new_settings_unreachable"}`.
Change one thing at a time.

### 2.8 Reading the configuration back

`{"cmd":"getconfig"}` publishes the live settings on `/logs`. Nothing changes,
no reboot.

```json
{"HW_ID":"860710081332556","status":"config_report","fw":"1.0.0",
 "duCount":3,"ztechCount":2,"pantechCount":0,"gensetType":1,"gensetModel":0,
 "gensetSlave":10,"gensetBaud":9600,"diEnable":3,"gpsEnable":false,
 "sfxGenset":"10","sfxDI":"","du1Make":"bluesky","du1Sfx":"01", ... ,"du6Sfx":"",
 "sfxZtech1":"", ... ,"sfxPantech4":"","apn":"AIRTELMETERV4","apnUser":"",
 "apnPassSet":false,"mqttHost":"mqtt.iotronsys.com","mqttPort":8777,"mqttUser":"gogas",
 "mqttPassSet":true,"mqttBaseTopic":"gogas","mqttTls":true,"connPending":false}
```

Every key from 2.4-2.7 is present. Passwords are never sent, only
`apnPassSet`/`mqttPassSet`. An empty tank, genset or DI suffix means the
default. `connPending:true` means a network change is still on trial. A report
can be edited and sent back as a setconfig: its read-only keys (`HW_ID`,
`status`, `fw`, `*PassSet`, `connPending`) are ignored, and leaving out
`mqttPass`/`apnPass` keeps the stored passwords.

### 2.9 Commands

| Send | Reply on `/logs` | Effect |
|---|---|---|
| `{"cmd":"getconfig"}` | `config_report` | section 2.8 |
| `{"cmd":"diagnostics"}` | the diagnostics report | section 4.2, sent now instead of at the end of its 15-min window |
| `{"cmd":"restart"}` | `restarting` | reboots, nothing erased |
| `{"cmd":"erase_config"}` | `erasing_config` | erases the station's equipment configuration, reboots `unconfigured` |
| `{"cmd":"factory_reset","confirm":"<IMEI>"}` | `factory_resetting` | erases everything the gateway has stored, reboots `unconfigured` like a freshly flashed unit |

Send a `cmd` on its own, never with settings. A `restart`, `erase_config` or
`factory_reset` arriving within 15 s of the gateway connecting is taken as a
retained leftover and dropped with `cmd_ignored_retained`; send it again after
15 s.

**Giving a station a new configuration: `erase_config`, then setconfig.**

1. Send `{"cmd":"erase_config"}`. The reply is `erasing_config`, then the
   gateway reboots.
2. Wait for `unconfigured` (about 1-2 min) before sending anything else: a
   push sent while the gateway restarts can be lost.
3. Send the station's setconfig (2.1). Everything starts OFF, GPS included, so
   send every piece of equipment the station has.

What `erase_config` erases: dispenser slots, radar tanks, genset, digital
inputs, GPS and every suffix. What it keeps: the APN and broker (the gateway
comes back where it was), the unsent readings (still delivered after the
reboot), the FOTA memory and the boot counter.

**`factory_reset`** erases the configuration, the unsent readings, the FOTA
memory and the boot counter. The installed firmware stays. The gateway comes
back on the APN and broker built into its firmware with `unconfigured` and
`firmware_credentials_applied` (`broker_changed:false`), like a freshly flashed
unit. Nothing it erases can be recovered, so two checks come first:

- `confirm` must be this gateway's IMEI, or the reset is refused with
  `confirm_required` / `confirm_mismatch`.
- With readings still waiting in the outage queue (`records.q_flash` in 4.2)
  it is refused with `unsent_readings_<n>`. Wait for the queue to empty, or
  add `"force":true` to throw them away.

The reply says how many readings went: `{"status":"factory_resetting","discarded":<n>}`.
If the queue cannot be emptied the reset stops with `store_purge_failed` and
nothing is erased. `confirm` and `force` are accepted with `factory_reset`
only; anywhere else, a misspelt `cmd` included, they are refused as
`unknown_key_confirm` / `unknown_key_force`.

### 2.10 Why a push was refused

Show the `reason` to the operator as it is.

| You sent | `reason` |
|---|---|
| Broken or cut-short JSON, a bare URL, a space before a `:` | `bad_payload` |
| A key the firmware does not know (`du1make`), or one key twice | `unknown_key_<key>`, `duplicate_key_<key>` |
| A number out of range or not whole (`9`, `-1`, `2.5`) | `<key> must be <lo>..<hi>` or `<key> must be a whole number <lo>..<hi>` |
| A switch that is not true/false/1/0 | `<key> must be true or false` |
| A suffix that is not a 2-character string (`4`, `"045"`) | `<key> must be 2 digits as a string, e.g. 05` |
| A suffix with letters, or `00` | `Ztech1 suffix must be exactly 2 digits (01..99)` |
| A dispenser number out of range | `DU2 names dispenser 07, but a BlueSky dispenser must be 01..06` |
| Two units with one suffix | `DU4 and Ztech1 have duplicate suffix 04` |
| Two Modbus units on one slave | `Ztech1 and Genset both resolve to Modbus slave 4` |
| `sfxGenset` and `gensetSlave` disagree | `sfxGenset 03 and gensetSlave 4 disagree, send one of them` |
| `duCount` not equal to the slots ON | `duCount 3 but 2 dispenser suffixes set (a dispenser slot is ON only with its duNSfx)` |
| Both radar styles, or `radarType` without `tankCount` | `radar_mixed_style_...`, `radar_incomplete_...` |
| `cmd` mixed with settings, or an unknown `cmd` | `cmd_must_be_sent_alone`, `unknown_cmd` |
| `factory_reset` without this gateway's IMEI, or with readings still queued | `confirm_required`, `confirm_mismatch`, `unsent_readings_<n>` (`store_purge_failed`: the queue could not be emptied, nothing erased) |

Others name the key or the problem directly (`du_make_invalid_use_bluesky_or_yenen`,
`gensetBaud must be 4800, 9600 or 19200`, `<key> is longer than <n> characters`,
`empty_apn`, `bad_mqtt_broker_fields`, `nvs_save_failed`...). Numbers and
switches may be quoted (`"duCount":"2"`, `"gpsEnable":"true"`).

### 2.11 Building the configuration page

What the dashboard's configuration page should do:

1. **Load.** Send `{"cmd":"getconfig"}` and fill the form from the
   `config_report`. The gateway is the truth, not your database.
2. **Dispensers.** Six rows, each with ON/OFF, make (BlueSky / Yenen) and
   dispenser number (01-06, Yenen 01-05). A row that is OFF sends
   `du<N>Sfx:""`. Work out `duCount` from the rows ON; don't let the user type it.
3. **Tanks.** Vendor (Ztech / Pantech / none) and tank count 0-4. Show each
   tank's slave (default 04, 05...) and allow changing it. Offer the
   "both vendors" form only for a site that really has both.
4. **Genset.** ON/OFF, model dropdown with the names from 2.6, slave (default
   10), baud 4800 / 9600 / 19200.
5. **Digital inputs.** Four checkboxes, sent as the `diEnable` bitmask.
6. **GPS.** One switch.
7. **Check before sending:** every suffix 2 digits, no two equal, no two
   Modbus units on one slave, `duCount` = rows ON. This gives the operator a
   quick error; the gateway checks again anyway.
8. **Send the whole equipment set** every time: all six `du<N>Make` and
   `du<N>Sfx`, `duCount`, the radar keys, the genset keys, `diEnable`,
   `gpsEnable`. Then nothing stale left on the gateway can slip through.
   Suffixes as strings, retain off.
9. **Keep network and broker settings on a separate, admin-only screen**, never
   in the same push as the equipment.
10. **Show the result.** Wait for `config_received`, then `config_successful`
    or `config_failed`, and show the `reason` word for word. After success
    wait for `ready`, send `getconfig` again and show what the gateway now has.
11. **One push at a time per gateway.** There is no request id; wait for the
    final status before sending another.

## 3. Telemetry on `<base>/data`

### 3.1 Rules for every message

- One unit per message. `TS` is `YYYY-MM-DD HH:MM:SS` UTC from the gateway
  clock; nothing is ever sent dated 1970.
- Hex is lowercase, comma separated, and is the whole frame the device sent,
  already checked by the gateway (Modbus: slave, function, byte count, data, CRC).
- `"404"` in a field means that read failed; a unit that did not answer at all
  sends `"data":"404"`. No message is over 1400 bytes.
- Dispenser readings taken during an outage are kept (up to 768 KB on flash)
  and sent oldest first when the link is back, so expect bursts of old `TS`.
  Delivery is at least once: dedupe on `HW_ID` + `TS` + `data`. Tanks, genset,
  DI and GPS are not kept.

### 3.2 BlueSky nozzle, every 20 s

```json
{"HW_ID":"86071008133255601",
 "nozzles":{"1":{"data":"f5,01,ac,00,02,15,18,26,23,30,19,35,34,c5,3f",
                 "rsp":"f5,01,a4,68,86,b6,08","kfact":"f5,01,a4,23,60,e2,71","rfid":"404"}},
 "TS":"2026-09-21 12:55:06"}
```

`data` is the totalizer, `rsp` the unit price, `kfact` the k-factor (read every
15 min, repeated in between). `rfid` is always `"404"`; it is not read. Nozzles
are read only when idle, so every `data` is a settled totalizer. A busy or
absent nozzle sends `{"nozzles":{"1":{"data":"404"}}}`.

### 3.3 Yenen nozzle, every 10 s

`{"HW_ID":"86071008133255603","nozzles":{"5":{"data":"02,34,39,30,32,30,..."}},"TS":"..."}`

`data` is the Command 2 reply; decode in the appendix. Use only frames with
status `00` (idle).

### 3.4 Radar tank, every 10 min

`{"HW_ID":"86071008133255604","tanks":{"1":{"data":"04,03,04,2f,13,00,00,56,22"}},"TS":"..."}`

The raw Modbus reply. Each vendor numbers its tanks from `"1"`; the `HW_ID`
tells them apart. A failed tank is retried 60 s later, up to 3 times.

### 3.5 Genset, every 5 min

Up to three messages per poll, same `TS`, in this order:

```json
{"HW_ID":"86071008133255610","genset":{"registers":"0a,04,48,01,3f,00,3e,...,5e,2c"},"TS":"..."}
{"HW_ID":"86071008133255610","genset":{"Alarms":"0a,04,06,11,10,11,11,11,11,48,44"},"TS":"..."}
{"HW_ID":"86071008133255610","genset":{"status":"0a,04,04,c0,00,00,00,7d,44"},"TS":"..."}
```

Register k is `frame[3+2k] << 8 | frame[3+2k+1]`. `Alarms` has a capital A, as
in the DG spec. A failed read keeps its key with `"404"`.

| Model | Function | `registers` | `Alarms` | `status` |
|---|---|---|---|---|
| GC1211 / GC1111 | 04 / 03 | 1-36 | 57-59 | 67-68 |
| SGC 120 | 03 | 1-78 (alarms and status inside) | none | none |
| Ashok Leyland, GC902 | 04 | 1-36 | none | 67 |

Register maps and scaling: `GENSET_REGISTER_MAPS.md` in this repo.

### 3.6 Digital inputs (on change) and GPS (every 5 min, only with `gpsEnable`)

```json
{"HW_ID":"86071008133255611","di":{"DI1":0,"DI2":1},"TS":"..."}
{"HW_ID":"860710081332556","gps":{"lat":12.887360,"lon":77.641840,"hdop":5.9,"nsat":8,"spd":0.0},"TS":"..."}
```

Inputs are checked every 2 s and sent only on a change, so silence is normal;
`TS` is when the change happened and an unreadable input is the number `404`.
For GPS, `lat`/`lon` is the fix; `hdop`, `nsat` and `spd` (km/h) may be
missing. No fix is `{"gps":{"data":"404"}}`.

## 4. Status and diagnostics on `<base>/logs`

### 4.1 Status messages

All carry the bare IMEI as `HW_ID`.

| `status` | Meaning |
|---|---|
| `ready` / `unconfigured` | booted with / without equipment configured (`unconfigured` repeats every 3 min) |
| `config_*`, `restarting`, `erasing_config`, `factory_resetting`, `cmd_ignored_retained` | section 2 |
| `connectivity_rollback` | a network change failed and was undone |
| `firmware_credentials_applied` | the compiled broker settings replaced the stored ones. `config_erased:true` means the unit held another product's config and is now `unconfigured` |
| `fota_*` | section 5.1 |

### 4.2 Diagnostics, every 15 min

`{"cmd":"diagnostics"}` (2.9) sends one now. It closes the window like the
periodic one, so the next periodic report comes 15 min after it.

```json
{"HW_ID":"860710081332556","diagnostics":{
  "health":"ok","why":[],
  "fw":"1.0.0","product":"alds-gg-vvm701/1","cred_fp":"3a91c2e7",
  "uptime_s":86412,"boot_count":14,"reset_reason":1,
  "records":{"q_ram":0,"q_flash":0,"q_bytes":0,"q_pct":0,"spilled":0,"dropped":0,"trimmed":0,"store_ok":true,"rtc_synced":true},
  "link":{"rssi":22,"operator":"Airtel","connected_s":86300,"recov_l1":0,"recov_l2":0,"recov_l3":0,"bearer_distrust":0,"pub_ok":1450,"pub_fail":0},
  "bus":{"nozzles_absent":[],"lock_timeouts":0,"cycle_max_ms":4210,"cycle_avg_ms":3100,"cycles":90},
  "sys":{"heap_free":248000,"heap_min_ever":236000,"stack_min":2400,"temp_c":46.5,"vbat_mv":3820,
         "cpu":{"window_s":900,"core0_avg":3,"core1_avg":1,"core0_peak":9,"core1_peak":2}},
  "subsystems":{"du":true,"radar":true,"genset":false,"di":true,"gps":false}},
 "TS":"2026-09-21 12:55:06"}
```

- `health` is `ok`, `degraded` or `fault`, with reasons in `why`. Alert on
  `fault` (`store_unmounted`, `records_dropped`, `records_trimmed`,
  `last_stop_fault`); show `degraded` (`clock_unsynced`, `nozzle_absent`,
  `queue_filling`, `link_rebuilt`, `publish_failures`, `temp_high`, `link_unstable`).
- `records.dropped` or `trimmed` above 0 means readings were lost. `q_pct` is
  how full the outage queue is. `bus.nozzles_absent` lists nozzles not answering.
- `link.rssi` is CSQ 0-31 (99 unknown); dBm = -113 + 2 x rssi.
- Once after a reboot a `last_stop` block says what the gateway was doing when
  it stopped; `fault:true` means crash, watchdog or brownout. `reset_reason`:
  1 power-on, 3 software restart, 4 crash, 5/6/7 watchdog, 9 brownout.
  `boot_count` climbing with a small `uptime_s` is a reboot loop.

## 5. FOTA, panel lock, rsp/kfactor tunnel

### 5.1 FOTA

To one gateway on `<base>/<imei>/update`, or to all GG gateways on
`<base>/fleet/gg/update`. Retain off. Either form works:

```
https://bucket.s3.ap-south-1.amazonaws.com/fota/gg/alds-gg-vvm701-fw-1.0.1.bin
{"url":"https://.../alds-gg-vvm701-fw-1.0.1.bin","target":"gg","force":true}
{"url":"https://.../alds-ggy-vvm701-fw-1.0.0.bin","handover":true}
```

HTTPS only (`https://` in any case), URL up to 1535 characters; units on
firmware older than this guide take 511 at most, so keep links short until the
fleet is updated (an S3 link presigned with an IAM-user key is ~360). S3
(plain or presigned) and GitHub raw links work; the link must answer directly
(no redirect) and send `Content-Length`. `HW_ID` is optional, but when present
it must be that unit's IMEI (or IMEI + suffix): a push naming another unit is
refused (`wrong_hw_id` on the unit's own topic, silently on the fleet topic),
so leave it out of a push meant for the whole fleet. Other keys (`file_name`,
...) are ignored. `\u0026`-style escapes in the link are
decoded, and a space in it is sent as `%20` (send `%20` anyway). `target` is
optional; anything other than `gg`, `alds-gg`, `alds-gg-vvm701` or
`alds-gg-vvm701-fw` is refused, which keeps a GGY push off GG units (answered
`wrong_target` on the unit's own topic, silently on the fleet topic). Fleet
pushes are spread over 30 s.

Another product's firmware (a deliberate swap to CNG, GGY, ...) is installed
only when the push says `"handover":true` (`true`, `1` or `"true"`); without it
that file is downloaded, refused as `not_this_product`, and nothing changes. A
bare URL never swaps. `"force"` takes the same values.

Replies on `/logs`. A push gets exactly one final answer, and its `status` is
`fota_successful` or `fota_failed`, so stop waiting at the first of those two;
`reason` says why. `fota_initiated` (MQTT closes while it downloads) is the
only progress message.

| `status` | `reason` | Meaning |
|---|---|---|
| `fota_initiated` | | download started; the unit is off MQTT for about 2-5 min |
| `fota_successful` | `update_confirmed` | the new image runs and has confirmed itself (`fw` = its version, `build` = its build id); sent by the new image soon after its `ready` |
| `fota_successful` | `fota_skip_already_applied` | this exact URL is already installed (`fw`, `build` of the running image) |
| `fota_successful` | `handover` | another product's firmware was flashed (`"handover":true`); the unit comes up on that product's topics, `unconfigured` |
| `fota_failed` | `fota_reverted` | the new image never confirmed itself and the bootloader went back to the old one (`image` = the one discarded) |
| `fota_failed` | `fota_skip_reverted` / `fota_skip_known_bad` | this exact URL was rolled back before / already failed since the last reboot |
| `fota_failed` | `fota_ignored_retained` | arrived within 15 s of the unit (re)connecting, so treated as retained: push again, retain off. After a `ready`, wait 15 s before pushing |
| `fota_failed` | `wrong_target` | `target` names another product |
| `fota_failed` | `wrong_hw_id` | `HW_ID` names another unit |
| `fota_failed` | `not_this_product` | the file is another product's firmware and the push did not say `"handover":true`; nothing changed |
| `fota_failed` | `http_403_err_0` / `http_404_err_<n>` / `bad_length_<n>` / `ufs_full_<free>_of_<need>` | link expired or refused / no such file / not a firmware file / the image does not fit the modem's storage (about 555 KB on some units): answered after the first try, no retries |
| `fota_failed` | e.g. `http_-1_err_<n>` (server never reached), `no_url`, `url_too_long`, `non_https_url`, `tls_setup_failed`, `readfile_timeout`, `budget_exhausted` | the download or install failed; up to 5 tries within 15 min |

`"force":true` overrides the three `fota_skip_*` refusals. Give every build its
own file name. No answer at all means the push never reached the unit
(offline, or another product's `target` or another unit's `HW_ID` on the fleet
topic): time out after about 2 min without `fota_initiated` and about 40 min
without a final answer.
FOTA never touches the configuration, except a handover to the CNG or GGY
firmware (or back), which comes up as the other product, `unconfigured`.

### 5.2 Panel lock

Publish a string containing `unlock` or `lock` to `<base>/<imei>/lock`, retain
off. Replies on `<base>/<imei>/lock/out`:

- `{"event":"unlocked","window_ms":8000}`: box open for 8 s, then locks itself
- `{"event":"locked"}`: locked now
- `{"event":"bad_command"}`: the payload had neither word
- `{"event":"ignored_retained"}`: unlock within 5 s of connecting, dropped
- `{"event":"safety_restart"}`: next boot after an unlock that never closed

`unlocked` means the relay got the command; the bolt is not sensed. Every reboot locks the box.

### 5.3 rsp / kfactor tunnel

The cloud builds the dispenser command (price write, k-factor write, any read).
The gateway sends it to the pump once and returns whatever comes back. Use
`<base>/rsp/imei/<hw_id>/in` for BlueSky and Yenen, and
`<base>/kfactor/imei/<hw_id>/in` for BlueSky only. `<hw_id>` is the
dispenser's, e.g. `86071008133255602`. The payload is the frame in hex, in any
of these forms:

```
f5,02,a4,68,80,b2,09     F5 02 A4 68 80 B2 09     F502A46880B209     0xF5,0x02,...
{"Status":"lock","NZ1":"02,30,38,30,31,30,4B,36,35,33,32,03,37,31"}
```

The last is the pilot format; the first `NZ<n>` value is the frame. The reply
goes to the same topic with `/out` in place of `/in`, in lowercase
comma hex, exactly as the pump sent it: an ack (`f5,02,a3,59,b2,3f`,
`06,30,31`), a NAK (`15`), anything. It is `404` when the payload is not valid
hex or is over 64 bytes, 16 commands are already waiting, the bus stayed busy
for 9 s, the pump did not answer within 2 s, or the reply ran past 128 bytes
or 6 s. Commands run one at a time in the order received, about 2 s each.
There is no request id, so keep one command in flight per dispenser.

The gateway does not check, retry or read back, and it will send a write even
during a sale. Read the nozzle status through the tunnel first and write only
when idle:

- BlueSky: `F5 <nz> A2 D5 <cs>` with `cs = nz ^ D5 ^ D7` (nozzle 1:
  `f5,01,a2,d5,03`). Idle when the 4th reply byte is `80` or `00`.
- Yenen: Command 2 (appendix). Idle when status is `00`.

Confirm the new price from the next telemetry: `rsp` for BlueSky, characters
6-9 of `data` for Yenen. After a BlueSky tunnel command the k-factor is re-read
on the next round.

## 6. Operating notes

- There is no "offline" message. Treat about 10 minutes of silence as offline.
- Never publish a command retained. A retained `restart` loops reboots; a
  retained `unlock` opens the box on every reconnect.
- With no SIM or coverage the gateway keeps polling and storing. It restarts
  for network reasons at most 6 times in a row, then stays up.
  During a broker outage it retries every 30 s.
- Trusted TLS roots: ISRG Root X1, X2, YE, YR and Amazon Root CA 1. A broker or
  FOTA server on another CA will not connect.

## 7. Coming from the old GG firmware

Telemetry topic, envelope and payload shapes are unchanged. Different now:
status and diagnostics are on `/logs`; suffixes are digits only and
dispensers use `du<N>Make`/`du<N>Sfx`/`duCount` (not `duBlueskyCount`/`sfxBs<N>`);
a bad setconfig is refused, not quietly changed; genset frames are whole
(register k starts at byte 3) and Mahindra adds `Alarms`; price and k-factor
writes go through the tunnel (no `setrsp`); outage readings replay from flash.

## Appendix: Yenen frame decode

RS485 ASCII, 9600 8N1. Nozzle code: side A is `'0' + (head-1)`, side B is
`'5' + (head-1)`. `data` is the raw Command 2 reply. Drop the leading STX and
the trailing ETX and 2 checksum bytes, then read 47 ASCII characters:

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
| 27 | 6 | totalizer litres (add 1,000,000 to match the pump display) |
| 33 | 8 | totalizer amount |
| 41 | 2 | status: `00` idle, anything else dispensing |
| 43 | 4 | not used |

Example `49010A67720260000035902431187433322182451110990`: head 1 side A, price
67.72, 3.59 L for 243.11, totalizer 874,333 L, status `11` (mid-sale, don't
record). Work out sales from the difference between idle totalizers.

Price write (Command K), built by the cloud and sent through the rsp tunnel:
`02 30 38 30 <'0'+head> <side code> 4B <price, 4 ASCII digits, rupees x 100> 03 <ck1> <ck2>`.
The checksum is the XOR of STX through ETX, each nibble plus `0x30`. Head 1
side A at 65.32 is `02,30,38,30,31,30,4b,36,35,33,32,03,37,31`. The pump
answers `06,30,31` (accepted) or `15` (refused); confirm from the next `data`.
