# Release process

One procedure, followed every time. The point is not ceremony — it is that a
gateway sealed in a panel box reports `fw:"1.0.0"` for years, and somebody will
eventually need to know exactly what source and exactly what bytes that was.

This mirrors the process proven on `cng-cpil-vvm701-fw` — the two products share
a flash map and are swapped between stations by FOTA, so they are released the
same way on purpose.

---

## 1. The standard: always flash the four separate files

**There is no merged image.** The release build deliberately does not produce
one — see "Why one method" below.

| Offset | File |
|---|---|
| `0x0` | `bootloader.bin` |
| `0xc000` | `partition-table.bin` |
| `0x12000` | `ota_data_initial.bin` |
| `0x20000` | `alds-gg-vvm701-fw-<version>.bin` |

### Flash settings: always "keep"

**Leave flash mode, frequency and size at the tool's default / "keep".** If the
flasher forces a choice, this release is DIO / 80 MHz / 16 MB — but prefer not
to choose.

Those values are already inside `bootloader.bin`, stamped into the image header
by the build (`E9 ...` at offset 0). The flasher settings do not supply them,
they OVERWRITE them. "Keep" writes what the build put there, which is correct
for whatever board that build targeted.

A rule that is right because of a fact about one board (16 MB, DIO) goes stale
the first time a differently-wired part ships. "Use what the build put there"
does not.

### Why one method rather than "merged for fresh, separate for commissioned"

The merged file spans `0x0`–`0x8b…`, and `merge_bin` fills the gaps between
images with `0xFF`. The nvs partition sits at `0xd000`, inside that span — so
flashing the merged image **erases the station plan and broker credentials**,
whatever the "erase flash" checkbox says. On a fresh board that costs nothing.
On a commissioned one it costs a site revisit.

Two procedures means someone eventually applies the wrong one, and the failure
is silent until a technician is standing at a forecourt. The four files are
safe on a board in *any* state, so that is the only procedure we publish.

The cost is four offsets instead of one, and `0xc000` is the trap: at the
ESP-IDF default `0x8000` the bootloader finds nothing and the unit does not
boot, with no error at flash time. Handle that by giving engineers the offsets
in writing — never from memory.

### Never tick "erase all flash"

It destroys the offline queue at `0xe20000` as well as NVS. Nothing in this
procedure needs it.

---

## 2. Cutting a release

### Version

`FW_VERSION` in `src/config.h` is the single source of truth. It is what every
device stamps into its `status`, `diagnostics` and data payloads (`fw:"…"`), so
it is what the fleet view and FOTA reason about.

Bump it for **any** change that reaches a station. Two different images must
never share a version — the cloud cannot then tell which one a unit runs, and
FOTA cannot answer "is this device older than what I am pushing".

The release build refuses to publish if the tag and `FW_VERSION` disagree.

### Steps

```bash
# 1. Bump FW_VERSION in src/config.h to the version you are shipping.

# 2. Clean build from the exact tree you intend to ship.
idf.py fullclean; idf.py build

# 3. Commit everything. The tag must point at a commit CI has passed.
git status --short          # must be empty
gh run list --limit 3       # ci green on HEAD

# 4. Tag and push. This triggers the release build.
git tag -a v<version> -m "<what this release is for>"
git push origin v<version>

# 5. Verify.
gh run list --limit 3
gh release view v<version>
```

The release workflow checks the tag against `FW_VERSION`, runs the host tests
(`test/host/runall.sh`), builds, and publishes the images with a `SHA256SUMS`
file. Release assets do not expire; CI artifacts do, which is why the release —
not a CI run — is the thing we hand out.

---

## 3. Distribution

The GitHub release is the source of truth. **The repo is private**, so support
engineers cannot open it — pull the files down and forward them:

```bash
gh release download v<version> -D flash-set-<version>
```

Send that folder by shared link (OneDrive) in preference to attachments — mail
filters strip `.bin` regularly, and a link can be re-shared next release. Either
way include `SHA256SUMS`, and **never rename the files**: the version in
`alds-gg-vvm701-fw-<version>.bin` is what stops the wrong image being flashed a
year later.

| Audience | What they take |
|---|---|
| Support engineers | The four files above, plus `SHA256SUMS` |
| Cloud team, for FOTA | `alds-gg-vvm701-fw-<version>.bin` only |

Put the offsets and the "leave flash settings on keep" rule in the message
BODY, not only in an attachment. That is the part that gets lost.

**FOTA takes the app image alone** — an OTA writes an app slot and cannot write
a bootloader or partition table. Put the version in the S3 path
(`.../gg/<version>/...`) so the URL says what it serves and a later release
cannot overwrite an earlier one at the same key while devices are mid-download.
Publish the FOTA trigger **non-retained**, and use a plain public object URL —
`FOTA_URL_MAX` is 511 and an S3 presigned URL is usually longer.

---

## 4. After flashing

A freshly flashed board comes up `unconfigured` and **polls nothing**. It does
connect to the compiled broker and announce itself (`status:"ready"`) — that is
how you find it.

1. Engineer flashes, confirms the unit appears on the broker, reports the IMEI.
2. Office pushes `setconfig` to `<base>/<imei>/setconfig` with that station's
   dispenser + radar list (see `CLOUD_INTEGRATION_GUIDE.md`).
3. Unit applies it, reboots, and starts publishing.

Flashing and provisioning are one operation. A board left at step 1 looks
healthy and produces no readings.

---

## 5. Changing broker credentials

Credentials are compiled into `src/config.h` (`MQTT_HOST`, `MQTT_PORT`,
`MQTT_USER`, `MQTT_PASS`, `MQTT_BASE_TOPIC`, `MQTT_TLS`, and the APN trio). The
firmware fingerprints those compiled values (`cred_fp`, FNV-1a) and stamps the
fingerprint at boot. When a new image carries different credentials, the
fingerprint changes and the firmware **overwrites the stored broker fields with
the compiled ones** before the modem is touched — so a reflash or FOTA that
changes the broker actually moves the unit, and NVS left over from the old
destination cannot keep it publishing to the wrong place. When the broker
DESTINATION moves, the offline queue is purged too, so the old site's buffered
readings cannot reach the new one.

Consequence: to repoint a fleet, edit `src/config.h`, bump `FW_VERSION`,
release, and roll it out. Equipment config (dispensers/radars) is left intact —
only the connectivity fields are forced. Announced to the cloud as
`firmware_credentials_applied`.

The current image ships **production** credentials (`mqtt.iotronsys.com:8777`,
user `gogas`, base topic `gogas`, TLS on) — the same set as alds-ggy (same
broker, user and base topic; ~300 GG stations and ~20 GGY stations on one tree).
The production broker's certificate is ECDSA and chains leaf -> YE1 -> **ISRG
Root YE** (cross-signed by ISRG Root X2 until 2032-09-02); `src/tls_ca.c`
bundles both, alongside ISRG Root X1 (dev broker), ISRG Root YR (GitHub Pages)
and Amazon Root CA 1 (S3 FOTA) — verified against the live endpoints
2026-09-28. Moving to a different broker is a config.h edit + a version bump,
released the normal way; check its cert root is in the bundle first
(section 6).

Because GG and GGY share one base topic, what keeps the two fleets apart is:
the fleet FOTA topic (`<base>/fleet/gg/update` vs `.../fleet/ggy/update`), the
FOTA JSON `target` belt (`gg` vs `ggy`), and the distinct `PRODUCT_ID`
(`alds-gg-vvm701/1`) — a GG↔GGY swap by FOTA wipes the equipment config and
comes up `unconfigured`, by design.

---

## 6. TLS

`src/tls_ca.c` carries three roots: **ISRG Root X1** (dev broker, GitHub raw)
and **ISRG Root X2** (production broker, both Let's Encrypt) for MQTT, and
**Amazon Root CA 1** for AWS S3 (the FOTA host). An endpoint that
moves to a CA outside that list fails the handshake **closed** — connection
refused, not silently accepted. That is the correct direction, but it means a
new root must be compiled in *before* the endpoint moves. Check before pointing
the fleet at any new host, including a CDN in front of an existing one.

---

## 7. Cross-product swap (gg ↔ cng-cpil ↔ ggy)

The same VVM701 board moves between an LPG (gg) and a CNG station and is
switched to the other product by **FOTA alone** — `partitions.csv`, the nvs
partition and the littlefs `storage` partition are byte-identical between the
products, and neither runs Secure Boot, so an unsigned image of either product
boots on a unit last flashed with the other.

No GG unit was ever flashed with the pre-rebase image (which used a different,
default partition layout), so every GG gateway carries the swap-set layout from
its first flash and is FOTA-reachable from CNG and GGY — and vice versa — with
no serial step. On such a handover the incoming
image purges the offline queue and cleans the sibling product's NVS namespace
so it cannot republish the other station's readings as its own. Never let the
two products diverge on `partitions.csv`, the partition-table offset, or the
secure-boot setting — those are not OTA-updatable and a divergence silently
breaks the swap.
