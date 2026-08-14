#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── GNSS reading, via the modem's built-in GPS ───────────────────────────
 * The EC200U-CN has GNSS integrated into the same module used for cellular
 * (its own dedicated antenna port, separate from the main LTE antenna) — see
 * hardware_board.pdf. There is no separate GPS chip/UART; every field here
 * comes from AT+QGPSLOC over the existing MODEM_UART.
 *
 * lat/lon are stored as double: a coordinate needs ~9 significant digits
 * (e.g. 12.9715987), which is past float's ~7.2-digit mantissa precision.
 * Only lat/lon are kept — AT+QGPSLOC also reports altitude/hdop/satellite
 * count/speed/course/fix-time, but nothing downstream uses them, and this is
 * a stationary asset (fuel station) where they add little; position was
 * confirmed against a real capture on 2026-07-17. If a confidence signal
 * (hdop/numSat) is ever needed — e.g. to distinguish a real relocation from
 * a marginal fix for a drift/tamper alert — gps_poll() already receives
 * those fields on the wire, they'd just need to be threaded back out. */
typedef struct {
    bool   valid;      /* true only if AT+QGPSLOC returned a usable fix */
    double latitude;   /* decimal degrees, +N/-S */
    double longitude;  /* decimal degrees, +E/-W */
} GpsFix_t;

/* Called once at boot, after network_init() has brought up the modem's AT
 * session for this boot (see the app_main comment on why it runs there and
 * not right after modem power-on). A true no-op (issues zero AT traffic)
 * when enable is false — this is what keeps the feature fully inert while
 * gps_enable is false in config, which is the default and normal state for
 * most gateways (not every unit has the GNSS antenna physically connected).
 * When enable is true, powers on the GNSS engine (AT+QGPS=1) once so it can
 * acquire/hold a fix in the background for the rest of uptime; gps_poll()
 * only ever queries afterwards, it never re-powers the engine. Failure is
 * logged and non-fatal, never blocks boot. */
void gps_init(bool enable);

/* Queries the current fix (AT+QGPSLOC). Returns false (leaving out->valid =
 * false) if the engine has no fix yet or the AT command errors/times out —
 * this is the normal state for a long time after first boot and is not
 * treated as a fault. Mirrors genset_poll()'s contract. */
bool gps_poll(GpsFix_t *out);

/* Powers the GNSS engine off (AT+QGPSEND). Only called when a live setconfig
 * push disables GPS on a device where it was previously on — not part of the
 * normal poll flow. */
void gps_end(void);

/* Serialise a reading to JSON in buf (includes HW_ID and TS), following the
 * same contract as the other payload builders: fix->valid selects the full
 * object vs. the {"data":"404"} fallback used everywhere else in this
 * firmware for "sensor tried, no reading this cycle". hw_id is expected to
 * be the bare IMEI (GPS describes the gateway itself, not a specific piece
 * of equipment behind it — no suffix), but the function itself doesn't care. */
void gps_build_payload(const GpsFix_t *fix, const char *hw_id, const char *ts,
                        char *buf, size_t buf_size);
