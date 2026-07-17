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
 * fix_time_utc/fix_date are kept as raw, unreformatted tokens copied
 * straight out of the modem response rather than parsed into a struct-tm —
 * see the AT-manual-verification note in gps.c before trusting fix_type/
 * speed/course_deg's exact units/encoding. */
typedef struct {
    bool     valid;          /* true only if AT+QGPSLOC returned a usable fix */

    double   latitude;       /* decimal degrees, +N/-S      */
    double   longitude;      /* decimal degrees, +E/-W      */
    float    altitude_m;     /* metres above sea level      */
    float    hdop;           /* horizontal dilution of precision */
    uint8_t  num_sats;       /* satellites used in the fix  */
    char     fix_time_utc[16]; /* raw HHMMSS.s token, unformatted */
    char     fix_date[8];      /* raw DDMMYY token, unformatted   */

    /* Present in Quectel's general GNSS field set but NOT yet confirmed for
     * exact meaning/units on this specific module/firmware — see gps.c. */
    uint8_t  fix_type;       /* TODO verify against AT manual (2D/3D encoding) */
    float    speed;          /* TODO verify unit (km/h vs knots)               */
    float    course_deg;     /* TODO verify true vs magnetic course            */
} GpsFix_t;

/* Called once at boot, right after the modem is confirmed powered on. A true
 * no-op (issues zero AT traffic) when enable is false — this is what keeps
 * the feature fully inert while gps_enable is false in config, which is the
 * default and normal state for most gateways (not every unit has the GNSS
 * antenna physically connected). When enable is true, powers on the GNSS
 * engine (AT+QGPS=1) once so it can acquire/hold a fix in the background for
 * the rest of uptime; gps_poll() only ever queries afterwards, it never
 * re-powers the engine. Failure is logged and non-fatal, never blocks boot. */
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

/* Serialise a reading to JSON in buf (includes HW_ID and TS), mirroring
 * genset_build_payload()'s contract exactly: fix->valid selects the full
 * object vs. the {"data":"404"} fallback used everywhere else in this
 * firmware for "sensor tried, no reading this cycle". hw_id is expected to
 * be the bare IMEI (GPS describes the gateway itself, not a specific piece
 * of equipment behind it — no suffix), but the function itself doesn't care. */
void gps_build_payload(const GpsFix_t *fix, const char *hw_id, const char *ts,
                        char *buf, size_t buf_size);
