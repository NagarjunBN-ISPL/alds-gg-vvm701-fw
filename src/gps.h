#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── GNSS reading, via the modem's built-in GPS ───────────────────────────
 * The EC200U-CN has GNSS integrated into the same module used for cellular
 * (its own dedicated antenna port, separate from the main LTE antenna) — see
 * hardware_board.pdf. There is no separate GPS chip/UART; every field here
 * comes from AT+QGPSLOC=2 over the existing MODEM_UART.
 *
 * lat/lon are double: a coordinate needs ~9 significant digits
 * (e.g. 12.9715987), past float's ~7.2-digit mantissa. This is a MOVING
 * asset, so the fix-quality and motion fields the modem already sends in the
 * same reply are kept too — hdop and satellite count let the cloud discard a
 * marginal fix instead of drawing a jump, and speed is a direct "is the
 * bowser moving" signal. They are optional on the wire: a fix with lat/lon
 * but none of them is still a fix. Field order and units per the EC200U
 * GNSS Application Note, verified against a real capture 2026-07-17. */
typedef struct {
    bool   valid;      /* true only if AT+QGPSLOC returned a usable fix */
    double latitude;   /* decimal degrees, +N/-S */
    double longitude;  /* decimal degrees, +E/-W */
    float  hdop;       /* horizontal dilution of precision, 0 = not reported */
    float  speed_kmh;  /* speed over ground, km/h; meaningful only if speed_valid */
    bool   speed_valid;/* the modem supplied a readable speed field. Kept apart
                        * from the value because 0.0 is the one reading a moving
                        * asset most needs ("stopped") and must be publishable,
                        * while an absent/garbled field must not masquerade as it */
    int    nsat;       /* satellites used, 0 = not reported */
} GpsFix_t;

/* Called once at boot, after network_init() has brought up the modem's AT
 * session for this boot. When enable is false, nothing GPS is read or
 * published anywhere (the caller never polls), and this makes sure the MODEM
 * is off too: session stopped, and the saved "autogps" and AGPS switches
 * turned off, so a unit whose GPS was switched off — or one that arrived by
 * FOTA from a product that had it on — does not run GNSS or fetch AGPS data
 * on its own. A modem that is already off gets three queries and no write.
 * When enable is true: makes sure the saved modem options are right —
 * "autogps" on (the engine starts with the modem, before this firmware asks),
 * "apflash" on (hot start from saved ephemeris within ~2 h of a power-down),
 * "nmeasrc" off (no NMEA on the AT port) — sets up AGPS on its own PDP
 * context per GPS_AGPS_* in config.h, then powers the engine on (AT+QGPS=1)
 * in case it is not already running. gps_poll() only ever queries
 * afterwards, except to re-arm the engine after a modem power-cycle, and it
 * logs the time to first fix and a one-shot antenna check (GSV C/N0). Every
 * failure here is logged and non-fatal. */
void gps_init(bool enable);

/* Pure: summarise every "$xxGSV" sentence in a QGPSGNMEA reply — satellites
 * listed, those tracked at >= strong_db dB-Hz, and the best C/N0 seen. Any
 * talker (GP, BD, GB, GL, GA, GN, PQ). Returns false when the text holds no
 * GSV sentence at all. Host-tested in test/host/test_gps.c. */
bool gps_gsv_summary(const char *resp, int strong_db, int *in_view, int *strong, int *best);

/* Queries the current fix (AT+QGPSLOC=2). Returns false (leaving out->valid =
 * false) if the engine has no fix yet or the AT command errors/times out —
 * the normal state for a while after a cold start, not a fault. */
bool gps_poll(GpsFix_t *out);

/* Pure parser behind gps_poll(): the modem's reply text (anything containing
 * "+QGPSLOC: ...") -> fix. No I/O, so it is host-tested against captured and
 * corrupted replies (test/host/test_gps.c). Rejects out-of-range and
 * non-finite coordinates and the (0,0) origin. */
bool gps_parse_loc(const char *resp, GpsFix_t *out);

/* Switches the modem's GNSS fully off, as gps_init(false) does. Called when a
 * live setconfig push disables GPS on a device where it was on. */
void gps_end(void);

/* Serialise a reading to JSON in buf (includes HW_ID and TS), following the
 * same contract as the other payload builders: fix->valid selects the full
 * object vs. the {"data":"404"} fallback used everywhere else in this
 * firmware for "sensor tried, no reading this cycle". hw_id is the bare IMEI
 * (GPS describes the gateway itself, not equipment behind it — no suffix).
 * hdop/nsat are emitted when non-zero (a fix never has zero of either, so
 * zero means "not reported"); spd is emitted whenever the modem supplied the
 * field, INCLUDING 0.0 — "stopped" is a reading, not an absence. A consumer
 * keyed on lat/lon alone is unaffected. */
void gps_build_payload(const GpsFix_t *fix, const char *hw_id, const char *ts,
                        char *buf, size_t buf_size);
