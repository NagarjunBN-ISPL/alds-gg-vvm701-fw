#pragma once
#include <stdbool.h>
#include <stddef.h>

void modem_uart_init(void);
bool modem_power_on(void);
void modem_hard_reset(void);
bool modem_at(const char *cmd, const char *expect, int timeout_ms,
              char *resp, size_t resp_size);
void modem_flush(void);
/* Called with any bytes modem_flush() is about to discard, before it
 * discards them, so a subscriber (e.g. MQTT) can rescue a +QMTRECV: URC
 * that arrived on the wire between AT commands instead of losing it. */
void modem_set_flush_rescue(void (*cb)(const char *data));
bool modem_get_imei(char *imei, size_t size);
int  modem_get_csq(void);
bool modem_get_vbat(int *vbat_mv);
/* Network operator name (e.g. "Airtel"), via AT+COPS?. */
bool modem_get_operator(char *out, size_t out_size);
/* ── Time ─────────────────────────────────────────────────────────────────
 * Network time is treated the way NTP is on any other system: it disciplines
 * the ESP32's RTC, and everything else reads the RTC. Timestamps must never
 * block on modem I/O — modem_get_timestamp() used to be called once per poll
 * cycle (an AT round-trip, up to 5 s, silently yielding 1970 on failure) and
 * the single value was then stamped onto every payload in that cycle, so the
 * last nozzle of a cycle carried a timestamp ~40 s older than its own reading.
 *
 * modem_time_sync(): pulls network time (AT+QLTS=2, falling back to AT+CCLK?),
 * converts the modem's LOCAL time + quarter-hour offset to UTC, and sets the
 * RTC via settimeofday(). Call once after network bring-up, then periodically
 * (see modem_time_resync_due()) to bound RTC drift.
 *
 * rtc_timestamp(): formats the current RTC time as UTC "YYYY-MM-DD HH:MM:SS".
 * Instant, non-blocking, cannot fail on I/O — so it is affordable to call at
 * the exact moment each nozzle is read. Returns false (and writes the epoch
 * string) if the RTC has never been synced, so callers can tell the difference
 * between "1970 because unsynced" and a real reading. */
bool modem_time_sync(void);
bool rtc_timestamp(char *buf, size_t size);
bool rtc_is_synced(void);
/* true if never synced, or the last good sync is older than
 * TIME_RESYNC_INTERVAL_S — drives the periodic re-sync from the poll loop. */
bool modem_time_resync_due(void);
/* Read UART (no command sent) until expect is found or timeout elapses. */
bool modem_read_until(const char *expect, int timeout_ms,
                      char *resp, size_t resp_size);
/* Uploads `len` bytes of `data` to a file on the modem's UFS filesystem via
 * AT+QFUPL (send command, wait for CONNECT, stream the raw bytes, wait for
 * the +QFUPL: confirmation). Overwrites any existing file of the same name.
 * Same CONNECT-prompt pattern fota.c already uses for AT+QHTTPURL. */
bool modem_upload_file(const char *filename, const char *data, size_t len);
