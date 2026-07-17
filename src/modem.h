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
bool modem_get_timestamp(char *buf, size_t size);
/* Read UART (no command sent) until expect is found or timeout elapses. */
bool modem_read_until(const char *expect, int timeout_ms,
                      char *resp, size_t resp_size);
/* Uploads `len` bytes of `data` to a file on the modem's UFS filesystem via
 * AT+QFUPL (send command, wait for CONNECT, stream the raw bytes, wait for
 * the +QFUPL: confirmation). Overwrites any existing file of the same name.
 * Same CONNECT-prompt pattern fota.c already uses for AT+QHTTPURL. */
bool modem_upload_file(const char *filename, const char *data, size_t len);
