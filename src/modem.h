#pragma once
#include <stdbool.h>
#include <stddef.h>

void modem_uart_init(void);
bool modem_power_on(void);
void modem_hard_reset(void);
bool modem_at(const char *cmd, const char *expect, int timeout_ms,
              char *resp, size_t resp_size);
void modem_flush(void);
bool modem_get_imei(char *imei, size_t size);
int  modem_get_csq(void);
bool modem_get_vbat(int *vbat_mv);
bool modem_get_timestamp(char *buf, size_t size);
/* Read UART (no command sent) until expect is found or timeout elapses. */
bool modem_read_until(const char *expect, int timeout_ms,
                      char *resp, size_t resp_size);
