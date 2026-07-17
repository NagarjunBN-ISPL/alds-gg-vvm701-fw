#include "modem.h"
#include "config.h"
#include "mqtt.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "modem";

void modem_uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = MODEM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(MODEM_UART, MODEM_BUF, MODEM_BUF, 0, NULL, 0);
    uart_param_config(MODEM_UART, &cfg);
    uart_set_pin(MODEM_UART, MODEM_TX_PIN, MODEM_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MODEM_PWRKEY),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(MODEM_PWRKEY, 0);
}

static void (*s_flush_rescue)(const char *data) = NULL;

void modem_set_flush_rescue(void (*cb)(const char *data)) {
    s_flush_rescue = cb;
}

void modem_flush(void) {
    /* Bound the drain: "read until silent" never terminates if the modem
     * streams continuously (URC storm / firmware fault) — cap total bytes
     * so flush can never wedge the calling task. */
    uint8_t tmp[64];
    int drained = 0;
    const int drain_cap = 8192;

    if (!s_flush_rescue) {
        int n;
        while ((n = uart_read_bytes(MODEM_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(30))) > 0) {
            drained += n;
            if (drained > drain_cap) break;
        }
        return;
    }

    /* A +QMTRECV: URC can land on the wire between unrelated AT commands.
     * Accumulate what would otherwise be silently discarded and let the
     * rescue callback pull any complete message out of it before we
     * actually throw the rest away. Sized to mqtt.c's PAYLOAD_MAX (2048) —
     * a setconfig with the full connectivity block (apn/mqttHost/mqttPass/
     * mqttBaseTopic etc.) can approach ~600 bytes on its own, and this used
     * to be sized at exactly 600 with no margin: any URC landing in this
     * rescue path (rather than the normal drain_recv() receive path, which
     * is already sized to PAYLOAD_MAX) larger than that was silently
     * truncated and dropped, with no retry. */
    static char acc[2048 + 400];
    int total = 0;
    int n;
    while ((n = uart_read_bytes(MODEM_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(30))) > 0) {
        if (total + n < (int)sizeof(acc) - 1) {
            memcpy(acc + total, tmp, n);
            total += n;
        }
        drained += n;
        if (drained > drain_cap) break;
    }
    if (total > 0) {
        acc[total] = '\0';
        if (strstr(acc, "+QMTRECV:")) s_flush_rescue(acc);
    }
}

/* QMTCONN is the only AT command in this codebase carrying a plaintext
 * secret (the MQTT username/password — compiled into config.h by default,
 * but overridable at runtime via mqtt_set_credentials() from a remote
 * setconfig). Logging it verbatim at INFO level would put broker credentials
 * on the serial console for anyone with a USB-UART adapter or JTAG probe on
 * the board. Detect it by the literal password substring (the live one, via
 * mqtt_get_pass(), not the compile-time default) rather than by command
 * name, so this stays correct even if the QMTCONN call site changes shape. */
static void log_at_cmd(const char *cmd) {
    const char *pass = mqtt_get_pass();
    /* An empty needle makes strstr() "match" every command at position 0,
     * which would blank all AT-command logging — guard it explicitly rather
     * than rely on the password field always being non-empty. */
    if (pass[0] != '\0' && strstr(cmd, pass)) {
        ESP_LOGI(TAG, ">> AT+QMTCONN=... (credentials redacted)");
    } else if (strncmp(cmd, "AT+QICSGP=", 10) == 0) {
        /* QICSGP carries the APN username/password (apnUser/apnPass, also
         * remotely settable via setconfig). The APN NAME is not secret and is
         * still visible in the AT+CGDCONT log line, so redact the whole
         * command rather than parse out just the two credential fields. */
        ESP_LOGI(TAG, ">> AT+QICSGP=... (APN credentials redacted)");
    } else {
        ESP_LOGI(TAG, ">> %s", cmd);
    }
}

/* Called once a marker (expect string, or an error like "+CME ERROR") has
 * been spotted in buf, but the rest of that line — a value field, a numeric/
 * verbose error code, the trailing OK — may not have arrived yet since this
 * is checked byte-by-byte as bytes stream in. Settles briefly then drains up
 * to 500ms more so callers (and logs) see the complete line, not just
 * whatever had arrived at the instant the marker matched. */
static void drain_rest_of_line(char *buf, size_t buf_size, int *total) {
    vTaskDelay(pdMS_TO_TICKS(30));
    int space = (int)buf_size - 1 - *total;
    int64_t drain_deadline = esp_timer_get_time() + 500000LL;
    while (space > 0 && esp_timer_get_time() < drain_deadline) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        int n2 = uart_read_bytes(MODEM_UART, (uint8_t *)buf + *total,
                                  space, pdMS_TO_TICKS(20));
        if (n2 <= 0) break;
        *total += n2;
        buf[*total] = '\0';
        space = (int)buf_size - 1 - *total;
    }
}

bool modem_at(const char *cmd, const char *expect, int timeout_ms,
              char *resp, size_t resp_size) {
    modem_flush();

    /* snprintf returns the length it WANTED to write, not what fit. Passing
     * that straight to uart_write_bytes() when the command overflows `line`
     * would read PAST the end of the buffer and transmit whatever adjacent
     * memory happens to follow it — a silent out-of-bounds read that leaks
     * RAM contents onto the wire. No caller currently gets close (the longest
     * is QMTCONN at ~511 bytes from mqtt.c's cmd[512]), but fota.c passes a
     * cmd[640] buffer into here, so a future long command would land exactly
     * on this landmine. Fail loudly instead. */
    static char line[520];
    int n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    if (n < 0 || n >= (int)sizeof(line)) {
        ESP_LOGE(TAG, "modem_at: command too long (%d bytes, max %d) — dropped",
                 n, (int)sizeof(line) - 1);
        return false;
    }
    log_at_cmd(cmd);
    uart_write_bytes(MODEM_UART, line, n);

    static char buf[2048];
    memset(buf, 0, sizeof(buf));
    int  total = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            if (total < (int)sizeof(buf) - 1) {
                buf[total++] = (char)ch;
                buf[total]   = '\0';
            }
            if (expect && strstr(buf, expect)) {
                /* Hard cap on top of the "exit on silence" condition inside
                 * drain_rest_of_line() — a modem trickling single bytes
                 * continuously (RF noise, a chatty URC stream) would
                 * otherwise keep it alive for tens of seconds, and callers
                 * chaining many modem_at() calls (e.g. network_init()) could
                 * accumulate well past what's reasonable across a single
                 * bring-up attempt. */
                drain_rest_of_line(buf, sizeof(buf), &total);
                ESP_LOGI(TAG, "<< %s", buf);
                if (resp && resp_size > 0) {
                    strncpy(resp, buf, resp_size - 1);
                    resp[resp_size - 1] = '\0';
                }
                return true;
            }
            if (strstr(buf, "\r\nERROR") || strstr(buf, "+CME ERROR")) {
                /* Same drain as the expect-match branch above — the matched
                 * substring ("+CME ERROR" in particular) can arrive before
                 * its trailing ": <code>" has been transmitted, so without
                 * this a real numeric/verbose CME error code was being cut
                 * off here and logged as a bare, undiagnosable "+CME ERROR"
                 * (seen in the field on 2026-07-17 debugging AT+QGPS=1). */
                drain_rest_of_line(buf, sizeof(buf), &total);
                ESP_LOGW(TAG, "<< %s", buf);
                if (resp && resp_size > 0) {
                    strncpy(resp, buf, resp_size - 1);
                    resp[resp_size - 1] = '\0';
                }
                return false;
            }
        }
    }
    ESP_LOGW(TAG, "<< (timeout %d ms) %s", timeout_ms, buf);
    if (resp && resp_size > 0) {
        strncpy(resp, buf, resp_size - 1);
        resp[resp_size - 1] = '\0';
    }
    return false;
}

bool modem_read_until(const char *expect, int timeout_ms,
                      char *resp, size_t resp_size) {
    /* Like modem_at but sends nothing — just reads until pattern found */
    static char buf[2048];
    memset(buf, 0, sizeof(buf));
    int  total = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            if (total < (int)sizeof(buf) - 1) {
                buf[total++] = (char)ch;
                buf[total]   = '\0';
            }
            if (expect && strstr(buf, expect)) {
                /* The value for URCs like +QHTTPGET/+QHTTPREADFILE follows
                 * the marker (e.g. "+QHTTPGET: 0,200,309120") and may not
                 * have arrived yet — drain the rest of the line before
                 * returning so callers can parse the fields. Same class of
                 * fix as modem_at(). */
                vTaskDelay(pdMS_TO_TICKS(30));
                int space = (int)sizeof(buf) - 1 - total;
                /* Same unbounded-trickle risk as modem_at()'s trailing
                 * drain, and the same hard cap on top of "exit on silence". */
                int64_t drain_deadline = esp_timer_get_time() + 500000LL;
                while (space > 0 && esp_timer_get_time() < drain_deadline) {
                    if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
                    int n2 = uart_read_bytes(MODEM_UART, (uint8_t *)buf + total,
                                              space, pdMS_TO_TICKS(20));
                    if (n2 <= 0) break;
                    total += n2;
                    buf[total] = '\0';
                    space = (int)sizeof(buf) - 1 - total;
                    if (strchr(strstr(buf, expect), '\n')) break;
                }
                if (resp && resp_size > 0) {
                    strncpy(resp, buf, resp_size - 1);
                    resp[resp_size - 1] = '\0';
                }
                return true;
            }
        }
    }
    if (resp && resp_size > 0) {
        strncpy(resp, buf, resp_size - 1);
        resp[resp_size - 1] = '\0';
    }
    return false;
}

bool modem_upload_file(const char *filename, const char *data, size_t len) {
    char cmd[80];

    /* QFUPL does NOT overwrite an existing file — it rejects the upload
     * (+CME ERROR: 407, observed on real hardware 2026-07-17 on the second
     * boot after a successful first upload). The modem's UFS survives ESP
     * resets, since the modem itself stays powered across them, so on every
     * boot after the first the file is already there. Delete first and
     * ignore the failure when it doesn't exist — same fire-and-forget QFDEL
     * pattern fota.c already uses for its own download file. Without this,
     * a file that got there once could never be REPLACED — so a truncated
     * or corrupt first upload would be permanent, and TLS would fail
     * forever with no way to recover short of a modem power-cycle. */
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", filename);
    modem_at(cmd, "OK", 5000, NULL, 0);

    snprintf(cmd, sizeof(cmd), "AT+QFUPL=\"%s\",%d,10", filename, (int)len);
    if (!modem_at(cmd, "CONNECT", 10000, NULL, 0)) {
        ESP_LOGE(TAG, "QFUPL: no CONNECT for %s", filename);
        return false;
    }
    uart_write_bytes(MODEM_UART, data, len);

    /* +QFUPL: <size>,<crc16> confirms the modem received and wrote exactly
     * what was sent. A byte-exact CRC check isn't done here — the data is a
     * small, fixed, compiled-in constant (not something that varies at
     * runtime the way a FOTA download body does), so the only realistic
     * failure mode is "the write didn't happen at all," which the marker's
     * presence/absence already catches. */
    char resp[128] = {0};
    if (!modem_read_until("+QFUPL:", 15000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QFUPL: no confirmation for %s", filename);
        return false;
    }
    ESP_LOGI(TAG, "QFUPL: uploaded %s (%d bytes)", filename, (int)len);
    return true;
}

bool modem_power_on(void) {
    /* Try AT first — module may already be on */
    for (int i = 0; i < 3; i++) {
        if (modem_at("AT", "OK", 1000, NULL, 0)) {
            ESP_LOGI(TAG, "Modem already on");
            modem_at("ATE0", "OK", 2000, NULL, 0);
            return true;
        }
    }

    /* Pulse PWRKEY high for 2.2 s. Quectel's own hardware design spec calls
     * for >= 2.0 s minimum to reliably hit the internal PMIC power-on
     * threshold; 1.5 s was borderline and more likely to intermittently
     * fail to wake the baseband under field conditions (high enclosure
     * temperature, voltage sag) than on a bench. 2.2 s gives real margin
     * above the spec floor. */
    gpio_set_level(MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(2200));
    gpio_set_level(MODEM_PWRKEY, 0);

    /* Wait up to 15 s for the module to boot */
    for (int i = 0; i < 15; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (modem_at("AT", "OK", 1000, NULL, 0)) {
            ESP_LOGI(TAG, "Modem boot OK");
            modem_at("ATE0", "OK", 2000, NULL, 0);
            return true;
        }
    }
    ESP_LOGE(TAG, "Modem power-on failed");
    return false;
}

void modem_hard_reset(void) {
    ESP_LOGW(TAG, "Hard reset: power-cycling EC200U via PWRKEY ...");

    /* Graceful shutdown attempt — may fail if modem is hung, ignore either way */
    modem_at("AT+QPOWD=1", "OK", 5000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Force off: hold PWRKEY high > 2 s */
    gpio_set_level(MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
    gpio_set_level(MODEM_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));  /* allow full power-off */

    /* Power on: 2.2 s pulse (matches modem_power_on's pulse width) */
    gpio_set_level(MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(2200));
    gpio_set_level(MODEM_PWRKEY, 0);
    ESP_LOGI(TAG, "EC200U power-cycled — waiting for boot ...");

    for (int i = 0; i < 15; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (modem_at("AT", "OK", 1000, NULL, 0)) {
            ESP_LOGI(TAG, "Modem boot OK after hard reset");
            modem_at("ATE0", "OK", 2000, NULL, 0);
            return;
        }
    }
    ESP_LOGE(TAG, "Modem unresponsive after hard reset");
}

bool modem_get_imei(char *imei, size_t size) {
    char resp[128] = {0};
    if (!modem_at("AT+CGSN", "OK", 5000, resp, sizeof(resp))) return false;

    /* Find a 15-digit run of digits */
    for (char *p = resp; *p; p++) {
        if (*p >= '0' && *p <= '9') {
            size_t len = 0;
            while (p[len] >= '0' && p[len] <= '9') len++;
            if (len == 15) {
                size_t copy = (len < size - 1) ? len : size - 1;
                memcpy(imei, p, copy);
                imei[copy] = '\0';
                return true;
            }
            p += len - 1;
        }
    }
    return false;
}

int modem_get_csq(void) {
    char resp[64] = {0};
    if (!modem_at("AT+CSQ", "+CSQ:", 3000, resp, sizeof(resp))) return -1;
    char *p = strstr(resp, "+CSQ:");
    if (!p) return -1;
    p += 5;
    while (*p == ' ') p++;
    return atoi(p);
}

bool modem_get_vbat(int *vbat_mv) {
    char resp[64] = {0};
    if (!modem_at("AT+CBC", "+CBC:", 3000, resp, sizeof(resp))) return false;
    char *p = strstr(resp, "+CBC:");
    if (!p) return false;
    /* +CBC: <bcs>,<bcl>,<voltage> */
    p = strchr(p, ','); if (!p) return false; p++;
    p = strchr(p, ','); if (!p) return false; p++;
    *vbat_mv = atoi(p);
    return true;
}

bool modem_get_operator(char *out, size_t out_size) {
    if (out_size > 0) out[0] = '\0';
    char resp[128] = {0};
    if (!modem_at("AT+COPS?", "+COPS:", 5000, resp, sizeof(resp))) return false;
    /* +COPS: <mode>,<format>,"<operator>",<AcT> */
    char *q1 = strchr(resp, '"');
    if (!q1) return false;
    q1++;
    char *q2 = strchr(q1, '"');
    if (!q2) return false;
    size_t len = (size_t)(q2 - q1);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, q1, len);
    out[len] = '\0';
    return true;
}

bool modem_get_timestamp(char *buf, size_t size) {
    char resp[128] = {0};

    /* Prefer AT+QLTS=2 → "+QLTS: "2026/03/22,10:30:00+22,0"" */
    if (modem_at("AT+QLTS=2", "+QLTS:", 5000, resp, sizeof(resp))) {
        char *q = strchr(resp, '"');
        if (q) {
            q++;
            int Y, Mo, D, h, m, s;
            if (sscanf(q, "%d/%d/%d,%d:%d:%d", &Y, &Mo, &D, &h, &m, &s) == 6) {
                /* QLTS=2 is documented as 4-digit year, but some Quectel
                 * firmware variants have been seen returning 2-digit — apply
                 * the same normalization already used for the CCLK fallback
                 * below rather than assume the modem always complies. */
                if (Y < 100) Y += 2000;
                snprintf(buf, size, "%04d-%02d-%02d %02d:%02d:%02d",
                         Y, Mo, D, h, m, s);
                return true;
            }
        }
    }

    /* Fallback: AT+CCLK? → "+CCLK: "26/03/22,10:30:00+22"" */
    memset(resp, 0, sizeof(resp));
    if (modem_at("AT+CCLK?", "+CCLK:", 3000, resp, sizeof(resp))) {
        char *q = strchr(resp, '"');
        if (q) {
            q++;
            int y, Mo, D, h, m, s;
            if (sscanf(q, "%d/%d/%d,%d:%d:%d", &y, &Mo, &D, &h, &m, &s) == 6) {
                snprintf(buf, size, "20%02d-%02d-%02d %02d:%02d:%02d",
                         y, Mo, D, h, m, s);
                return true;
            }
        }
    }

    snprintf(buf, size, "1970-01-01 00:00:00");
    return false;
}
