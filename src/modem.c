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
#include <time.h>
#include <sys/time.h>

static const char *TAG = "modem";

void modem_uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = MODEM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    /* ESP_INTR_FLAG_IRAM matches CONFIG_UART_ISR_IN_IRAM=y (sdkconfig.defaults);
     * with 0 here IDF set the flag itself and logged that on every boot. Called
     * once at boot, so any error is real — log it like the other two UARTs. */
    esp_err_t err = uart_driver_install(MODEM_UART, MODEM_BUF, MODEM_BUF, 0, NULL,
                                        ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
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

/* Remove NUL bytes from buf[from .. *total) and re-terminate. Every reply here
 * is searched with strstr(), which stops at the first NUL — so one NUL from
 * line noise or a modem glitch ahead of the "OK" made a command that SUCCEEDED
 * read as a timeout (a FOTA wait for +QHTTPGET or +QHTTPREADFILE then sat out
 * its whole 150/310 s and spent an attempt), and hid any +QMTRECV behind it
 * from the URC rescue (a lost inbound command). The AT channel never carries a
 * meaningful NUL; the binary FOTA transfer does not come through here (fota.c
 * reads the UART itself). Same as alds-sr and cng-cpil. */
static void strip_nuls(char *buf, int from, int *total) {
    int w = from;
    for (int r = from; r < *total; r++)
        if (buf[r] != '\0') buf[w++] = buf[r];
    *total = w;
    buf[w] = '\0';
}

void modem_flush(void) {
    /* Bound the drain TWO ways: "read until silent" never terminates if the
     * modem streams continuously (URC storm / firmware fault), so total bytes
     * are capped — and, because a pathological trickle of one byte every
     * ~30 ms would take ~245 s to reach that cap (past the 120 s task WDT), a
     * wall-clock deadline ends it too, with the WDT fed meanwhile. Either bound
     * alone leaves a way for a faulty modem to turn a flush into a reboot. */
    uint8_t tmp[64];
    int drained = 0;
    const int     drain_cap = 8192;
    const int64_t deadline  = esp_timer_get_time() + 2000000LL;   /* 2 s */

    if (!s_flush_rescue) {
        int n;
        while ((n = uart_read_bytes(MODEM_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(30))) > 0) {
            drained += n;
            if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
            if (drained > drain_cap || esp_timer_get_time() > deadline) break;
        }
        return;
    }

    /* A +QMTRECV: URC can land on the wire between unrelated AT commands.
     * Accumulate what would otherwise be silently discarded and let the
     * rescue callback pull any complete message out of it before we
     * actually throw the rest away. Room for TWO of mqtt.c's PAYLOAD_MAX
     * (2048) messages plus their URC headers: a setconfig with the full
     * connectivity block can approach ~600 bytes, and an update carrying a
     * long presigned link (FOTA_URL_MAX 1535) ~1.7-1.9 KB, so one of those
     * beside another message did not fit the 2448 bytes this used to be. A
     * URC that does not fit is silently cut and dropped, with no retry. */
    static char acc[4096 + 400];
    int total = 0;
    int n;
    while ((n = uart_read_bytes(MODEM_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(30))) > 0) {
        if (total + n < (int)sizeof(acc) - 1) {
            memcpy(acc + total, tmp, n);
            int before = total;
            total += n;
            strip_nuls(acc, before, &total);
        }
        drained += n;
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        if (drained > drain_cap || esp_timer_get_time() > deadline) break;
    }
    if (total > 0) {
        acc[total] = '\0';
        /* +QMTSTAT (link-lost) as well as +QMTRECV: parse_qmtrecv() runs
         * check_qmtstat() first, so a lone +QMTSTAT in the pre-command drain must
         * reach it too or a silent link drop between commands goes unnoticed. */
        if (strstr(acc, "+QMTRECV:") || strstr(acc, "+QMTSTAT:")) s_flush_rescue(acc);
    }
}

/* Rescue any MQTT URC (+QMTRECV inbound message, +QMTSTAT link-lost) that
 * interleaved into a command's response buffer. Without this, a URC arriving on
 * the wire while an UNRELATED AT command (diagnostics CSQ/CBC/COPS, time sync,
 * GNSS) reads its reply is captured into that command's buffer and silently
 * discarded — losing an inbound setconfig/lock/rsp/FOTA command (Quectel
 * auto-ACKs a QoS-1 message to the broker on +QMTRECV, so it is NOT redelivered)
 * or the one-shot link-down notification (which stalls reconnect until the slow
 * publish-fail streak trips). parse_qmtrecv() acts only on a COMPLETE URC and is
 * already proven safe on an arbitrary buffer (modem_flush feeds it the same
 * way), so this is purely additive — it never changes the command's own result. */
static void modem_rescue_urcs(const char *buf) {
    if (s_flush_rescue && (strstr(buf, "+QMTRECV:") || strstr(buf, "+QMTSTAT:")))
        s_flush_rescue(buf);
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
        int before = *total;
        *total += n2;
        strip_nuls(buf, before, total);          /* re-terminates */
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

    /* 4 KB, not 2: an inbound update message carrying a long presigned link
     * (~1.7-1.9 KB) that lands while this command waits must fit beside its
     * reply, or it is cut, and a cut +QMTRECV is lost for good (the modem has
     * already acked it to the broker). */
    static char buf[4096];
    memset(buf, 0, sizeof(buf));
    int  total = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            if (ch == 0) continue;               /* see strip_nuls() */
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
                modem_rescue_urcs(buf);   /* a URC may have interleaved into this reply */
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
                modem_rescue_urcs(buf);   /* a URC may have interleaved into this reply */
                return false;
            }
        }
    }
    ESP_LOGW(TAG, "<< (timeout %d ms) %s", timeout_ms, buf);
    if (resp && resp_size > 0) {
        strncpy(resp, buf, resp_size - 1);
        resp[resp_size - 1] = '\0';
    }
    modem_rescue_urcs(buf);   /* a URC may have interleaved before the timeout */
    return false;
}

bool modem_read_until(const char *expect, int timeout_ms,
                      char *resp, size_t resp_size) {
    /* Like modem_at but sends nothing — just reads until pattern found.
     * 4 KB for the same reason as modem_at(). */
    static char buf[4096];
    memset(buf, 0, sizeof(buf));
    int  total = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            if (ch == 0) continue;               /* see strip_nuls() */
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
                    int before = total;
                    total += n2;
                    strip_nuls(buf, before, &total);   /* re-terminates */
                    space = (int)sizeof(buf) - 1 - total;
                    if (strchr(strstr(buf, expect), '\n')) break;
                }
                if (resp && resp_size > 0) {
                    strncpy(resp, buf, resp_size - 1);
                    resp[resp_size - 1] = '\0';
                }
                modem_rescue_urcs(buf);   /* a URC may have interleaved into this read */
                return true;
            }
        }
    }
    if (resp && resp_size > 0) {
        strncpy(resp, buf, resp_size - 1);
        resp[resp_size - 1] = '\0';
    }
    modem_rescue_urcs(buf);   /* a URC may have interleaved before the timeout */
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
    /* <size> is what the modem actually stored. The CA bundle grew to ~6.5 KB
     * on 2026-09-28 (Root YE/YR appended last), and a cut-off upload would
     * silently lose those roots first — so say so. Logged, not failed: the
     * file exists either way, and the TLS handshake is the real verdict. */
    const char *p = strstr(resp, "+QFUPL:");
    int stored = p ? atoi(p + 7) : -1;
    if (stored != (int)len)
        ESP_LOGE(TAG, "QFUPL: %s — modem stored %d of %d bytes (upload cut short?)",
                 filename, stored, (int)len);
    else
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

    /* Find a run of at least 15 digits and take the first 15 as the IMEI.
     * >= 15 (not == 15) so a module answering AT+CGSN with the 17-digit
     * IMEISV (IMEI + 2-digit software version) still yields a valid IMEI —
     * the IMEI is the leading 15 digits — instead of failing the run match
     * and falling back to the all-zero placeholder (which would collide
     * HW_IDs and command topics across every such unit). */
    for (char *p = resp; *p; p++) {
        if (*p >= '0' && *p <= '9') {
            size_t len = 0;
            while (p[len] >= '0' && p[len] <= '9') len++;
            if (len >= 15 && size > 15) {
                memcpy(imei, p, 15);
                imei[15] = '\0';
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

/* ── Time: network → RTC, everything else reads the RTC ─────────────────── */
static bool    s_time_synced   = false;
static int64_t s_last_sync_us  = 0;

/* Parse a Quectel time string and set the ESP32 RTC from it, in UTC.
 *
 * Format: "YYYY/MM/DD,HH:MM:SS±ZZ[,D]" (QLTS) or "YY/MM/DD,HH:MM:SS±ZZ" (CCLK).
 * The value is the modem's LOCAL time, and ±ZZ is its offset from UTC in
 * QUARTER-hours (IST = +5:30 = +22). The previous implementation parsed only
 * the six date/time fields and ignored ±ZZ entirely, so on an Indian network
 * it published IST while the cloud interpreted the field as UTC — every
 * timestamp 5:30 ahead. The offset is applied here so the RTC holds true UTC.
 *
 * The sign is located by scanning past the SECOND ':' rather than with
 * strchr('-'): a '-' can legitimately appear nowhere else in these formats
 * today, but anchoring to the seconds field keeps that from becoming a silent
 * trap if the modem ever emits a different date separator. */
/* Days since the Unix epoch (1970-01-01) for a proleptic-Gregorian UTC date,
 * per Howard Hinnant's days_from_civil algorithm. Self-contained so the UTC
 * conversion in parse_and_set_rtc() needs neither mktime()+tzset() (which
 * mutates the global TZ state) nor timegm() (not declared in ESP-IDF's newlib
 * headers). Exact for every date this firmware sees (validated to 2020–2099). */
static int64_t days_from_civil(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int64_t  era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);                       /* [0, 399]    */
    const unsigned doy = (153u * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1; /* [0, 365]    */
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;           /* [0, 146096] */
    return era * 146097 + (int64_t)doe - 719468;
}

/* Consecutive >24h re-sync rejections tolerated before we stop trusting the
 * stored clock and re-anchor to network time — see the guard in
 * parse_and_set_rtc(). At the ~6h resync cadence this bounds a wrong-clock
 * lock-in to ~18h instead of forever (the failure mode where a bad value
 * latched at the first sync rejects every later CORRECT time permanently). */
#define RTC_RESYNC_REANCHOR_AFTER 3

static bool parse_and_set_rtc(const char *s, bool two_digit_year) {
    int Y, Mo, D, h, m, sec;
    if (sscanf(s, "%d/%d/%d,%d:%d:%d", &Y, &Mo, &D, &h, &m, &sec) != 6)
        return false;

    int  tz_qh   = 0;
    char tz_sign = '+';
    {
        int colons = 0;
        for (const char *p = s; *p && *p != '"'; p++) {
            if (*p == ':') { colons++; continue; }
            if (colons >= 2 && (*p == '+' || *p == '-')) {
                tz_sign = *p;
                tz_qh   = atoi(p + 1);
                break;
            }
        }
    }
    /* tz_sign carries the direction, so tz_qh is a magnitude: 0..56 quarter-
     * hours covers every real zone (UTC-12 .. UTC+14). Anything outside that is
     * a mis-parse — treat it as "no offset" rather than shifting the clock by
     * hours on the strength of a garbled field. */
    if (tz_qh < 0 || tz_qh > 56) tz_qh = 0;

    if (two_digit_year || Y < 100) Y += 2000;

    /* Upper bound 2050, not 2099: an un-synced modem's CCLK placeholder is
     * "80/01/06" (Quectel's epoch), which two-digit-year handling turns into
     * 2080 — inside a 2099 cap, and on the INITIAL sync there is nothing to
     * compare against, so the RTC would be set 54 years ahead and every
     * payload stamped with it until the re-anchor logic below caught up
     * (~18 h). Found in cng-cpil; the range check is the only guard here. */
    if (Y < 2020 || Y > 2050 || Mo < 1 || Mo > 12 || D < 1 || D > 31 ||
        h < 0 || h > 23 || m < 0 || m > 59 || sec < 0 || sec > 60)
        return false;

    /* Convert the validated UTC calendar fields to epoch seconds directly — no
     * struct tm, no mktime()/tzset() global-TZ mutation, and no timegm() (which
     * ESP-IDF's newlib headers don't declare). See days_from_civil() above. */
    int64_t epoch = days_from_civil(Y, (unsigned)Mo, (unsigned)D) * 86400LL
                  + (int64_t)h * 3600 + (int64_t)m * 60 + sec;

    long off = (long)tz_qh * 15 * 60;
    epoch = (tz_sign == '+') ? epoch - off : epoch + off;

    /* ── Sanity-gate a RE-sync against a wild jump ────────────────────────
     * The calendar-field validation above rejects nonsense, but a garbled
     * response can still parse into a plausible WRONG time (say a corrupted
     * year or day). On the INITIAL sync we have nothing to compare against and
     * must accept it — that is the jump from 1970 to real time. But once the
     * clock has been set, real network time only ever drifts by seconds
     * between syncs, so anything more than a day of movement is a mis-parse,
     * not reality. Accepting it would stamp every payload with a wrong TS until
     * the next good sync (up to 6 h), and the cloud orders sales by TS — a
     * clock that jumps backwards makes totalizer deltas go negative, i.e.
     * phantom transactions. Reject and keep the last good time instead. */
    static int s_resync_rejects = 0;
    if (s_time_synced) {
        struct timeval now;
        gettimeofday(&now, NULL);
        int64_t delta = (int64_t)epoch - (int64_t)now.tv_sec;
        if (delta < 0) delta = -delta;
        if (delta > 24 * 3600) {
            /* Normally a >24h jump is a garbled response — reject and keep the
             * last good sync. But if the FIRST sync latched a wrong value (e.g.
             * the modem's own un-network-synced RTC via the CCLK fallback),
             * every later CORRECT time is also >24h away and would be rejected
             * forever. So after RTC_RESYNC_REANCHOR_AFTER consecutive
             * rejections, stop trusting the stored clock and accept the network
             * time as the new truth. */
            if (++s_resync_rejects < RTC_RESYNC_REANCHOR_AFTER) {
                ESP_LOGW(TAG, "RTC re-sync rejected: proposed time jumps %lld s from current "
                              "(likely a garbled response) — keeping last good sync "
                              "(reject %d/%d)",
                         (long long)delta, s_resync_rejects, RTC_RESYNC_REANCHOR_AFTER);
                return false;
            }
            ESP_LOGW(TAG, "RTC re-sync: %d consecutive >24h rejections — the stored clock "
                          "was likely wrong; re-anchoring to network time",
                     s_resync_rejects);
            /* fall through and accept the proposed time */
        }
    }

    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    if (settimeofday(&tv, NULL) != 0) return false;
    s_resync_rejects = 0;   /* clock is tracking again — reset the reject streak */
    return true;
}

/* The time string in an AT+QNTP result, +QNTP: <err>,"<time>" — or NULL unless
 * <err> is exactly 0 and a quoted time follows. <time> has the QLTS layout
 * ("YYYY/MM/DD,hh:mm:ss±zz", local time + quarter-hour offset), so it goes
 * through parse_and_set_rtc() and its range and jump checks like any other
 * source. */
static const char *qntp_time(const char *resp) {
    const char *p = resp ? strstr(resp, "+QNTP:") : NULL;
    if (!p) return NULL;
    p += 6;
    while (*p == ' ') p++;
    if (p[0] != '0' || p[1] != ',') return NULL;
    const char *q = strchr(p, '"');
    return q ? q + 1 : NULL;
}

/* NTP through the modem's PDP context 1 (the one network.c activates). Rate
 * limited to one try per NTP_RETRY_INTERVAL_S, alternating servers, because an
 * unreachable server holds the caller for up to NTP_TIMEOUT_MS. autosettime=1
 * also sets the modem's own clock, so a later CCLK read returns real time. */
static bool ntp_sync(void) {
    static int64_t  s_last_try_us = 0;
    static unsigned s_turn        = 0;
    int64_t now = esp_timer_get_time();
    if (s_last_try_us != 0 &&
        now - s_last_try_us < (int64_t)NTP_RETRY_INTERVAL_S * 1000000LL)
        return false;
    s_last_try_us = now;

    const char *server = (s_turn++ % 2) ? NTP_SERVER_2 : NTP_SERVER_1;
    char cmd[96];
    char resp[192] = {0};
    snprintf(cmd, sizeof(cmd), "AT+QNTP=1,\"%s\",123,1", server);
    if (!modem_at(cmd, "+QNTP:", NTP_TIMEOUT_MS, resp, sizeof(resp))) {
        ESP_LOGW(TAG, "NTP %s: no result", server);
        return false;
    }
    const char *t = qntp_time(resp);
    if (!t || !parse_and_set_rtc(t, false)) {
        ESP_LOGW(TAG, "NTP %s: unusable result", server);
        return false;
    }
    return true;
}

bool modem_time_sync(void) {
    char resp[128] = {0};

    /* Prefer AT+QLTS=2 → "+QLTS: "2026/03/22,10:30:00+22,0"" */
    if (modem_at("AT+QLTS=2", "+QLTS:", 5000, resp, sizeof(resp))) {
        char *q = strchr(resp, '"');
        if (q && parse_and_set_rtc(q + 1, false)) {
            s_time_synced  = true;
            s_last_sync_us = esp_timer_get_time();
            char now[32];
            rtc_timestamp(now, sizeof(now));
            ESP_LOGI(TAG, "RTC synced from AT+QLTS — %s UTC", now);
            return true;
        }
    }

    /* No network time: ask NTP before trusting the modem's own clock, which
     * free-runs (or sits at its 1980 placeholder) when the cell never sent
     * time. Rate limited inside ntp_sync(). */
    if (ntp_sync()) {
        s_time_synced  = true;
        s_last_sync_us = esp_timer_get_time();
        char now[32];
        rtc_timestamp(now, sizeof(now));
        ESP_LOGI(TAG, "RTC synced from NTP — %s UTC", now);
        return true;
    }

    /* Fallback: AT+CCLK? → "+CCLK: "26/03/22,10:30:00+22"" (2-digit year) */
    memset(resp, 0, sizeof(resp));
    if (modem_at("AT+CCLK?", "+CCLK:", 3000, resp, sizeof(resp))) {
        char *q = strchr(resp, '"');
        if (q && parse_and_set_rtc(q + 1, true)) {
            s_time_synced  = true;
            s_last_sync_us = esp_timer_get_time();
            char now[32];
            rtc_timestamp(now, sizeof(now));
            ESP_LOGI(TAG, "RTC synced from AT+CCLK — %s UTC", now);
            return true;
        }
    }

    /* A failed re-sync is not fatal: the RTC keeps running on the last good
     * sync, which is far better than the old behaviour of shipping 1970 for
     * that cycle. Only a device that has NEVER synced has unusable time. */
    ESP_LOGW(TAG, "RTC sync failed — %s",
             s_time_synced ? "continuing on last good sync" : "clock still UNSYNCED");
    return false;
}

bool rtc_timestamp(char *buf, size_t size) {
    if (!buf || size < 20) return false;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm t;
    /* gmtime_r, not localtime_r: the RTC holds UTC by construction (see
     * parse_and_set_rtc), so this stays correct even if TZ is changed later. */
    gmtime_r(&tv.tv_sec, &t);

    if (t.tm_year + 1900 < 2020) {          /* clock never set */
        snprintf(buf, size, "1970-01-01 00:00:00");
        return false;
    }
    snprintf(buf, size, "%04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    return true;
}

bool rtc_is_synced(void) { return s_time_synced; }

bool modem_time_resync_due(void) {
    if (!s_time_synced) return true;
    return (esp_timer_get_time() - s_last_sync_us)
           >= (int64_t)TIME_RESYNC_INTERVAL_S * 1000000LL;
}
