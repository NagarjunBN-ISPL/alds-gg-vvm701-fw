#include "fota.h"
#include "modem.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "fota";

#define FOTA_FILE      "UFS:vvm_fw.bin"
#define FOTA_CHUNK     4096
#define FOTA_DL_TIMEOUT_S 300  /* 5 min max download */

/* ── HTTPS download → EC200U UFS ─────────────────────────────────────── */
static bool fota_download(const char *url) {
    char cmd[640], resp[256];

    /* Configure SSL: no cert verify, TLS any version */
    modem_at("AT+QSSLCFG=\"sslversion\",0,4",    "OK", 3000, NULL, 0);
    modem_at("AT+QSSLCFG=\"seccertlevel\",0,0",  "OK", 3000, NULL, 0);
    modem_at("AT+QHTTPCFG=\"sslctxid\",0",       "OK", 3000, NULL, 0);
    modem_at("AT+QHTTPCFG=\"responseheader\",0", "OK", 3000, NULL, 0);

    /* Remove old file */
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
    modem_at(cmd, "OK", 5000, NULL, 0);

    /* Set URL */
    int url_len = strlen(url);
    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80", url_len);
    if (!modem_at(cmd, "CONNECT", 15000, NULL, 0)) {
        ESP_LOGE(TAG, "QHTTPURL no CONNECT");
        return false;
    }
    uart_write_bytes(MODEM_UART, url, url_len);
    /* Do NOT send anything — modem counts bytes and responds with OK itself */
    if (!modem_read_until("OK", 10000, NULL, 0)) {
        ESP_LOGE(TAG, "QHTTPURL post-URL no OK");
        return false;
    }

    /* Download to UFS */
    snprintf(cmd, sizeof(cmd), "AT+QHTTPGETEX=%d,\"%s\"",
             FOTA_DL_TIMEOUT_S, FOTA_FILE);
    ESP_LOGI(TAG, "Downloading firmware...");
    if (!modem_at(cmd, "+QHTTPGETEX:", (FOTA_DL_TIMEOUT_S + 30) * 1000,
                  resp, sizeof(resp))) {
        ESP_LOGE(TAG, "Download failed: %s", resp);
        return false;
    }

    /* Parse: +QHTTPGETEX: <err>,<http_code>,<file_size> */
    char *p = strstr(resp, "+QHTTPGETEX:");
    if (!p) return false;
    p += 12;
    while (*p == ' ') p++;
    int err = atoi(p);
    p = strchr(p, ','); if (!p) return false; p++;
    int code = atoi(p);
    if (err != 0 || code != 200) {
        ESP_LOGE(TAG, "HTTPS err=%d code=%d", err, code);
        return false;
    }
    ESP_LOGI(TAG, "Download OK (HTTP 200)");
    return true;
}

/* ── UFS → OTA partition ─────────────────────────────────────────────── */
static bool fota_flash(void) {
    char cmd[128], hdr[128];

    /* Open file */
    snprintf(cmd, sizeof(cmd), "AT+QFOPEN=\"%s\",0", FOTA_FILE);
    char resp[64] = {0};
    if (!modem_at(cmd, "+QFOPEN:", 10000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QFOPEN failed");
        return false;
    }
    char *p = strstr(resp, "+QFOPEN:");
    if (!p) return false;
    p += 8;
    while (*p == ' ') p++;
    int fh = atoi(p);
    ESP_LOGI(TAG, "File handle: %d", fh);

    /* Begin OTA */
    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) { ESP_LOGE(TAG, "No OTA partition"); return false; }

    esp_ota_handle_t ota_hdl;
    if (esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed");
        return false;
    }

    static uint8_t chunk[FOTA_CHUNK];
    bool ok = true;

    for (;;) {
        snprintf(cmd, sizeof(cmd), "AT+QFREAD=%d,%d", fh, FOTA_CHUNK);
        modem_flush();
        char full[160];
        int n = snprintf(full, sizeof(full), "%s\r\n", cmd);
        uart_write_bytes(MODEM_UART, full, n);

        /* Read header line: "CONNECT <actual_bytes>\r\n" */
        int hi = 0;
        bool got_connect = false;
        int  actual = 0;
        int64_t dl = esp_timer_get_time() + 10000000LL;
        while (esp_timer_get_time() < dl) {
            uint8_t ch;
            if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(20)) > 0) {
                if (hi < (int)sizeof(hdr) - 1) hdr[hi++] = ch;
                hdr[hi] = '\0';
                if (ch == '\n' && strstr(hdr, "CONNECT")) {
                    char *pn = strstr(hdr, "CONNECT");
                    pn += 7;
                    while (*pn == ' ') pn++;
                    actual = atoi(pn);
                    got_connect = true;
                    break;
                }
            }
        }
        if (!got_connect || actual == 0) break; /* EOF or error */

        /* Read 'actual' binary bytes */
        int got = 0;
        dl = esp_timer_get_time() + 15000000LL;
        while (got < actual && esp_timer_get_time() < dl) {
            int r = uart_read_bytes(MODEM_UART, chunk + got,
                                    actual - got, pdMS_TO_TICKS(200));
            if (r > 0) got += r;
        }
        if (got != actual) { ok = false; break; }

        if (esp_ota_write(ota_hdl, chunk, got) != ESP_OK) {
            ESP_LOGE(TAG, "OTA write error");
            ok = false;
            break;
        }

        /* Consume trailing OK — modem sends it automatically, no TX needed */
        modem_read_until("OK", 5000, NULL, 0);

        if (actual < FOTA_CHUNK) break; /* Last chunk */
    }

    /* Close and delete file */
    snprintf(cmd, sizeof(cmd), "AT+QFCLOSE=%d", fh);
    modem_at(cmd, "OK", 5000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
    modem_at(cmd, "OK", 5000, NULL, 0);

    if (!ok) { esp_ota_abort(ota_hdl); return false; }
    if (esp_ota_end(ota_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "OTA end (verify) failed");
        return false;
    }
    if (esp_ota_set_boot_partition(update_part) != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed");
        return false;
    }
    ESP_LOGI(TAG, "OTA flash complete");
    return true;
}

bool fota_perform(const char *url, const char *pub_topic) {
    ESP_LOGI(TAG, "FOTA start URL=%s topic=%s", url, pub_topic);
    if (!fota_download(url)) return false;
    if (!fota_flash())       return false;
    return true;  /* Caller publishes fota_success then calls esp_restart() */
}
