#include "fota.h"
#include "modem.h"
#include "network.h"
#include "tls_ca.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

static const char *TAG = "fota";

/* Write a short machine-readable failure cause into the caller's buffer
 * (safe with a NULL buffer). Only the FIRST cause set per attempt survives
 * for the caller — but since each failure path returns immediately, the one
 * set is always the actual point of failure. */
static void fota_set_reason(char *reason, size_t size, const char *fmt, ...) {
    if (!reason || size == 0) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(reason, size, fmt, ap);
    va_end(ap);
}

#define FOTA_FILE      "UFS:vvm_fw.bin"
#define FOTA_CHUNK     4096
#define FOTA_SSL_CTX   1     /* FOTA HTTPS uses SSL ctx 1; MQTT uses ctx 2 */
/* Wall-clock ceiling on the whole UFS->OTA flash loop. The loop is already
 * bounded (each UART read has its own deadline; esp_ota_write fails when the
 * partition fills) and the reads are LOCAL to the modem's UFS, so a real
 * flash finishes in ~1-2 min even for a large image. This cap only exists as
 * belt-and-suspenders for a driver/hardware-level UART hang while the task
 * WDT is intentionally detached for FOTA — abort cleanly instead of tying up
 * an unattended station indefinitely. */
#define FOTA_FLASH_MAX_MS  300000

/* ── HTTPS download → EC200U UFS ──────────────────────────────────────────
 * Ported from the proven OTA_Integration flow: full QSSLCFG setup on a
 * dedicated SSL context, QHTTPURL → QHTTPGET (wait for +QHTTPGET URC with
 * http status) → QHTTPREADFILE (save body to UFS). The previous QHTTPGETEX
 * one-shot + "seccertlevel" config was rejected by this modem firmware and
 * never downloaded a byte.
 */
/* Ensure the PDP data context is active — MQTT teardown (QMTCLOSE) can
 * leave it in a state where QHTTPGET fails to connect. Mirrors
 * OTA_Integration's pdp_refresh(). */
static void fota_pdp_refresh(void) {
    char resp[128];
    modem_at("AT+QIACT?", "OK", 3000, resp, sizeof(resp));
    if (strstr(resp, "+QIACT: 1,1")) {
        ESP_LOGI(TAG, "PDP already active");
        return;
    }
    ESP_LOGW(TAG, "PDP not active — reactivating for FOTA ...");
    char cmd[200];
    modem_at("AT+QIDEACT=1", "OK", 15000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    network_build_qicsgp(cmd, sizeof(cmd));
    modem_at(cmd, "OK", 5000, NULL, 0);
    modem_at("AT+QIACT=1", "OK", 60000, NULL, 0);
}

static bool fota_download(const char *url, char *reason, size_t reason_size) {
    char cmd[640], resp[256];

    fota_pdp_refresh();

    /* SSL config on dedicated context. Until 2026-07-17 this ran with
     * seclevel=0 PLUS ignoreinvalidcertsign=1 and ignoremulticertchainverify
     * =1 — meaning the HTTPS-only check in fota_perform() was the only real
     * gate, and a network-position attacker (rogue DNS, compromised APN
     * routing, a fake AP) could present ANY certificate, including a
     * self-signed one, and this device would download and FLASH whatever
     * "firmware" it served. That's the worst-case outcome in this whole
     * system: attacker-controlled code execution. Now verified against the
     * CA tls_ca_provision() uploads at boot (see tls_ca.c) — cacert is
     * re-asserted here every attempt, same defensive posture as the rest of
     * this block, since a modem_hard_reset() between retries (see
     * fota_perform()) clears QSSLCFG state.
     *
     * ignorelocaltime stays 1: this is a clock-skew safety net (the modem's
     * own RTC may not be network-synced yet at the point FOTA runs), not a
     * certificate-authenticity check — keeping it doesn't reopen the RCE
     * gap the other two flags did. sni stays 1: needed for any host that
     * serves multiple certs off one IP (common on CDNs/S3-style storage).
     *
     * VERIFIED on real hardware 2026-07-17 against two different FOTA hosts,
     * both downloading successfully with validation active:
     *   AWS S3  (…s3.ap-south-1.amazonaws.com) — Amazon RSA 2048 M01
     *                                            -> Amazon Root CA 1
     *   GitHub  (raw.githubusercontent.com)     — LE YR2 -> ISRG Root YR
     *                                            -> ISRG Root X1 (4-deep,
     *                                            cross-signed: also proves
     *                                            ignoremulticertchainverify=0
     *                                            handles real chains) */
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"sslversion\",%d,4", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"ciphersuite\",%d,0XFFFF", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"cacert\",%d,\"%s\"", FOTA_SSL_CTX, TLS_CA_FILE);
    modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"seclevel\",%d,1", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"ignorelocaltime\",%d,1", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"ignoreinvalidcertsign\",%d,0", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"ignoremulticertchainverify\",%d,0", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"sni\",%d,1", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);

    /* Bind HTTP to PDP context 1 and the SSL context; no response header. */
    modem_at("AT+QHTTPCFG=\"contextid\",1",      "OK", 3000, NULL, 0);
    modem_at("AT+QHTTPCFG=\"responseheader\",0", "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QHTTPCFG=\"sslctxid\",%d", FOTA_SSL_CTX);
    modem_at(cmd, "OK", 3000, NULL, 0);

    /* Reset any prior HTTP session state (ignore errors). */
    modem_at("AT+QHTTPSTOP",   "OK", 3000, NULL, 0);
    modem_at("AT+QHTTPDISCON", "OK", 5000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* Remove old file */
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
    modem_at(cmd, "OK", 5000, NULL, 0);

    /* Set URL: modem echoes CONNECT, we stream the URL bytes, it replies OK */
    int url_len = strlen(url);
    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,30", url_len);
    if (!modem_at(cmd, "CONNECT", 15000, NULL, 0)) {
        ESP_LOGE(TAG, "QHTTPURL no CONNECT");
        fota_set_reason(reason, reason_size, "url_setup_no_connect");
        return false;
    }
    uart_write_bytes(MODEM_UART, url, url_len);
    if (!modem_read_until("OK", 10000, NULL, 0)) {
        ESP_LOGE(TAG, "QHTTPURL post-URL no OK");
        fota_set_reason(reason, reason_size, "url_setup_no_ok");
        return false;
    }

    /* Kick off the GET — returns OK immediately, then a +QHTTPGET: URC
     * arrives (up to ~2 min later) with err,http_status,content_length. */
    ESP_LOGI(TAG, "Downloading firmware...");
    if (!modem_at("AT+QHTTPGET=120", "OK", 5000, NULL, 0)) {
        ESP_LOGE(TAG, "QHTTPGET rejected");
        fota_set_reason(reason, reason_size, "get_rejected");
        return false;
    }
    if (!modem_read_until("+QHTTPGET:", 150000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "Timeout waiting for +QHTTPGET");
        fota_set_reason(reason, reason_size, "get_timeout");
        return false;
    }
    /* +QHTTPGET: <err>,<http_code>,<content_len> */
    int g_err = -1, g_status = -1, g_len = -1;
    char *p = strstr(resp, "+QHTTPGET:");
    if (p) sscanf(p, "+QHTTPGET: %d,%d,%d", &g_err, &g_status, &g_len);
    if (g_err != 0 || g_status != 200) {
        ESP_LOGE(TAG, "HTTPS GET failed (err=%d http=%d)", g_err, g_status);
        /* The single most useful diagnostic: http=404 (wrong file/URL),
         * 403 (forbidden), 505 (mangled URL) vs. err!=0 with http=-1
         * (socket/DNS/TLS never reached the server). */
        fota_set_reason(reason, reason_size, "http_%d_err_%d", g_status, g_err);
        return false;
    }
    /* g_err/g_status parsing correctly (both matched) doesn't guarantee the
     * third field did too — a response with no Content-Length would leave
     * g_len at its -1 sentinel. Proceeding would save a 0-byte file, and
     * fota_flash() would then fail cleanly, but only after burning a retry
     * attempt on a request that was never going to work. */
    if (g_len <= 0) {
        ESP_LOGE(TAG, "HTTPS GET: missing or invalid content-length (%d)", g_len);
        fota_set_reason(reason, reason_size, "bad_content_length");
        return false;
    }
    ESP_LOGI(TAG, "HTTP 200 OK (%d bytes) — saving to UFS ...", g_len);

    /* Save the response body to a UFS file, then fota_flash() reads it out. */
    snprintf(cmd, sizeof(cmd), "AT+QHTTPREADFILE=\"%s\",120", FOTA_FILE);
    if (!modem_at(cmd, "OK", 5000, NULL, 0)) {
        ESP_LOGE(TAG, "QHTTPREADFILE rejected");
        fota_set_reason(reason, reason_size, "readfile_rejected");
        return false;
    }
    if (!modem_read_until("+QHTTPREADFILE:", 130000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QHTTPREADFILE timeout");
        fota_set_reason(reason, reason_size, "readfile_timeout");
        return false;
    }
    int dl_result = -1;
    p = strstr(resp, "+QHTTPREADFILE:");
    if (p) sscanf(p, "+QHTTPREADFILE: %d", &dl_result);
    if (dl_result != 0) {
        ESP_LOGE(TAG, "QHTTPREADFILE err=%d", dl_result);
        fota_set_reason(reason, reason_size, "readfile_err_%d", dl_result);
        return false;
    }
    ESP_LOGI(TAG, "UFS download OK — flashing ...");
    return true;
}

/* ── UFS → OTA partition ─────────────────────────────────────────────── */
static bool fota_flash(char *reason, size_t reason_size) {
    char cmd[128], hdr[128];

    /* Open file */
    snprintf(cmd, sizeof(cmd), "AT+QFOPEN=\"%s\",0", FOTA_FILE);
    char resp[64] = {0};
    if (!modem_at(cmd, "+QFOPEN:", 10000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QFOPEN failed");
        fota_set_reason(reason, reason_size, "file_open_failed");
        return false;
    }
    char *p = strstr(resp, "+QFOPEN:");
    if (!p) { fota_set_reason(reason, reason_size, "file_open_parse"); return false; }
    p += 8;
    while (*p == ' ') p++;
    int fh = atoi(p);
    ESP_LOGI(TAG, "File handle: %d", fh);

    /* Begin OTA. The EC200U's UFS only holds a handful of simultaneously-open
     * file handles — fh is already open at this point, so every return path
     * below must close+delete it, or a repeated failure here (rare, but not
     * impossible: a corrupt partition table, or esp_ota_begin failing under
     * heap pressure) leaks a handle each time until the UFS runs out and
     * every future FOTA attempt fails at QFOPEN until a physical reboot. */
    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        ESP_LOGE(TAG, "No OTA partition");
        fota_set_reason(reason, reason_size, "no_ota_partition");
        snprintf(cmd, sizeof(cmd), "AT+QFCLOSE=%d", fh);
        modem_at(cmd, "OK", 5000, NULL, 0);
        snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
        modem_at(cmd, "OK", 5000, NULL, 0);
        return false;
    }

    esp_ota_handle_t ota_hdl;
    if (esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed");
        fota_set_reason(reason, reason_size, "ota_begin_failed");
        snprintf(cmd, sizeof(cmd), "AT+QFCLOSE=%d", fh);
        modem_at(cmd, "OK", 5000, NULL, 0);
        snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
        modem_at(cmd, "OK", 5000, NULL, 0);
        return false;
    }

    static uint8_t chunk[FOTA_CHUNK];
    bool ok = true;
    int64_t flash_deadline = esp_timer_get_time() + (int64_t)FOTA_FLASH_MAX_MS * 1000;

    for (;;) {
        if (esp_timer_get_time() > flash_deadline) {
            ESP_LOGE(TAG, "flash loop exceeded %d ms — aborting", FOTA_FLASH_MAX_MS);
            fota_set_reason(reason, reason_size, "flash_timeout");
            ok = false;
            break;
        }
        snprintf(cmd, sizeof(cmd), "AT+QFREAD=%d,%d", fh, FOTA_CHUNK);
        modem_flush();
        /* Same snprintf-return bounds hazard guarded in modem_at()/mqtt.c —
         * this loop writes the UART directly rather than going through
         * modem_at(), so it needs its own check. */
        char full[160];
        int n = snprintf(full, sizeof(full), "%s\r\n", cmd);
        if (n < 0 || n >= (int)sizeof(full)) {
            ESP_LOGE(TAG, "QFREAD cmd too long (%d) — aborting", n);
            fota_set_reason(reason, reason_size, "read_cmd_too_long");
            ok = false;
            break;
        }
        uart_write_bytes(MODEM_UART, full, n);

        /* Read header line: "CONNECT <actual_bytes>\r\n" */
        int hi = 0;
        bool got_connect = false;
        int  actual = 0;
        int64_t dl = esp_timer_get_time() + 10000000LL;
        while (esp_timer_get_time() < dl) {
            /* Feed the WDT: this loop reads the UART directly instead of via
             * modem_at() (which feeds internally), and can sit here for the
             * full 10 s. Harmless while FOTA ran with the WDT detached, but
             * the WDT now stays armed for the device's whole life — an
             * unfed 10 s window is still far inside the 120 s timeout, but
             * feeding here keeps the margin honest rather than accidental. */
            if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
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

        /* Never trust a length parsed off the wire: one corrupted byte in
         * the "CONNECT <n>" header ("CONNECT 99999") and the read below
         * would overflow chunk[FOTA_CHUNK] — memory corruption mid-OTA.
         * We asked for at most FOTA_CHUNK bytes, so anything larger (or
         * negative from atoi garbage) is a corrupt header: abort cleanly. */
        if (actual < 0 || actual > FOTA_CHUNK) {
            ESP_LOGE(TAG, "QFREAD corrupt length %d (max %d) — aborting", actual, FOTA_CHUNK);
            fota_set_reason(reason, reason_size, "read_corrupt_length");
            ok = false;
            break;
        }

        /* Read 'actual' binary bytes */
        int got = 0;
        dl = esp_timer_get_time() + 15000000LL;
        while (got < actual && esp_timer_get_time() < dl) {
            if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
            int r = uart_read_bytes(MODEM_UART, chunk + got,
                                    actual - got, pdMS_TO_TICKS(200));
            if (r > 0) got += r;
        }
        if (got != actual) {
            fota_set_reason(reason, reason_size, "read_short");
            ok = false;
            break;
        }

        if (esp_ota_write(ota_hdl, chunk, got) != ESP_OK) {
            ESP_LOGE(TAG, "OTA write error");
            fota_set_reason(reason, reason_size, "ota_write_failed");
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

    if (!ok) {
        esp_ota_abort(ota_hdl);
        /* A loop failure set a specific reason already; guard the (currently
         * unreachable) case where ok is false with none set. */
        if (reason && reason[0] == '\0') fota_set_reason(reason, reason_size, "download_read_failed");
        return false;
    }
    if (esp_ota_end(ota_hdl) != ESP_OK) {
        /* esp_ota_end verifies the image (magic byte, SHA if secure boot) —
         * this is what fails on a truncated or corrupted download. */
        ESP_LOGE(TAG, "OTA end (verify) failed");
        fota_set_reason(reason, reason_size, "image_verify_failed");
        return false;
    }
    if (esp_ota_set_boot_partition(update_part) != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed");
        fota_set_reason(reason, reason_size, "set_boot_failed");
        return false;
    }
    ESP_LOGI(TAG, "OTA flash complete");
    return true;
}

#define FOTA_MAX_RETRIES 3

/* Strip leading/trailing whitespace, quotes and CR/LF from the URL.
 * MQTT publishers (dashboards, scripts, MQTT Explorer) often append a
 * trailing newline — the modem then emits "GET /path\n  HTTP/1.1" and the
 * server rejects it (S3 answers 505 HTTP Version Not Supported). Ported
 * from OTA_Integration's sanitize_url(). */
static void sanitize_url(char *url) {
    size_t start = 0;
    while (url[start] == ' '  || url[start] == '\t' ||
           url[start] == '"'  || url[start] == '\'' ||
           url[start] == '\r' || url[start] == '\n')
        start++;
    if (start > 0)
        memmove(url, url + start, strlen(url) - start + 1);

    size_t len = strlen(url);
    while (len > 0 && (url[len - 1] == '\n' || url[len - 1] == '\r' ||
                       url[len - 1] == ' '  || url[len - 1] == '\t' ||
                       url[len - 1] == '"'  || url[len - 1] == '\'')) {
        url[--len] = '\0';
    }
}

bool fota_perform(const char *url, const char *pub_topic,
                  char *reason, size_t reason_size) {
    if (reason && reason_size) reason[0] = '\0';

    static char clean_url[512];
    strncpy(clean_url, url, sizeof(clean_url) - 1);
    clean_url[sizeof(clean_url) - 1] = '\0';
    sanitize_url(clean_url);

    /* HTTPS only. "http" (4 chars) would also match "http://" — accepting
     * plaintext FOTA means the firmware binary itself travels unencrypted
     * and unauthenticated, letting anyone on the network path (or a
     * compromised/spoofed server) hand the device arbitrary code to flash
     * and boot. We only ever generate/expect https:// URLs. */
    if (strncmp(clean_url, "https://", 8) != 0) {
        ESP_LOGE(TAG, "FOTA: rejecting non-HTTPS URL [%s]", clean_url);
        fota_set_reason(reason, reason_size, "non_https_url");
        return false;
    }
    ESP_LOGI(TAG, "FOTA start URL=[%s] topic=%s", clean_url, pub_topic);

    /* The download can fail transiently on cellular (err=716 socket connect)
     * — plain retries cover that. But in-modem failures like err=729 (memory
     * allocation) persist across HTTP sessions and only clear with a baseband
     * restart, so before the FINAL attempt escalate: PWRKEY power-cycle the
     * modem and rebuild the network, then try once more. */
    bool downloaded = false;
    for (int attempt = 1; attempt <= FOTA_MAX_RETRIES; attempt++) {
        if (attempt == FOTA_MAX_RETRIES) {
            ESP_LOGW(TAG, "FOTA: escalating — power-cycling modem before final attempt");
            modem_hard_reset();
            if (!network_init()) {
                ESP_LOGE(TAG, "FOTA: network rebuild failed after modem reset");
                fota_set_reason(reason, reason_size, "network_rebuild_failed");
                break;
            }
        }
        /* fota_download overwrites reason on each failed attempt, so the
         * buffer ends up holding the LAST attempt's cause — the relevant one.*/
        if (fota_download(clean_url, reason, reason_size)) { downloaded = true; break; }
        ESP_LOGW(TAG, "FOTA download attempt %d/%d failed", attempt, FOTA_MAX_RETRIES);
        if (attempt < FOTA_MAX_RETRIES)
            vTaskDelay(pdMS_TO_TICKS(attempt * 5000));
    }
    if (!downloaded) return false;

    if (!fota_flash(reason, reason_size)) return false;
    return true;  /* Caller publishes fota_success then calls esp_restart() */
}
