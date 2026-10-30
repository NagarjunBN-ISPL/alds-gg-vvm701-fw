#include "fota.h"
#include "modem.h"
#include "network.h"
#include "tls_ca.h"
#include "config.h"
#include "store.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <strings.h>
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

/* Same name as cng-cpil, so a gateway swapped between the products reuses one
 * file. GGY/SR/bwsr (and this firmware before 2026-09-28) used vvm_fw.bin;
 * both names are deleted before every download, so neither product's leftover
 * can fill the modem's ~1.5 MB UFS (140 KB was all that remained on the office
 * unit after a download on 2026-09-28). */
#define FOTA_FILE        "UFS:fw.bin"
#define FOTA_FILE_LEGACY "UFS:vvm_fw.bin"
#define FOTA_CHUNK     2048
#define FOTA_SSL_CTX   1     /* FOTA HTTPS uses SSL ctx 1; MQTT uses ctx 2 */
/* Wall-clock ceiling on the whole UFS->OTA flash loop. The loop is already
 * bounded (each UART read has its own deadline; esp_ota_write fails when the
 * partition fills) and the reads are LOCAL to the modem's UFS, so a real
 * flash finishes in ~1-2 min even for a large image. This cap only exists as
 * belt-and-suspenders for a driver/hardware-level UART hang: the task WDT
 * stays armed through FOTA and each read loop below feeds it, but this
 * wall-clock cap bounds the whole flash independently so a stall aborts
 * cleanly instead of tying up an unattended station indefinitely. */
#define FOTA_FLASH_MAX_MS  300000
/* How long the modem may take to save the body to UFS (QHTTPREADFILE), and
 * how long we wait for its result line: 120 s failed every try at a weak site
 * — ~450 KB at 30 kbit/s is 2 min on its own. 300 s, as btpl-gg (field-proven
 * there); the whole update stays bounded by FOTA_TOTAL_BUDGET_MS. */
#define FOTA_READFILE_WAIT_S 300
/* UFS room asked for beyond the image itself: the filesystem's own overhead
 * (btpl-gg). */
#define FOTA_UFS_MARGIN    (16 * 1024)

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
static void ufs_cleanup(int fh);

/* Set by fota_flash() when the image it just staged is another product's
 * (see image_is_foreign). main.c reads it after fota_perform() returns to
 * decide whether the RAM queue may be spilled to flash before the restart. */
static bool s_last_image_foreign = false;
bool fota_last_image_foreign(void) { return s_last_image_foreign; }

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

/* True when `resp` holds a complete line starting with `marker` — a URC that
 * arrived inside an earlier command's reply (alds-sr). */
static bool urc_line_in(const char *resp, const char *marker) {
    const char *p = strstr(resp, marker);
    return p && strchr(p, '\n') != NULL;
}

/* "+QFLDS: <free>,<total>" -> free bytes, or -1 when the modem did not say. */
static long ufs_free_bytes(void) {
    char resp[128];
    if (!modem_at("AT+QFLDS=\"UFS\"", "OK", 5000, resp, sizeof(resp))) return -1;
    const char *p = strstr(resp, "+QFLDS:");
    long free_b = -1;
    if (!p || sscanf(p, "+QFLDS: %ld", &free_b) != 1 || free_b < 0) return -1;
    return free_b;
}

/* True for a name ending ".bin", any case. */
static bool ufs_name_is_bin(const char *name) {
    size_t n = strlen(name);
    if (n < 4) return false;
    const char *s = name + n - 4;
    return s[0] == '.' && (s[1] | 0x20) == 'b' && (s[2] | 0x20) == 'i' && (s[3] | 0x20) == 'n';
}

/* Makes sure UFS can hold an image of `len` bytes, once its size is known
 * (btpl-gg, field-proven there). Normally a no-op: both FOTA file names were
 * deleted before the download and a clean modem has ~1.5 MB. But a modem that
 * has lived through other products or firmware generations can hold their
 * images under names this firmware never used — the btpl office unit had
 * ~960 KB in use with neither of ours present (2026-10-15), room for a ~580 KB
 * image at most, and a bigger one failed on every attempt. So when the room is
 * short, every *.bin in UFS goes: a firmware image (any product's, any name)
 * or a GNSS assistance cache the modem fetches again — nothing this firmware
 * needs (its CA bundle is a .pem, re-uploaded every boot). The listing is
 * logged first, so what was removed is on record. Still short after that: the
 * update fails HERE, at once and without retries, with a reason that says so
 * ("ufs_full_<free>_of_<need>"), not as a truncated file five tries later. */
static bool ufs_make_room(int len, char *reason, size_t reason_size) {
    const long need = (long)len + FOTA_UFS_MARGIN;
    long free_b = ufs_free_bytes();
    if (free_b < 0 || free_b >= need) return true;   /* unknown: the size check after the download decides */

    ESP_LOGW(TAG, "UFS has %ld bytes free, the image needs %ld — removing leftover .bin files",
             free_b, need);
    static char list[2048];
    /* The reply is read even when the call fails: a listing longer than the
     * buffer never shows its closing OK and times out, yet every whole line
     * that did arrive still names a file. */
    list[0] = '\0';
    modem_at("AT+QFLST=\"*\"", "OK", 5000, list, sizeof(list));
    ESP_LOGW(TAG, "UFS holds: %s", list);
    for (const char *p = strstr(list, "+QFLST: \""); p; p = strstr(p + 1, "+QFLST: \"")) {
        char name[72];
        if (sscanf(p, "+QFLST: \"%71[^\"]\"", name) != 1 || !ufs_name_is_bin(name)) continue;
        char cmd[96];
        /* Some modem builds list names without the "UFS:" drive prefix. */
        snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s%s\"",
                 strncmp(name, "UFS:", 4) == 0 ? "" : "UFS:", name);
        ESP_LOGW(TAG, "deleting %s", name);
        modem_at(cmd, "OK", 5000, NULL, 0);
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
    }
    free_b = ufs_free_bytes();
    if (free_b >= 0 && free_b < need) {
        ESP_LOGE(TAG, "UFS still short after cleanup: %ld bytes free, %ld needed", free_b, need);
        fota_set_reason(reason, reason_size, "ufs_full_%ld_of_%ld", free_b, need);
        return false;
    }
    return true;
}

/* Both image names — this firmware's and the one GGY/SR/bwsr use — so a
 * failed update leaves no ~450 KB file holding the room the next boot's CA
 * upload, or the next update, needs (as alds-sr). An absent file answers
 * "+CME ERROR: 418", which is ignored. */
static void ufs_delete_images(void) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
    modem_at(cmd, "OK", 5000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE_LEGACY);
    modem_at(cmd, "OK", 5000, NULL, 0);
}

/* Downloads to FOTA_FILE; on success *len_out is the Content-Length, which
 * fota_flash() then holds the bytes it wrote against. */
static bool fota_download(const char *url, int *len_out, char *reason, size_t reason_size) {
    char cmd[640], resp[256];
    *len_out = 0;

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

    /* Reset any prior HTTP session state (ignore errors). QHTTPSTOP is the
     * EC200U's only HTTP teardown command — a former QHTTPDISCON call here just
     * returned "+CME ERROR: invalid command line" every FOTA (verified on the
     * live 1.0.1 update, 2026-08-21) because that command does not exist on this
     * modem; removed to keep the priority path clean. */
    modem_at("AT+QHTTPSTOP", "OK", 3000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* Remove old files — this firmware's and the other name (FOTA_FILE_LEGACY
     * above). +CME ERROR: 418 for an absent file is expected and ignored. */
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
    modem_at(cmd, "OK", 5000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE_LEGACY);
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
    if (!modem_at("AT+QHTTPGET=120", "OK", 5000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QHTTPGET rejected");
        fota_set_reason(reason, reason_size, "get_rejected");
        return false;
    }
    /* The result can follow the OK so closely that modem_at() read it already
     * (it drains the rest of the line); waiting for it again then sat out the
     * full 150 s and failed the attempt as get_timeout (alds-sr). */
    if (!urc_line_in(resp, "+QHTTPGET:") &&
        !modem_read_until("+QHTTPGET:", 150000, resp, sizeof(resp))) {
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
    /* Size sanity, as cng-cpil: an error page served with a 200 is a few
     * hundred bytes and would otherwise be written into the OTA slot. */
    if (g_len < FOTA_MIN_IMAGE_BYTES || g_len > FOTA_MAX_IMAGE_BYTES) {
        ESP_LOGE(TAG, "HTTPS GET: implausible image size %d (allowed %d..%d)",
                 g_len, FOTA_MIN_IMAGE_BYTES, FOTA_MAX_IMAGE_BYTES);
        fota_set_reason(reason, reason_size, "bad_length_%d", g_len);
        return false;
    }
    /* Room for it in the modem's UFS, before the body is saved there. */
    if (!ufs_make_room(g_len, reason, reason_size)) return false;
    *len_out = g_len;
    ESP_LOGI(TAG, "HTTP 200 OK (%d bytes) — saving to UFS ...", g_len);

    /* Save the response body to a UFS file, then fota_flash() reads it out. */
    snprintf(cmd, sizeof(cmd), "AT+QHTTPREADFILE=\"%s\",%d", FOTA_FILE, FOTA_READFILE_WAIT_S);
    if (!modem_at(cmd, "OK", 5000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QHTTPREADFILE rejected");
        fota_set_reason(reason, reason_size, "readfile_rejected");
        return false;
    }
    if (!urc_line_in(resp, "+QHTTPREADFILE:") &&   /* same as +QHTTPGET above */
        !modem_read_until("+QHTTPREADFILE:", (FOTA_READFILE_WAIT_S + 10) * 1000,
                          resp, sizeof(resp))) {
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

    /* VERIFY WHAT ACTUALLY LANDED IN UFS. "+QHTTPREADFILE: 0" means the modem
     * finished its transfer, NOT that the whole body reached its filesystem:
     * cng-cpil saw a 462848-byte image report 0 here and then stop delivering
     * at ~303 KB, mid-chunk — that was all the file held. Without this check it
     * presents as a "short chunk" a minute into flashing, after the OTA slot has
     * been part-written, and the retry loop burns four more passes on a file
     * that will never be complete. Free space is logged alongside because "the
     * file is short" and "the filesystem is full" want different answers. */
    modem_at("AT+QFLDS=\"UFS\"", "OK", 5000, resp, sizeof(resp));
    ESP_LOGI(TAG, "UFS space: %s", resp);
    snprintf(cmd, sizeof(cmd), "AT+QFLST=\"%s\"", FOTA_FILE);
    if (modem_at(cmd, "+QFLST:", 5000, resp, sizeof(resp))) {
        int stored = -1;
        p = strstr(resp, "+QFLST:");                /* +QFLST: "UFS:vvm_fw.bin",<size> */
        const char *comma = p ? strchr(p, ',') : NULL;
        if (comma) stored = atoi(comma + 1);
        ESP_LOGI(TAG, "UFS holds %d bytes, HTTP delivered %d", stored, g_len);
        if (stored >= 0 && stored != g_len) {
            ESP_LOGE(TAG, "UFS file is SHORT — %d of %d bytes; refusing to flash a partial image",
                     stored, g_len);
            fota_set_reason(reason, reason_size, "ufs_short_%d", stored);
            return false;
        }
    }
    ESP_LOGI(TAG, "UFS download OK — flashing ...");
    return true;
}

/* ── Cross-product handover ───────────────────────────────────────────────
 * The same VVM701 board moves between LPG and CNG stations and is switched
 * to the other product's firmware by FOTA alone — the partition table, the
 * NVS partition and the littlefs `storage` partition are shared and
 * byte-compatible by design (partitions.csv). What must NOT be shared is the
 * offline queue: records buffered here belong to this station and this
 * product's topic tree, and the incoming firmware would republish every one
 * of them as its own. The image's project_name (CMakeLists project()) is the
 * identity that distinguishes the two products' binaries. */
static bool image_is_foreign(const esp_partition_t *part) {
    const esp_app_desc_t *mine = esp_app_get_description();
    esp_app_desc_t theirs;

    if (mine == NULL || part == NULL ||
        esp_ota_get_partition_description(part, &theirs) != ESP_OK) {
        ESP_LOGW(TAG, "cannot read the new image's identity - assuming it is "
                      "this product and keeping the queued records");
        return false;
    }

    const bool foreign = strncmp(mine->project_name, theirs.project_name,
                                 sizeof(theirs.project_name)) != 0;

    if (foreign) {
        ESP_LOGW(TAG,
                 "the new image is '%.*s', this one is '%.*s' - a "
                 "cross-product handover",
                 (int)sizeof(theirs.project_name), theirs.project_name,
                 (int)sizeof(mine->project_name), mine->project_name);
    }
    return foreign;
}

/* ── Which images this gateway will take ──────────────────────────────────
 * Every VVM701 product — alds-gg, alds-ggy, alds-iocl, btpl-gg, cng-cpil,
 * bwsr-neo, hsd-snf, project name "<product>-vvm701-fw" — shares one partition
 * table, bootloader configuration and flash layout, so any of them may replace
 * any other by FOTA, in every direction. Anything else (an Arduino or
 * PlatformIO build, whose partition table sits at 0x8000; a test image; the
 * wrong file) would be booted by this board's bootloader and fail to find its
 * partition table: a rollback at best, a site visit at worst. alds-sr is a
 * VVM701 product but runs secure boot, outside the swap set, and is refused.
 *
 * A NAME PATTERN, not a list, on purpose: a product added later is accepted by
 * every gateway already in the field without updating them first. The same
 * rule, word for word, is in every VVM701 product's FOTA. */
#define FOTA_IMAGE_SUFFIX   "-vvm701-fw"
#define FOTA_IMAGE_REFUSED  "alds-sr-vvm701-fw"

static bool fota_image_allowed(const char *name, size_t name_size) {
    char   n[64];
    size_t sl = strlen(FOTA_IMAGE_SUFFIX);
    if (!name || name_size == 0) return false;
    int len = snprintf(n, sizeof(n), "%.*s", (int)name_size, name);
    if (len <= 0 || (size_t)len >= sizeof(n) || (size_t)len <= sl) return false;
    if (strcmp(n + (size_t)len - sl, FOTA_IMAGE_SUFFIX) != 0) return false;
    return strcmp(n, FOTA_IMAGE_REFUSED) != 0;
}

/* ── UFS → OTA partition ─────────────────────────────────────────────── */
static bool fota_flash(int expected, bool allow_handover, char *reason, size_t reason_size) {
    char cmd[128], hdr[128];

    /* Open file */
    snprintf(cmd, sizeof(cmd), "AT+QFOPEN=\"%s\",0", FOTA_FILE);
    /* 512, not 64: modem_at() copies the reply from the START of what it read,
     * so 63 bytes of unrelated URCs ahead of "+QFOPEN:" pushed the answer out
     * of a small buffer — file_open_parse with the file left open (its handle
     * unknown, so leaked until the next modem reset). As alds-sr. */
    char resp[512] = {0};
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
        ufs_cleanup(fh);
        return false;
    }

    /* ── Erase what the image needs, not the whole slot (alds-sr, cng-cpil)
     * OTA_SIZE_UNKNOWN erases the ENTIRE 7 MB partition inside one call that
     * cannot feed this task's watchdog: tens of seconds at typical 64 KB block
     * erase times, and flash erase slows with temperature and wear, so a unit
     * that passes on the bench gets closer to the 120 s task WDT in a hot panel
     * box. The size is known exactly here — fota_download() took it from
     * Content-Length, bounded it to FOTA_MIN/MAX_IMAGE_BYTES and checked the
     * UFS file holds exactly that many bytes — so only that much is erased
     * (esp_ota_begin rounds it up to a 4 KB sector): about a second for a
     * ~450 KB image. The write loop below never goes past it. */
    const size_t erase_len = (expected > 0) ? (size_t)expected : (size_t)OTA_SIZE_UNKNOWN;
    esp_ota_handle_t ota_hdl;
    if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
    if (esp_ota_begin(update_part, erase_len, &ota_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed");
        fota_set_reason(reason, reason_size, "ota_begin_failed");
        ufs_cleanup(fh);
        return false;
    }
    if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();

    static uint8_t chunk[FOTA_CHUNK];
    bool ok = true;
    int  written = 0;
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
                /* Compact instead of freezing a full buffer: if an interleaved
                 * URC (+QMTSTAT/+QIURC/…) or line noise fills hdr before CONNECT
                 * arrives, a fixed accumulator would stop appending and the
                 * CONNECT that follows would never match. Keep only the last
                 * KEEP bytes — comfortably more than a whole "CONNECT <len>\r\n"
                 * — so the header can always still be matched. */
                if (hi >= (int)sizeof(hdr) - 1) {
                    const int KEEP = 24;
                    memmove(hdr, hdr + hi - KEEP, KEEP);
                    hi = KEEP;
                }
                hdr[hi++] = ch;
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

        /* Never past the span esp_ota_begin() erased: programming flash that
         * was not erased corrupts it silently. A file longer than the length
         * the download reported is wrong anyway (alds-sr, cng-cpil). */
        if (expected > 0 && written + got > expected) {
            ESP_LOGE(TAG, "image runs past its reported %d bytes — aborting", expected);
            fota_set_reason(reason, reason_size, "size_mismatch_%d", written + got);
            ok = false;
            break;
        }

        if (esp_ota_write(ota_hdl, chunk, got) != ESP_OK) {
            ESP_LOGE(TAG, "OTA write error");
            fota_set_reason(reason, reason_size, "ota_write_failed");
            ok = false;
            break;
        }
        written += got;

        /* Consume trailing OK — modem sends it automatically, no TX needed */
        modem_read_until("OK", 5000, NULL, 0);

        if (actual < FOTA_CHUNK) break; /* Last chunk */
    }

    /* Every downloaded byte must have reached the slot, as cng-cpil checks: the
     * loop also ends on a missing CONNECT (end of file), so a transfer that
     * stopped early would otherwise be caught only by esp_ota_end(), and the
     * reason would not say how far it got. */
    if (ok && expected > 0 && written != expected) {
        ESP_LOGE(TAG, "size mismatch: downloaded %d bytes, wrote %d", expected, written);
        fota_set_reason(reason, reason_size, "size_mismatch_%d", written);
        ok = false;
    }

    /* Close and delete file */
    ufs_cleanup(fh);

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

    /* Before the boot slot is committed: a refused image is written but never
     * booted, and the running firmware stays exactly as it was. */
    {
        esp_app_desc_t desc;
        memset(&desc, 0, sizeof(desc));
        if (esp_ota_get_partition_description(update_part, &desc) != ESP_OK ||
            !fota_image_allowed(desc.project_name, sizeof(desc.project_name))) {
            ESP_LOGE(TAG, "the image is '%.*s' — not a VVM701 swap-set firmware; refused, "
                          "nothing changed", (int)sizeof(desc.project_name), desc.project_name);
            fota_set_reason(reason, reason_size, "not_a_vvm701_image");
            return false;
        }
    }

    /* Another product's firmware only when the push asked for it. A handover
     * wipes this station's configuration and moves the unit to the other
     * product's topics, so a wrong file picked on the cloud must not be able
     * to do that by accident; a deliberate swap says "handover":true. Refused
     * here, before the boot slot is switched: written, never booted. */
    const bool foreign = image_is_foreign(update_part);
    if (foreign && !allow_handover) {
        ESP_LOGE(TAG, "the image is another product's and the push did not say "
                      "\"handover\":true — refused, nothing changed");
        fota_set_reason(reason, reason_size, "not_this_product");
        return false;
    }

    if (esp_ota_set_boot_partition(update_part) != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed");
        fota_set_reason(reason, reason_size, "set_boot_failed");
        return false;
    }

    /* Only once the boot slot is committed — a purge before a failed
     * set_boot would throw away a backlog for an update that never happens. */
    s_last_image_foreign = foreign;
    if (s_last_image_foreign) {
        /* HAND OVER AN EMPTY, UNCLAIMED QUEUE. store_disown() drops the owner
         * marker so that when this gateway comes back to this product — by
         * whichever firmware performs that trip — store_init() discards what
         * the other product left rather than republishing it. du_task keeps
         * polling until the restart; main.c purges once more right before
         * esp_restart() and skips its usual RAM->flash spill for that reason. */
        if (store_purge())
            ESP_LOGW(TAG, "cross-product handover — discarded the offline queue; those "
                          "readings belong to this deployment, not the incoming firmware's");
        else
            ESP_LOGE(TAG, "cross-product handover — could NOT purge the offline queue");
        store_disown();
    }
    ESP_LOGI(TAG, "OTA flash complete");
    return true;
}

/* ── Retry policy ─────────────────────────────────────────────────────────
 * BOTH stages retry, and the flash stage is inside the loop. alds-sr's first
 * field update took four attempts: two socket failures on the download, then
 * one short read and one corrupt image on the transfer back out of the modem
 * — all transient, a further try succeeded with nothing changed. So a
 * `read_short` or a failed esp_ota_end() costs one more pass, not an operator
 * noticing fota_failed and re-pushing the URL. Re-downloading rather than
 * re-flashing the file already in UFS is deliberate: a corrupt transfer does
 * not say WHERE the corruption came from, and re-reading a file that is
 * itself wrong fails identically every time.
 *
 * modem_hard_reset() before attempt 3 AND the last: it costs ~20 s and tears
 * down the PDP context, so it is not worth doing after the first hiccup, but
 * in-modem faults (err=729 memory allocation, err=716 socket) persist across
 * HTTP sessions and only clear with a baseband restart.
 *
 * Two things bound the whole thing. FOTA_TOTAL_BUDGET_MS caps the wall clock:
 * MQTT is torn down for the duration, du_task is buffering to flash, and the
 * queue needs the broker back more than this update needs a fifth try on a
 * link that has failed four times. And a reason that cannot change between
 * attempts aborts at once: no OTA partition, an over-long QFREAD command, an
 * image that is not a VVM701 swap-set firmware or is another product's without
 * "handover":true, a body whose size cannot be a firmware image or does not
 * fit the modem's UFS even after its leftover .bin files went, and an HTTP
 * 403 or 404 — an expired or refused signed link, a file that is not there,
 * the same answer on every try. Everything else (network, other HTTP
 * statuses, short read, verify) is retried. */
#define FOTA_ATTEMPTS        5
#define FOTA_TOTAL_BUDGET_MS (15 * 60 * 1000)

static bool fota_reason_is_final(const char *reason) {
    return reason && (strcmp(reason, "no_ota_partition")   == 0 ||
                      strcmp(reason, "read_cmd_too_long")  == 0 ||
                      strcmp(reason, "not_a_vvm701_image") == 0 ||
                      strcmp(reason, "not_this_product")   == 0 ||
                      strncmp(reason, "ufs_full_", 9)      == 0 ||
                      strncmp(reason, "bad_length_", 11)   == 0 ||
                      strncmp(reason, "http_403_", 9)      == 0 ||
                      strncmp(reason, "http_404_", 9)      == 0);
}

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

static void ufs_cleanup(int fh) {
    char cmd[128];
    if (fh >= 0) {
        snprintf(cmd, sizeof(cmd), "AT+QFCLOSE=%d", fh);
        modem_at(cmd, "OK", 5000, NULL, 0);
    }
    snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", FOTA_FILE);
    modem_at(cmd, "OK", 5000, NULL, 0);
}

bool fota_perform(const char *url, const char *pub_topic,
                  char *reason, size_t reason_size, bool allow_handover) {
    if (reason && reason_size) reason[0] = '\0';

    /* Sized from FOTA_URL_MAX so the cap the caller enforces and the buffer
     * that enforces it can never drift apart. */
    static char clean_url[FOTA_URL_MAX + 1];
    strncpy(clean_url, url, sizeof(clean_url) - 1);
    clean_url[sizeof(clean_url) - 1] = '\0';
    sanitize_url(clean_url);

    /* HTTPS only. "http" (4 chars) would also match "http://" — accepting
     * plaintext FOTA means the firmware binary itself travels unencrypted
     * and unauthenticated, letting anyone on the network path (or a
     * compromised/spoofed server) hand the device arbitrary code to flash
     * and boot. We only ever generate/expect https:// URLs. The scheme is
     * matched in any case ("HTTPS://" is the same link) and handed to the
     * modem in lower case, the form its HTTP stack is proven with. */
    if (strncasecmp(clean_url, "https://", 8) != 0) {
        ESP_LOGE(TAG, "FOTA: rejecting non-HTTPS URL [%s]", clean_url);
        fota_set_reason(reason, reason_size, "non_https_url");
        return false;
    }
    memcpy(clean_url, "https://", 8);
    ESP_LOGI(TAG, "FOTA start URL=[%s] topic=%s", clean_url, pub_topic);

    s_last_image_foreign = false;
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)FOTA_TOTAL_BUDGET_MS * 1000LL;

    for (int attempt = 1; attempt <= FOTA_ATTEMPTS; attempt++) {
        if (attempt > 1 && esp_timer_get_time() > deadline_us) {
            ESP_LOGE(TAG, "FOTA: giving up after %d min — the queue needs the broker back "
                          "more than this update needs another try", FOTA_TOTAL_BUDGET_MS / 60000);
            fota_set_reason(reason, reason_size, "budget_exhausted");
            break;
        }
        if (attempt == 3 || attempt == FOTA_ATTEMPTS) {
            ESP_LOGW(TAG, "FOTA: escalating — power-cycling modem before attempt %d", attempt);
            modem_hard_reset();
            esp_task_wdt_reset();
            /* A failed rebuild no longer ends the update: as cng-cpil does, the
             * attempt is spent anyway — the network may come up during it, and
             * a download that still cannot connect fails with its own reason. */
            if (!network_init())
                ESP_LOGE(TAG, "FOTA: network rebuild failed after modem reset — "
                              "spending this attempt anyway");
            /* The download verifies S3's certificate against the CA file in
             * the modem's UFS. Re-provisioning after a power-cycle is cheap
             * insurance against a module that came back without it. */
            tls_ca_provision();
            esp_task_wdt_reset();
        }

        int len = 0;
        if (fota_download(clean_url, &len, reason, reason_size) &&
            fota_flash(len, allow_handover, reason, reason_size))
            return true;

        ESP_LOGW(TAG, "FOTA attempt %d/%d failed (%s)", attempt, FOTA_ATTEMPTS,
                 (reason && reason[0]) ? reason : "?");
        if (fota_reason_is_final(reason)) {
            ESP_LOGE(TAG, "FOTA: '%s' cannot change between attempts — not retrying", reason);
            break;
        }
        if (attempt < FOTA_ATTEMPTS) {
            /* Sliced so the WDT is fed through the back-off. */
            for (int w = 0; w < attempt * 5; w++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_task_wdt_reset();
            }
        }
    }
    /* The update failed: leave no image file behind. Several failures (a short
     * UFS file, a read error, a QFOPEN that failed) return without deleting
     * it, and the caller re-checks the CA file right after this, which needs
     * the room (alds-sr). */
    ufs_delete_images();
    return false;
}
