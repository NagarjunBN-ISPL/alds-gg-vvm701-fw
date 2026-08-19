#include "mqtt.h"
#include "modem.h"
#include "tls_ca.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "mqtt";

/* Inbound-command queue depth. Quectel auto-ACKs a QoS-1 message to the broker
 * when it emits the +QMTRECV URC, so a message dropped here (stash full) is NOT
 * redelivered — it's gone at the MQTT layer (the cloud can still retry at the
 * app layer, e.g. re-sending an rsp/kfactor that got no response). 16 covers a
 * realistic worst-case burst (a fleet k-factor push hitting all 12 nozzles at
 * once) with margin; parse_qmtrecv logs if it ever fills. Each slot is
 * TOPIC_MAX+PAYLOAD_MAX ≈ 2.3 KB of BSS. */
#define STASH_MAX    16
#define TOPIC_MAX    256
#define PAYLOAD_MAX  2048

typedef struct {
    char topic[TOPIC_MAX];
    char payload[PAYLOAD_MAX];
} MqttMsg_t;

/* Inbound-command queue as a circular buffer: push at (head + cnt) % STASH_MAX,
 * pop at head. Avoids the O(n) array shift a linear queue does on every pop —
 * each MqttMsg_t is ~2.3 KB, so draining a full 16-deep stash the linear way
 * memmoved ~34 KB per pop. Single-task access (parse_qmtrecv pushes, mqtt_poll
 * pops, both on the main task), so no locking is needed. */
static MqttMsg_t g_stash[STASH_MAX];
static int       g_stash_head = 0;   /* index of the oldest queued message */
static int       g_stash_cnt  = 0;   /* number of messages queued          */
static int       g_msgid      = 1;
static bool      g_connected  = false;
static char      g_client_id[80];
static int       s_pub_fail_streak = 0;   /* consecutive failed publishes */

/* Broker credentials — default to the compiled-in config.h values so the
 * device still connects if mqtt_set_credentials() is never called (e.g.
 * before nvs_config_load() exists). app_main() overrides these from
 * DeviceConfig right after loading NVS. */
static char     s_mqtt_host[64] = MQTT_HOST;
static uint16_t s_mqtt_port     = MQTT_PORT;
static char     s_mqtt_user[32] = MQTT_USER;
static char     s_mqtt_pass[64] = MQTT_PASS;
static bool     s_mqtt_tls      = true;

void mqtt_set_credentials(const char *host, uint16_t port,
                           const char *user, const char *pass, bool tls) {
    strncpy(s_mqtt_host, host, sizeof(s_mqtt_host) - 1);
    s_mqtt_host[sizeof(s_mqtt_host) - 1] = '\0';
    s_mqtt_port = port;
    strncpy(s_mqtt_user, user, sizeof(s_mqtt_user) - 1);
    s_mqtt_user[sizeof(s_mqtt_user) - 1] = '\0';
    strncpy(s_mqtt_pass, pass, sizeof(s_mqtt_pass) - 1);
    s_mqtt_pass[sizeof(s_mqtt_pass) - 1] = '\0';
    s_mqtt_tls = tls;
}

const char *mqtt_get_pass(void) { return s_mqtt_pass; }

static void (*s_activity_cb)(void) = NULL;
void mqtt_set_activity_cb(void (*cb)(void)) { s_activity_cb = cb; }

static int next_msgid(void) {
    int id = g_msgid++;
    if (g_msgid > 65535) g_msgid = 1;
    return id;
}

/* ── Detect +QMTSTAT URCs (connection-lost notification) ─────────────────
 * Quectel sends "+QMTSTAT: 0,<code>" (code != 0) when the MQTT link drops
 * (broker gone, keepalive timeout, network loss). If we don't notice, we
 * keep thinking we're connected — so mqtt_reconnect_if_needed() never fires,
 * and any setconfig/FOTA the broker has queued for our persistent session
 * is never delivered. Scan every buffer read from the modem so it's caught
 * even mid-publish/subscribe. This was the missing piece vs. the known-good
 * build: a silently-dropped link left setconfig stuck in the broker queue. */
static void check_qmtstat(const char *buf) {
    const char *p = strstr(buf, "+QMTSTAT:");
    if (!p) return;
    p += 9;
    while (*p == ' ') p++;
    p = strchr(p, ',');
    if (p && atoi(p + 1) != 0) {
        if (g_connected) {
            ESP_LOGW(TAG, "QMTSTAT: connection lost — marking link down");
            g_connected = false;
        }
    }
}

/* Skip past '+QMTRECV: <id>,<msgid>,"<topic>",' to the payload field.
 * Returns NULL if that much of the line hasn't arrived over UART yet. */
static const char *qmtrecv_payload_start(const char *buf,
                                          const char **topic_start_out,
                                          size_t *topic_len_out) {
    const char *p = strstr(buf, "+QMTRECV:");
    if (!p) return NULL;
    p += 9;
    while (*p == ' ') p++;
    if (*p != '0') return NULL;                  /* link_id must be 0 */
    p++;
    if (*p != ',') return NULL;
    p++;
    while (*p == ' ') p++;
    while (*p >= '0' && *p <= '9') p++;           /* optional msgid digits */
    if (*p == ',') {                              /* msgid field present (even if empty) */
        p++;
        while (*p == ' ') p++;
    }
    if (*p != '"') return NULL;
    p++;
    const char *topic_start = p;
    while (*p && *p != '"') p++;
    if (*p != '"') return NULL;                   /* topic not fully arrived yet */
    size_t topic_len = (size_t)(p - topic_start);
    p++;
    if (*p != ',') return NULL;
    p++;
    while (*p == ' ') p++;
    if (topic_start_out) *topic_start_out = topic_start;
    if (topic_len_out)   *topic_len_out   = topic_len;
    return p;
}

/* Find the end of a +QMTRECV payload starting at p. Quectel wraps the
 * payload in quotes like the topic, and does NOT escape/strip literal
 * newline bytes inside it — a pretty-printed JSON payload arrives with
 * real '\n' characters embedded, so the end can't be found via strchr('\n').
 * Instead: skip the wrapping quote if present, then if the payload looks
 * like JSON, track brace depth (respecting quoted strings) to find its
 * true close. Returns NULL if the data captured so far is incomplete. */
static const char *qmtrecv_payload_end(const char *p) {
    bool quoted = (*p == '"');
    if (quoted) p++;

    if (*p == '{') {
        int  depth  = 0;
        bool in_str = false;
        const char *q = p;
        for (; *q; q++) {
            if (in_str) {
                if (*q == '\\' && q[1]) { q++; continue; }
                if (*q == '"') in_str = false;
                continue;
            }
            if (*q == '"') { in_str = true; continue; }
            if (*q == '{') depth++;
            else if (*q == '}') {
                depth--;
                if (depth == 0) { q++; break; }
            }
        }
        if (depth != 0) return NULL;              /* braces never balanced yet */
        if (quoted) {
            /* A pretty-printed JSON payload ends with "}\r\n" before the
             * modem's closing wrapper quote — skip that trailing whitespace,
             * otherwise the quote check fails and the whole message (e.g.
             * a multi-line setconfig) is silently dropped. */
            while (*q == ' ' || *q == '\r' || *q == '\n' || *q == '\t') q++;
            if (*q != '"') return NULL;            /* closing wrapper quote pending */
            q++;
        }
        return q;
    }

    if (quoted) {
        const char *q = p;
        for (; *q; q++) {
            if (*q == '\\' && q[1]) { q++; continue; }
            if (*q == '"') return q + 1;
        }
        return NULL;
    }
    return strchr(p, '\n');
}

/* True if an inbound payload carries a connectivity credential (broker or APN
 * user/pass, all settable via setconfig). Used ONLY to redact the RX log below
 * so a pushed secret never lands on the serial console in cleartext — the
 * payload itself is stashed and parsed untouched, so behaviour is identical. */
static bool payload_has_secret(const char *p) {
    return strstr(p, "\"mqttUser\"") || strstr(p, "\"mqttPass\"") ||
           strstr(p, "\"apnUser\"")  || strstr(p, "\"apnPass\"");
}

/* ── Parse +QMTRECV URCs from a buffer, push into stash ──────────────── */
static void parse_qmtrecv(const char *buf) {
    if (!buf) return;
    check_qmtstat(buf);   /* also scan for connection-lost notifications */

    const char *search_from = buf;
    for (;;) {
        if (g_stash_cnt >= STASH_MAX) {
            /* Stash full: any further complete URCs in this buffer get walked
             * past (and dropped) by drain_recv's resume logic. Surface it —
             * a silent drop of an inbound command is otherwise invisible. */
            if (strstr(search_from, "+QMTRECV:"))
                ESP_LOGW(TAG, "stash full (%d) — inbound command(s) may be dropped", STASH_MAX);
            break;
        }
        const char *topic_start; size_t topic_len;
        const char *p = qmtrecv_payload_start(search_from, &topic_start, &topic_len);
        if (!p) break;                             /* nothing (more) usable yet */

        bool quoted = (*p == '"');
        const char *content_start = quoted ? p + 1 : p;
        const char *content_stop  = qmtrecv_payload_end(p);
        if (!content_stop) break;                  /* payload still streaming in */
        const char *content_end = quoted ? content_stop - 1 : content_stop;
        size_t pl_len = (size_t)(content_end - content_start);
        if (pl_len > 0 && content_start[pl_len - 1] == '\r') pl_len--;

        MqttMsg_t *m = &g_stash[(g_stash_head + g_stash_cnt) % STASH_MAX];
        g_stash_cnt++;
        size_t tl = (topic_len < TOPIC_MAX - 1)   ? topic_len : TOPIC_MAX - 1;
        size_t pl = (pl_len   < PAYLOAD_MAX - 1)  ? pl_len   : PAYLOAD_MAX - 1;
        memcpy(m->topic,   topic_start,   tl); m->topic[tl]   = '\0';
        memcpy(m->payload, content_start, pl); m->payload[pl] = '\0';
        if (payload_has_secret(m->payload))
            ESP_LOGI(TAG, "RX stashed: topic=[%s] payload=[redacted — contains credentials]", m->topic);
        else
            ESP_LOGI(TAG, "RX stashed: topic=[%s] payload=[%s]", m->topic, m->payload);

        search_from = content_stop;
    }
}

static void drain_recv(int timeout_ms) {
    static char buf[PAYLOAD_MAX + 300];
    /* Persists across calls — see the resume-point logic at the end of this
     * function. Resetting this to 0 on every call (as it used to be) meant a
     * "+QMTRECV:" header split across two drain_recv() calls by unlucky UART
     * timing (e.g. the 9-byte marker starts arriving right as the soft
     * deadline below fires) had its already-read prefix silently overwritten
     * by the next call's fresh reads, permanently losing that remote command —
     * uart_read_bytes() consumes bytes from the driver's ring buffer, so
     * there's no way to "re-read" them once dropped here. */
    static int total = 0;
    int64_t dl      = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    int64_t hard_dl = esp_timer_get_time() + 2000000LL;  /* absolute safety cap */

    while (total < (int)sizeof(buf) - 1) {
        int64_t now      = esp_timer_get_time();
        const char *pstart = qmtrecv_payload_start(buf, NULL, NULL);
        if (pstart && qmtrecv_payload_end(pstart)) break;  /* full URC captured */
        if (now >= hard_dl) break;                          /* safety cap hit   */
        if (now >= dl && !pstart) break;                     /* nothing arrived  */

        /* Once the marker/topic has started, keep reading past the nominal
         * deadline until the payload completes — a partial payload silently
         * dropped is worse than a slightly late one. */
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            buf[total++] = ch;
            buf[total]   = '\0';
        }
    }
    if (total > 0) parse_qmtrecv(buf);

    /* Determine the resume point exactly the way parse_qmtrecv() itself walks
     * the buffer: skip over every COMPLETE URC it already consumed, and if
     * what's left is a URC whose header arrived but payload hasn't finished,
     * shift that partial tail down to offset 0 and keep accumulating onto it
     * next call. If nothing resembles a URC start at all, the buffer is just
     * stale AT chatter — safe to discard. */
    const char *search_from = buf;
    const char *incomplete_start = NULL;
    for (;;) {
        const char *hdr = strstr(search_from, "+QMTRECV:");
        if (!hdr) break;
        const char *pstart = qmtrecv_payload_start(search_from, NULL, NULL);
        if (!pstart) break;
        const char *pend = qmtrecv_payload_end(pstart);
        if (!pend) { incomplete_start = hdr; break; }
        search_from = pend;
    }

    size_t keep = incomplete_start ? strlen(incomplete_start) : 0;
    if (incomplete_start && keep < sizeof(buf) - 100) {
        /* Genuinely incomplete and still comfortably within capacity —
         * preserve it so the next call resumes accumulating from here. */
        memmove(buf, incomplete_start, keep + 1);
        total = (int)keep;
    } else {
        total = 0;
    }
}

/* ── Public API ───────────────────────────────────────────────────────── */

bool mqtt_init(const char *imei) {
    snprintf(g_client_id, sizeof(g_client_id), "iotronics-%s", imei);
    g_stash_head = 0;
    g_stash_cnt  = 0;
    g_connected = false;
    g_msgid     = 1;
    /* Rescue +QMTRECV: URCs that would otherwise be silently discarded by
     * modem_flush() at the start of any unrelated AT command. */
    modem_set_flush_rescue(parse_qmtrecv);
    return true;
}

bool mqtt_connect(void) {
    static char cmd[512], resp[512];

    modem_at("AT+QMTCLOSE=0", "OK", 5000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* TLS + persistent session — both must be set before QMTOPEN, since
     * Quectel rejects QMTCFG/QSSLCFG changes once the link is already open. */
    if (s_mqtt_tls) {
        /* seclevel=1: verify the broker's certificate against the CA
         * uploaded by tls_ca_provision() (called once at boot, before the
         * first connect attempt) — was seclevel=0 (no verification at all)
         * until 2026-07-17. cacert is re-asserted on every connect attempt,
         * same defensive "don't assume it survived" posture already used
         * for sslversion/ciphersuite below, since QSSLCFG settings are reset
         * by some recovery paths (e.g. modem_hard_reset) that this function
         * gets called again after. QMTCFG "ssl",0,1,2 = client 0, SSL on,
         * SSL context 2. */
        modem_at("AT+QSSLCFG=\"sslversion\",2,4",       "OK", 3000, NULL, 0);
        modem_at("AT+QSSLCFG=\"ciphersuite\",2,0XFFFF", "OK", 3000, NULL, 0);
        {
            char cacert_cmd[64];
            snprintf(cacert_cmd, sizeof(cacert_cmd), "AT+QSSLCFG=\"cacert\",2,\"%s\"", TLS_CA_FILE);
            modem_at(cacert_cmd, "OK", 3000, NULL, 0);
        }
        modem_at("AT+QSSLCFG=\"seclevel\",2,1",         "OK", 3000, NULL, 0);
        modem_at("AT+QMTCFG=\"ssl\",0,1,2",             "OK", 3000, NULL, 0);
    } else {
        /* Plaintext MQTT — credentials and telemetry travel unencrypted.
         * Only for brokers that don't offer TLS; guarded remotely by the
         * connectivity rollback (a wrong toggle can't reach a session and
         * self-reverts). QMTCFG "ssl",0,0 = SSL off on client 0. */
        ESP_LOGW(TAG, "MQTT TLS disabled — connecting in plaintext");
        modem_at("AT+QMTCFG=\"ssl\",0,0", "OK", 3000, NULL, 0);
    }
    modem_at("AT+QMTCFG=\"keepalive\",0,60",        "OK", 3000, NULL, 0);

    /* Persistent session: broker queues QoS-1 messages while device is offline.
     * Quectel's AT command set calls this "session", not "clean". */
    modem_at("AT+QMTCFG=\"session\",0,0", "OK", 5000, NULL, 0);

    /* Open TCP/TLS connection to broker */
    snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d", s_mqtt_host, s_mqtt_port);
    if (!modem_at(cmd, "+QMTOPEN:", 30000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QMTOPEN failed: %s", resp);
        return false;
    }
    /* +QMTOPEN: 0,<result>  — result 0 = success */
    char *p = strstr(resp, "+QMTOPEN:");
    if (!p) return false;
    p += 9;
    while (*p == ' ') p++;
    p = strchr(p, ','); if (!p) return false;
    if (atoi(p + 1) != 0) {
        ESP_LOGE(TAG, "QMTOPEN result: %s", resp);
        return false;
    }

    /* MQTT CONNECT */
    snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
             g_client_id, s_mqtt_user, s_mqtt_pass);
    if (!modem_at(cmd, "+QMTCONN:", 30000, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "QMTCONN failed: %s", resp);
        return false;
    }
    /* +QMTCONN: 0,<result>,<ret_code>  — both 0 = success */
    p = strstr(resp, "+QMTCONN:");
    if (!p) return false;
    p += 9;
    while (*p == ' ') p++;
    p = strchr(p, ','); if (!p) return false; p++;  /* skip link_id */
    int rc1 = atoi(p);
    p = strchr(p, ','); if (!p) return false; p++;
    int rc2 = atoi(p);
    if (rc1 != 0 || rc2 != 0) {
        ESP_LOGE(TAG, "QMTCONN rc1=%d rc2=%d", rc1, rc2);
        return false;
    }

    g_connected = true;
    s_pub_fail_streak = 0;
    ESP_LOGI(TAG, "MQTT connected, client=%s", g_client_id);
    return true;
}

bool mqtt_subscribe(const char *topic) {
    char cmd[512], resp[256];
    int mid = next_msgid();
    snprintf(cmd, sizeof(cmd), "AT+QMTSUB=0,%d,\"%s\",%d", mid, topic, MQTT_QOS);
    if (!modem_at(cmd, "+QMTSUB:", 10000, resp, sizeof(resp))) {
        ESP_LOGW(TAG, "Subscribe failed: %s", topic);
        return false;
    }
    ESP_LOGI(TAG, "Subscribed: %s", topic);
    return true;
}

/* Single accounting point for EVERY publish outcome. The streak counter is
 * the backstop for a silently-dead link (see the comment at the QMTPUBEX-ack
 * wait below) — but it only works if every failure mode feeds it. The two
 * early-out failures (modem ERROR before the '>' prompt, and no prompt at
 * all) used to return without counting: a modem whose MQTT session died in a
 * way that rejects QMTPUBEX outright — with the +QMTSTAT URC lost — would
 * fail every publish forever while g_connected stayed true, so the reconnect
 * ladder never fired and the device went silent indefinitely — there is no
 * scheduled on-device reboot to bail it out (that was removed; see config.h). */
static bool pub_result(bool ok, const char *topic) {
    if (ok) {
        s_pub_fail_streak = 0;
        return true;
    }
    ESP_LOGW(TAG, "Publish failed for %s (streak %d/%d)",
             topic, s_pub_fail_streak + 1, MQTT_PUB_FAIL_MAX);
    if (++s_pub_fail_streak >= MQTT_PUB_FAIL_MAX) {
        ESP_LOGE(TAG, "%d consecutive publish failures — marking link DOWN",
                 s_pub_fail_streak);
        g_connected = false;
        s_pub_fail_streak = 0;
    }
    return false;
}

bool mqtt_publish(const char *topic, const char *payload) {
    if (!g_connected) return false;

    if (s_activity_cb) s_activity_cb();   /* one LED blink per push */

    int  mid = next_msgid();
    int  len = strlen(payload);
    /* Backstop: the modem's QMTPUBEX ceiling is ~1500 B. Every builder already
     * self-caps under MQTT_MODEM_PAYLOAD_SAFE_MAX (falling back to a 404 shape),
     * so reaching this is an upstream bug — drop it rather than hand the modem a
     * doomed command it would reject or truncate into invalid JSON. Returns
     * false WITHOUT touching the dead-link streak: the link is fine, not the
     * payload. */
    if (len > MQTT_MODEM_PAYLOAD_SAFE_MAX) {
        ESP_LOGE(TAG, "publish %s: payload %d B over safe max %d — dropped",
                 topic, len, MQTT_MODEM_PAYLOAD_SAFE_MAX);
        return false;
    }
    /* Full topic + payload at INFO — field visibility into exactly what goes
     * to the broker on every publish, requested explicitly over the earlier
     * DEBUG-only tradeoff (that one existed to cap UART0 console volume;
     * this supersedes it for active field validation). */
    ESP_LOGI(TAG, "PUBLISH %s (%d bytes): %s", topic, len, payload);
    static char cmd[512];
    snprintf(cmd, sizeof(cmd), "AT+QMTPUBEX=0,%d,%d,0,\"%s\",%d", mid, MQTT_QOS, topic, len);

    /* Send command, wait for '>' prompt, rescue any +QMTRECV URCs */
    modem_flush();
    /* Same snprintf-return-is-not-what-fit hazard guarded in modem_at() —
     * this path bypasses modem_at() and writes the UART directly, so it needs
     * its own bounds check or an over-long command reads past `full` and
     * transmits adjacent RAM. */
    static char full[600];
    int n = snprintf(full, sizeof(full), "%s\r\n", cmd);
    if (n < 0 || n >= (int)sizeof(full)) {
        ESP_LOGE(TAG, "publish cmd too long (%d bytes, max %d) — dropped",
                 n, (int)sizeof(full) - 1);
        return pub_result(false, topic);
    }
    uart_write_bytes(MODEM_UART, full, n);

    static char buf[PAYLOAD_MAX + 300];
    memset(buf, 0, sizeof(buf));
    int  bi = 0;
    bool got_prompt = false;
    int64_t dl = esp_timer_get_time() + 10000000LL;

    while (esp_timer_get_time() < dl) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(20)) > 0) {
            if (bi < (int)sizeof(buf) - 1) { buf[bi++] = (char)ch; buf[bi] = '\0'; }
            if (ch == '>') { got_prompt = true; break; }
            if (strstr(buf, "ERROR")) {
                parse_qmtrecv(buf);
                return pub_result(false, topic);
            }
        }
    }
    parse_qmtrecv(buf);  /* rescue any URCs that arrived before > */

    if (!got_prompt) {
        ESP_LOGW(TAG, "No > prompt for %s", topic);
        return pub_result(false, topic);
    }

    /* Send payload bytes */
    uart_write_bytes(MODEM_UART, payload, len);

    /* Wait for +QMTPUBEX: URC, rescuing +QMTRECV meanwhile */
    memset(buf, 0, sizeof(buf));
    bi = 0;
    bool pub_ok = false;
    dl = esp_timer_get_time() + 15000000LL;
    while (esp_timer_get_time() < dl) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(20)) > 0) {
            if (bi < (int)sizeof(buf) - 1) { buf[bi++] = (char)ch; buf[bi] = '\0'; }
            if (strstr(buf, "+QMTPUBEX:") && strchr(strstr(buf, "+QMTPUBEX:"), '\n')) {
                char *p = strstr(buf, "+QMTPUBEX:");
                p += 10;
                while (*p == ' ') p++;
                p = strchr(p, ','); if (p) p++;  /* skip link_id */
                if (p) p = strchr(p, ',');
                if (p) p++;  /* skip msgid */
                pub_ok = (p && atoi(p) == 0);
                parse_qmtrecv(buf);
                break;
            }
            if (strstr(buf, "+QMTRECV:")) {
                parse_qmtrecv(buf);
                /* continue waiting for QMTPUBEX */
            }
            check_qmtstat(buf);          /* link dropped mid-publish? */
            if (!g_connected) break;
        }
    }

    /* Detect a silently-dead link: the modem can keep accepting QMTPUBEX
     * (returning OK/'>') while the broker TCP session is actually gone, so
     * mqtt_is_connected() alone never trips. Count consecutive missing
     * PUBEX acks; past the threshold, mark the link down so the next
     * mqtt_reconnect_if_needed() rebuilds it (L1→L4). Accounting lives in
     * pub_result() so the two early-out failure paths above feed the same
     * streak. */
    return pub_result(pub_ok, topic);
}

/* Pop the oldest queued message into the caller's buffers. O(1): advances the
 * ring-buffer head rather than shifting the array. Returns false if empty. */
static bool stash_pop(char *topic_out, size_t topic_size,
                      char *payload_out, size_t payload_size) {
    if (g_stash_cnt == 0) return false;
    const MqttMsg_t *m = &g_stash[g_stash_head];
    strncpy(topic_out,   m->topic,   topic_size - 1);
    topic_out[topic_size - 1] = '\0';
    strncpy(payload_out, m->payload, payload_size - 1);
    payload_out[payload_size - 1] = '\0';
    g_stash_head = (g_stash_head + 1) % STASH_MAX;
    g_stash_cnt--;
    return true;
}

bool mqtt_poll(char *topic_out, size_t topic_size,
               char *payload_out, size_t payload_size) {
    if (stash_pop(topic_out, topic_size, payload_out, payload_size)) return true;
    drain_recv(100);
    return stash_pop(topic_out, topic_size, payload_out, payload_size);
}

void mqtt_disconnect(void) {
    modem_at("AT+QMTDISC=0", "OK", 5000, NULL, 0);
    modem_at("AT+QMTCLOSE=0","OK", 5000, NULL, 0);
    g_connected = false;
}

bool mqtt_is_connected(void) { return g_connected; }
