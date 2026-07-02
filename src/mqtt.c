#include "mqtt.h"
#include "modem.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "mqtt";

#define STASH_MAX    8
#define TOPIC_MAX    256
#define PAYLOAD_MAX  2048

typedef struct {
    char topic[TOPIC_MAX];
    char payload[PAYLOAD_MAX];
} MqttMsg_t;

static MqttMsg_t g_stash[STASH_MAX];
static int       g_stash_cnt  = 0;
static int       g_msgid      = 1;
static bool      g_connected  = false;
static char      g_client_id[80];

static int next_msgid(void) {
    int id = g_msgid++;
    if (g_msgid > 65535) g_msgid = 1;
    return id;
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
    while (*p >= '0' && *p <= '9') p++;           /* skip msgid */
    if (*p != ',') return NULL;
    p++;
    if (*p != '"') return NULL;
    p++;
    const char *topic_start = p;
    while (*p && *p != '"') p++;
    if (*p != '"') return NULL;                   /* topic not fully arrived yet */
    size_t topic_len = (size_t)(p - topic_start);
    p++;
    if (*p != ',') return NULL;
    p++;
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

/* ── Parse +QMTRECV URCs from a buffer, push into stash ──────────────── */
static void parse_qmtrecv(const char *buf) {
    const char *search_from = buf;
    for (;;) {
        if (g_stash_cnt >= STASH_MAX) break;
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

        MqttMsg_t *m = &g_stash[g_stash_cnt++];
        size_t tl = (topic_len < TOPIC_MAX - 1)   ? topic_len : TOPIC_MAX - 1;
        size_t pl = (pl_len   < PAYLOAD_MAX - 1)  ? pl_len   : PAYLOAD_MAX - 1;
        memcpy(m->topic,   topic_start,   tl); m->topic[tl]   = '\0';
        memcpy(m->payload, content_start, pl); m->payload[pl] = '\0';

        search_from = content_stop;
    }
}

static void drain_recv(int timeout_ms) {
    static char buf[PAYLOAD_MAX + 300];
    int total = 0;
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
}

/* ── Public API ───────────────────────────────────────────────────────── */

bool mqtt_init(const char *imei) {
    snprintf(g_client_id, sizeof(g_client_id), "iotronics-%s", imei);
    g_stash_cnt = 0;
    g_connected = false;
    g_msgid     = 1;
    return true;
}

bool mqtt_connect(void) {
    static char cmd[512], resp[512];

    modem_at("AT+QMTCLOSE=0", "OK", 5000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* TLS + persistent session — both must be set before QMTOPEN, since
     * Quectel rejects QMTCFG/QSSLCFG changes once the link is already open.
     * seclevel=0: no cert verification (no CA cert provisioned on-device yet).
     * Encrypts the transport so credentials aren't sent in the clear. */
    modem_at("AT+QSSLCFG=\"sslversion\",2,4",       "OK", 3000, NULL, 0);
    modem_at("AT+QSSLCFG=\"ciphersuite\",2,0XFFFF", "OK", 3000, NULL, 0);
    modem_at("AT+QSSLCFG=\"seclevel\",2,0",         "OK", 3000, NULL, 0);
    modem_at("AT+QMTCFG=\"ssl\",0,1,2",             "OK", 3000, NULL, 0);
    modem_at("AT+QMTCFG=\"keepalive\",0,60",        "OK", 3000, NULL, 0);

    /* Persistent session: broker queues QoS-1 messages while device is offline.
     * Quectel's AT command set calls this "session", not "clean". */
    modem_at("AT+QMTCFG=\"session\",0,0", "OK", 5000, NULL, 0);

    /* Open TCP/TLS connection to broker */
    snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d", MQTT_HOST, MQTT_PORT);
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
             g_client_id, MQTT_USER, MQTT_PASS);
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

bool mqtt_publish(const char *topic, const char *payload) {
    if (!g_connected) return false;

    int  mid = next_msgid();
    int  len = strlen(payload);
    static char cmd[512];
    snprintf(cmd, sizeof(cmd), "AT+QMTPUBEX=0,%d,1,0,\"%s\",%d", mid, topic, len);

    /* Send command, wait for '>' prompt, rescue any +QMTRECV URCs */
    modem_flush();
    static char full[600];
    int n = snprintf(full, sizeof(full), "%s\r\n", cmd);
    uart_write_bytes(MODEM_UART, full, n);

    static char buf[PAYLOAD_MAX + 300];
    memset(buf, 0, sizeof(buf));
    int  bi = 0;
    bool got_prompt = false;
    int64_t dl = esp_timer_get_time() + 10000000LL;

    while (esp_timer_get_time() < dl) {
        uint8_t ch;
        if (uart_read_bytes(MODEM_UART, &ch, 1, pdMS_TO_TICKS(20)) > 0) {
            if (bi < (int)sizeof(buf) - 1) { buf[bi++] = (char)ch; buf[bi] = '\0'; }
            if (ch == '>') { got_prompt = true; break; }
            if (strstr(buf, "ERROR")) { parse_qmtrecv(buf); return false; }
        }
    }
    parse_qmtrecv(buf);  /* rescue any URCs that arrived before > */

    if (!got_prompt) {
        ESP_LOGW(TAG, "No > prompt for %s", topic);
        return false;
    }

    /* Send payload bytes */
    uart_write_bytes(MODEM_UART, payload, len);

    /* Wait for +QMTPUBEX: URC, rescuing +QMTRECV meanwhile */
    memset(buf, 0, sizeof(buf));
    bi = 0;
    bool pub_ok = false;
    dl = esp_timer_get_time() + 15000000LL;
    while (esp_timer_get_time() < dl) {
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
        }
    }

    if (!pub_ok) ESP_LOGW(TAG, "No PUBEX ack for %s", topic);
    return pub_ok;
}

bool mqtt_poll(char *topic_out, size_t topic_size,
               char *payload_out, size_t payload_size) {
    if (g_stash_cnt > 0) {
        strncpy(topic_out,   g_stash[0].topic,   topic_size - 1);
        topic_out[topic_size - 1] = '\0';
        strncpy(payload_out, g_stash[0].payload, payload_size - 1);
        payload_out[payload_size - 1] = '\0';
        for (int i = 1; i < g_stash_cnt; i++) g_stash[i - 1] = g_stash[i];
        g_stash_cnt--;
        return true;
    }
    drain_recv(100);
    if (g_stash_cnt > 0) {
        strncpy(topic_out,   g_stash[0].topic,   topic_size - 1);
        topic_out[topic_size - 1] = '\0';
        strncpy(payload_out, g_stash[0].payload, payload_size - 1);
        payload_out[payload_size - 1] = '\0';
        for (int i = 1; i < g_stash_cnt; i++) g_stash[i - 1] = g_stash[i];
        g_stash_cnt--;
        return true;
    }
    return false;
}

void mqtt_disconnect(void) {
    modem_at("AT+QMTDISC=0", "OK", 5000, NULL, 0);
    modem_at("AT+QMTCLOSE=0","OK", 5000, NULL, 0);
    g_connected = false;
}

bool mqtt_is_connected(void) { return g_connected; }
