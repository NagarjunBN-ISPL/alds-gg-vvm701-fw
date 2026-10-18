#include "rsp_tunnel.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "tunnel";

_Static_assert(RSP_TUNNEL_REQ_MAX <= 255, "a queued frame's length is a uint8_t");

/* ── Request text ─────────────────────────────────────────────────────────── */

/* Value of one hex digit, or -1. */
static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool is_sep(char c) {
    return c == ',' || c == ' ' || c == ':' || c == '-' ||
           c == '\t' || c == '\r' || c == '\n';
}

static bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

/* Whitespace off both ends of s[0 .. *len); returns the new start. */
static const char *trim(const char *s, size_t *len) {
    while (*len > 0 && is_space(s[0]))        { s++; (*len)--; }
    while (*len > 0 && is_space(s[*len - 1])) (*len)--;
    return s;
}

/* Hex text s[0 .. len) -> bytes. The token rules are cng-cpil's
 * du_hex_decode_frame(), so every tunnel in the fleet reads the same payloads. */
static size_t decode_span(const char *s, size_t len, uint8_t *out, size_t out_max) {
    s = trim(s, &len);
    /* ONE pair of quotes if the frame came as a JSON string. A quote anywhere
     * else is not hex, and refuses the payload below. */
    if (len >= 2 && s[0] == '"' && s[len - 1] == '"') {
        s++;
        len -= 2;
        s = trim(s, &len);
    }

    size_t n = 0, i = 0;
    while (i < len) {
        /* A run of separators is one gap, and an empty token between two
         * commas is skipped — the pilot skipped it too. */
        if (is_sep(s[i])) { i++; continue; }

        size_t end = i;
        while (end < len && !is_sep(s[end])) end++;

        /* strtol's optional prefix. 'x' is not a hex digit, so this can never
         * eat the start of a plain run like "0A0B". */
        size_t d = i;
        if (end - d >= 2 && s[d] == '0' && (s[d + 1] == 'x' || s[d + 1] == 'X')) d += 2;

        size_t digits = end - d;
        if (digits == 0) return 0;                  /* a bare "0x"               */
        for (size_t k = d; k < end; k++)
            if (hex_val(s[k]) < 0) return 0;        /* not hex at all            */

        if (digits == 1) {
            /* "2" is 0x02, exactly as strtol read the pilot's tokens. */
            if (n >= out_max) return 0;
            out[n++] = (uint8_t)hex_val(s[d]);
        } else if (digits % 2 != 0) {
            /* "021": which digit stands alone cannot be known, and guessing
             * puts an invented byte on the bus. */
            return 0;
        } else {
            for (size_t k = d; k < end; k += 2) {
                if (n >= out_max) return 0;         /* too long: refused whole   */
                out[n++] = (uint8_t)((hex_val(s[k]) << 4) | hex_val(s[k + 1]));
            }
        }
        i = end;
    }
    return n;
}

/* The pilot's Yenen wrapper, {"Status":"lock","NZ1":"02,30,..."}: the first
 * key "NZ<digits>" (either case) whose value is a string. Returns the start of
 * that string and sets its length, or NULL. Every other key is skipped. */
static const char *nz_value(const char *s, size_t *vlen) {
    for (const char *q = strchr(s, '"'); q != NULL; q = strchr(q + 1, '"')) {
        const char *k = q + 1;
        if ((k[0] != 'N' && k[0] != 'n') || (k[1] != 'Z' && k[1] != 'z')) continue;
        k += 2;
        if (*k < '0' || *k > '9') continue;
        while (*k >= '0' && *k <= '9') k++;
        if (*k != '"') continue;
        k++;
        while (is_space(*k)) k++;
        if (*k != ':') continue;
        k++;
        while (is_space(*k)) k++;
        if (*k != '"') continue;                    /* the frame must be a string */
        k++;
        const char *end = strchr(k, '"');
        if (end == NULL) return NULL;               /* unterminated              */
        *vlen = (size_t)(end - k);
        return k;
    }
    return NULL;
}

size_t rsp_tunnel_decode(const char *payload, uint8_t *out, size_t out_max) {
    if (payload == NULL || out == NULL || out_max == 0) return 0;
    size_t      len = strlen(payload);
    const char *s   = trim(payload, &len);
    if (len > 0 && s[0] == '{') {
        /* An object is read only as the pilot's wrapper, and only whole: one
         * that does not close is a cut payload, not a command. */
        if (s[len - 1] != '}') return 0;
        size_t      vlen = 0;
        const char *v    = nz_value(s, &vlen);
        return v ? decode_span(v, vlen, out, out_max) : 0;
    }
    return decode_span(s, len, out, out_max);
}

/* ── The exchange ─────────────────────────────────────────────────────────── */

/* Bytes -> lowercase comma hex, the pilot's "%02x" form. The caller has made
 * sure it fits: 3 characters per byte, NUL included. */
static void to_hex(const uint8_t *b, size_t n, char *out) {
    static const char digits[] = "0123456789abcdef";
    size_t w = 0;
    for (size_t i = 0; i < n; i++) {
        if (i) out[w++] = ',';
        out[w++] = digits[b[i] >> 4];
        out[w++] = digits[b[i] & 0x0F];
    }
    out[w] = '\0';
}

/* Static, not stack: the main task runs this under its deepest call chain
 * (poll_incoming_mqtt -> execute_pending), and only one exchange at a time. */
static uint8_t s_resp[RSP_TUNNEL_RESP_MAX];
static char    s_tx_text[RSP_TUNNEL_REQ_MAX * 3];

bool rsp_tunnel_exchange(const uint8_t *req, size_t len,
                         char *reply, size_t reply_size) {
    if (reply == NULL || reply_size < sizeof("404")) return false;
    snprintf(reply, reply_size, "404");
    if (req == NULL || len == 0 || len > RSP_TUNNEL_REQ_MAX) return false;

    to_hex(req, len, s_tx_text);
    ESP_LOGI(TAG, "TX (%u bytes): [%s]", (unsigned)len, s_tx_text);

    /* Anything already in the RX buffer is left over from an earlier exchange,
     * not this pump's reply. Bounded, so a line spewing noise cannot hold the
     * task here. */
    uint8_t junk[64];
    for (int i = 0; i < 16 && uart_read_bytes(RS485_UART, junk, sizeof(junk), 0) > 0; i++) {}

    uart_write_bytes(RS485_UART, req, len);

    /* The pilot's read: the window restarts on every byte, so the reply is
     * whatever arrives until the line goes quiet. No length byte, ETX or
     * checksum ends it early — the tunnel does not know the frame's layout. */
    const int64_t window_us = (int64_t)RS485_RECV_TIMEOUT_MS * 1000;
    const int64_t start_us  = esp_timer_get_time();
    int64_t       last_us   = start_us;   /* the first-byte wait starts at the send */
    size_t        got       = 0;          /* every byte seen, kept or not           */
    bool          quiet     = false;
    for (;;) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_us >= window_us)      { quiet = true; break; }
        if (now_us - start_us >= 3 * window_us) break;      /* never went quiet */
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            if (got < sizeof(s_resp)) s_resp[got] = ch;
            got++;
            last_us = esp_timer_get_time();
        }
    }

    if (got == 0) {
        ESP_LOGW(TAG, "RX: no reply within %d ms — 404", RS485_RECV_TIMEOUT_MS);
        return false;
    }
    if (!quiet) {
        ESP_LOGW(TAG, "RX: %u bytes and the line still talking after %d ms — "
                      "not a reply, 404", (unsigned)got, 3 * RS485_RECV_TIMEOUT_MS);
        return false;
    }
    if (got > sizeof(s_resp)) {
        ESP_LOGW(TAG, "RX: %u bytes, more than the %d a reply can be — 404",
                 (unsigned)got, RSP_TUNNEL_RESP_MAX);
        return false;
    }
    if (got * 3 > reply_size) {
        ESP_LOGW(TAG, "RX: %u bytes do not fit the %u-byte reply buffer — 404",
                 (unsigned)got, (unsigned)reply_size);
        return false;
    }
    to_hex(s_resp, got, reply);
    ESP_LOGI(TAG, "RX (%u bytes): [%s]", (unsigned)got, reply);
    return true;
}

/* ── Queue ────────────────────────────────────────────────────────────────── */
static RspTunnelCmd_t s_queue[RSP_TUNNEL_QUEUE_MAX];
static int            s_q_head  = 0;     /* index of the oldest command */
static int            s_q_count = 0;

bool rsp_tunnel_push(uint8_t unit, uint8_t kind, const uint8_t *frame, size_t len) {
    if (frame == NULL || len == 0 || len > RSP_TUNNEL_REQ_MAX) return false;
    if (s_q_count >= RSP_TUNNEL_QUEUE_MAX) return false;
    RspTunnelCmd_t *c = &s_queue[(s_q_head + s_q_count) % RSP_TUNNEL_QUEUE_MAX];
    c->unit = unit;
    c->kind = kind;
    c->len  = (uint8_t)len;
    memcpy(c->frame, frame, len);
    s_q_count++;
    return true;
}

bool rsp_tunnel_pop(RspTunnelCmd_t *out) {
    if (out == NULL || s_q_count == 0) return false;
    *out = s_queue[s_q_head];
    s_q_head = (s_q_head + 1) % RSP_TUNNEL_QUEUE_MAX;
    s_q_count--;
    return true;
}
