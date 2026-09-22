#include "yenen.h"
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

static const char *TAG = "yn";

#define YN_STX 0x02
#define YN_ETX 0x03
/* Command byte at frame offset 6. 'A' reads (Command 2), 'K' writes the RSP —
 * the two frames are otherwise identical up to that byte. See yenen.h. */
#define YN_CMD_SET_RSP 0x4B
/* RSP write reply. Not an STX frame and not address-echoed, so the poll path's
 * validation does not apply to it. */
#define YN_ACK 0x06
#define YN_NAK 0x15
/* YN_SYN (0x16) removed with Command 1 — it was that frame's lead byte and has
 * no other use in the protocol. See the one-command note in yenen.h. */

/* ── Frame generation ─────────────────────────────────────────────────────
 * See yenen.h for the full protocol notes. Frames are generated per address
 * rather than kept as a hardcoded table (the shape dispenser.c settled on for
 * BlueSky), so head count can scale without pre-computed entries. */

/* Nozzle side code: A -> '0'+(head-1), B -> '5'+(head-1). */
static uint8_t yenen_nzcode(uint8_t head, uint8_t side) {
    return (uint8_t)((side ? '5' : '0') + (head - 1));
}

/* XOR every byte STX..ETX inclusive, then emit each nibble as (nibble + 0x30).
 * NOT ASCII hex — 0x7C encodes as '7','<' (37 3C), not '7','C' (37 43). */
static void yenen_checksum(const uint8_t *frame, int len, uint8_t *out /* 2 bytes */) {
    uint8_t x = 0;
    for (int i = 0; i < len; i++) x ^= frame[i];
    out[0] = (uint8_t)(((x >> 4) & 0x0F) + 0x30);
    out[1] = (uint8_t)((x & 0x0F) + 0x30);
}

/* Command 2 — totalizer / total amount / RSP. 10 bytes incl. 2 checksum.
 * The ONLY command this driver issues.
 *
 * Verified byte-exact against every frame in the vendor doc:
 *   DU1 NZ1  02 30 38 30 31 30 41 03 37 39
 *   DU1 NZ2  02 30 38 30 31 35 41 03 37 3C
 *   DU2 NZ3  02 30 38 30 32 31 41 03 37 3B
 *   DU2 NZ4  02 30 38 30 32 36 41 03 37 3C
 * The generated <head>/<nzcode>/checksum reproduce all four exactly, and the
 * same generator extends to DU3..DU5 (heads 3..5, nzcodes '2'/'7', '3'/'8',
 * '4'/'9') — which is what DU_YENEN_MAX = 5 rests on. */
static int yenen_frame_total(uint8_t head, uint8_t side, uint8_t *out /* 10 bytes */) {
    out[0] = YN_STX;
    out[1] = '0';
    out[2] = '8';
    out[3] = '0';
    out[4] = (uint8_t)('0' + head);
    out[5] = yenen_nzcode(head, side);
    out[6] = 'A';
    out[7] = YN_ETX;
    yenen_checksum(out, 8, &out[8]);
    return 10;
}

/* ── RS485 mode helper ────────────────────────────────────────────────────
 * Parity: NONE (8N1) — confirmed 2026-08-19, not inferred. BlueSky is the odd
 * one out on this bus at 8E1, which is why the poll loop re-asserts the mode
 * per nozzle rather than once per cycle.
 *
 * Baud: YENEN_BAUD (9600) is still the working assumption — the vendor doc
 * lists frames but no line rate. If frames come back garbled (as opposed to
 * not at all) with parity now known-good, baud is the remaining variable. */
void yn_rs485_set_yenen_mode(void) {
    uart_config_t cfg = {
        .baud_rate = YENEN_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(RS485_UART, &cfg);
}

/* ── Low-level RS485 transaction ─────────────────────────────────────────
 * Mirrors rs485_xfer() in dispenser.c, including the WDT feed and the hard
 * deadline that keeps a noise storm from stretching one read toward a minute.
 *
 * The early-exit rule differs because the framing does: Yenen replies are
 * ETX-terminated, so once ETX arrives exactly 2 checksum bytes follow and the
 * frame is complete. That turns a read into tens of ms instead of waiting out
 * the full RS485_RECV_TIMEOUT_MS of silence — the same win the self-describing
 * length byte gives the BlueSky path. */
static int yenen_xfer(const uint8_t *cmd, size_t cmd_len,
                      uint8_t *resp, size_t resp_max) {
    uint8_t tmp[64];
    while (uart_read_bytes(RS485_UART, tmp, sizeof(tmp), 0) > 0);

    uart_write_bytes(RS485_UART, cmd, cmd_len);

    int got       = 0;
    int etx_at    = -1;   /* index of ETX once seen; -1 = not yet */
    uint32_t t0      = (uint32_t)(esp_timer_get_time() / 1000);
    int64_t  hard_dl = esp_timer_get_time()
                       + (int64_t)RS485_RECV_TIMEOUT_MS * 3 * 1000;
    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < RS485_RECV_TIMEOUT_MS
           && esp_timer_get_time() < hard_dl
           && got < (int)resp_max) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            resp[got++] = ch;
            t0 = (uint32_t)(esp_timer_get_time() / 1000); /* reset on each byte */

            /* Only trust an ETX that follows a leading STX — a stray 0x03 in
             * the middle of line noise would otherwise truncate a good frame
             * two bytes later. */
            if (etx_at < 0 && ch == YN_ETX && got > 1 && resp[0] == YN_STX)
                etx_at = got - 1;
            if (etx_at >= 0 && got >= etx_at + 3) break;   /* ETX + 2 checksum */
        }
    }
    return got;
}

/* ── Response validation ──────────────────────────────────────────────────
 * A reply must (a) be STX-framed, (b) carry ETX with exactly 2 trailing
 * checksum bytes, and (c) match the checksum we recompute over STX..ETX.
 * Anything failing is treated exactly like no response — the cloud sees
 * "404" rather than a corrupted reading forwarded as real data, which is the
 * same rule radar.c applies with its CRC check. */
static bool yenen_resp_valid(const uint8_t *buf, int len, const char *what,
                             uint8_t head, uint8_t side) {
    if (len < 4) {
        ESP_LOGW(TAG, "head%u/%c %s: short frame (%d bytes) — dropped",
                 head, side ? 'B' : 'A', what, len);
        return false;
    }
    if (buf[0] != YN_STX) {
        ESP_LOGW(TAG, "head%u/%c %s: no STX (0x%02x) — dropped",
                 head, side ? 'B' : 'A', what, buf[0]);
        return false;
    }
    int etx = -1;
    for (int i = 1; i < len; i++) if (buf[i] == YN_ETX) { etx = i; break; }
    if (etx < 0 || etx + 2 >= len) {
        ESP_LOGW(TAG, "head%u/%c %s: no ETX+checksum — dropped",
                 head, side ? 'B' : 'A', what);
        return false;
    }
    uint8_t ck[2];
    yenen_checksum(buf, etx + 1, ck);
    if (buf[etx + 1] != ck[0] || buf[etx + 2] != ck[1]) {
        ESP_LOGW(TAG, "head%u/%c %s: checksum mismatch "
                      "(calc %02x %02x, recv %02x %02x) — dropped",
                 head, side ? 'B' : 'A', what, ck[0], ck[1],
                 buf[etx + 1], buf[etx + 2]);
        return false;
    }

    /* ── Address echo MUST match the nozzle we asked for ──────────────────
     * Bytes [2:5] of the payload echo the address: '0', '0'+head, nzcode.
     * Checked because a pump was observed (2026-08-21, live bench) answering
     * the side-A query with a side-B addressed frame — it ignores the nozzle
     * code and always replies for the nozzle it actually has. STX, ETX and
     * checksum are all perfectly valid in that reply, so every earlier check
     * passes and the frame was accepted as side A's data.
     *
     * The consequence is worse than a bad reading: BOTH nozzles then publish
     * the same totalizer under different labels, and since the cloud derives
     * every sale by differencing consecutive totalizer readings per nozzle, a
     * real sale on the physical nozzle materialises a SECOND phantom sale on
     * the one that does not exist — silently doubling the site's recorded
     * volume. A single-sided DU makes this the normal case, not an edge case.
     *
     * Rejecting here makes the absent side publish "404" (accurate — there is
     * no nozzle at that address) instead of a duplicate, and after
     * DU_ABSENT_AFTER_ROUNDS it also stops spending a full retry budget on it
     * every round. Frame layout is fixed, so buf[4]/buf[5] are always the
     * address digits when the frame is long enough to have passed the checks
     * above. */
    /* A frame whose ETX lands before the address digits (buf[4]/buf[5]) is far
     * too short to be a real Command-2 reply and carries no address to verify.
     * A stray 0x03 early in line noise, with a coincidentally-valid 1-byte XOR
     * checksum, would otherwise skip the echo check entirely and be accepted as
     * a totalizer reading — the cloud then differences that garbage into a
     * phantom sale. Reject it outright; a genuine reply is ~51 bytes, so no
     * legitimate frame has etx < 6. */
    if (etx < 6) {
        ESP_LOGW(TAG, "head%u/%c %s: frame too short to carry an address "
                      "(etx=%d) — dropped", head, side ? 'B' : 'A', what, etx);
        return false;
    }
    {
        uint8_t want_head = (uint8_t)('0' + head);
        uint8_t want_nz   = yenen_nzcode(head, side);
        if (buf[4] != want_head || buf[5] != want_nz) {
            ESP_LOGW(TAG, "head%u/%c %s: reply is for a DIFFERENT nozzle "
                          "(echo '%c%c', expected '%c%c') — dropped, publishing 404 "
                          "rather than duplicating another nozzle's totalizer",
                     head, side ? 'B' : 'A', what,
                     buf[4], buf[5], want_head, want_nz);
            return false;
        }
    }
    return true;
}

/* Unconditional raw-bytes log — same field-debugging aid radar.c provides, so
 * a station visit can see real wire data without a logic analyzer. */
static void yn_log_raw(const char *what, uint8_t head, uint8_t side,
                       const uint8_t *buf, int len) {
    if (len <= 0) {
        ESP_LOGI(TAG, "head%u/%c %s RAW: no response", head, side ? 'B' : 'A', what);
        return;
    }
    char hex[YN_RESP_MAX * 3 + 1];
    int pos = 0;
    for (int i = 0; i < len && pos + 4 < (int)sizeof(hex); i++) {
        if (i > 0) hex[pos++] = ',';
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x", buf[i]);
    }
    hex[pos] = '\0';
    ESP_LOGI(TAG, "head%u/%c %s RAW (%d bytes): [%s]",
             head, side ? 'B' : 'A', what, len, hex);
}

/* Issue one command with retries. Returns bytes stored, 0 on total failure. */
static int yenen_read_field(const uint8_t *cmd, int cmd_len,
                            uint8_t *out, const char *what,
                            uint8_t head, uint8_t side, int retries) {
    /* Log the command bytes, not just the reply. Without this a silent pump is
     * indistinguishable from a malformed frame: "no response" could mean the
     * pump ignored a perfectly good command OR that we put the wrong bytes on
     * the wire, and those need completely different fixes. One line per read,
     * and it makes a field log self-documenting. */
    {
        char txhex[3 * 12 + 1];
        int pos = 0;
        for (int i = 0; i < cmd_len && pos + 4 < (int)sizeof(txhex); i++) {
            if (i > 0) txhex[pos++] = ',';
            pos += snprintf(txhex + pos, sizeof(txhex) - pos, "%02x", cmd[i]);
        }
        txhex[pos] = '\0';
        ESP_LOGI(TAG, "head%u/%c %s TX (%d bytes): [%s]",
                 head, side ? 'B' : 'A', what, cmd_len, txhex);
    }

    for (int attempt = 1; attempt <= retries; attempt++) {
        int len = yenen_xfer(cmd, cmd_len, out, YN_RESP_MAX);
        yn_log_raw(what, head, side, out, len);
        if (len > 0 && yenen_resp_valid(out, len, what, head, side))
            return len;
        if (attempt < retries) {
            ESP_LOGW(TAG, "head%u/%c %s: no valid frame (attempt %d/%d) — retrying",
                     head, side ? 'B' : 'A', what, attempt, retries);
            vTaskDelay(pdMS_TO_TICKS(YN_RETRY_MS));
        }
    }
    return 0;
}

/* ── Polling ──────────────────────────────────────────────────────────────
 * ONE command per nozzle — Command 2 — see the rationale in yenen.h.
 *
 * Unlike BlueSky there is no status/idle gate before the read, and that is
 * deliberate rather than an omission. BlueSky has to gate because its protocol
 * exposes the busy state in a SEPARATE status command — the only way to know a
 * reading is mid-sale is to ask first, so the firmware asks and skips.
 *
 * Yenen carries the flag INSIDE the reply itself: bytes [41:43] of the decoded
 * Command 2 payload are non-zero while dispensing and 00 when idle. The cloud
 * therefore receives everything it needs to reject a partial reading, and it is
 * documented to do exactly that (CLOUD_INTEGRATION_GUIDE_SR.md, Appendix B.4
 * rule 1). Adding a firmware-side gate would mean discarding frames the backend
 * can already classify, and would cost an extra transaction per nozzle to learn
 * something the reply was going to tell us anyway. */
void yn_poll_nozzle(uint8_t head, uint8_t side, YenenNozzle_t *nz) {
    memset(nz, 0, sizeof(*nz));
    if (head < 1 || head > DU_YENEN_MAX || side > 1) {
        ESP_LOGW(TAG, "poll: bad address head=%u side=%u", head, side);
        return;
    }

    uint8_t cmd[10];
    int n = yenen_frame_total(head, side, cmd);
    nz->total_len = yenen_read_field(cmd, n, nz->total, "total", head, side,
                                     YN_TOTAL_RETRIES);

    nz->present = (nz->total_len > 0);
    if (!nz->present)
        ESP_LOGW(TAG, "head%u/%c: no response", head, side ? 'B' : 'A');
}

/* ── Hex + payload ────────────────────────────────────────────────────── */
static void yn_bytes_to_hex(const uint8_t *data, int len, char *out, size_t out_size) {
    int pos = 0;
    for (int i = 0; i < len && pos + 4 < (int)out_size; i++) {
        if (i > 0) out[pos++] = ',';
        pos += snprintf(out + pos, out_size - pos, "%02x", data[i]);
    }
    out[pos] = '\0';
}

/* Same clamp guard as dispenser.c / genset.c — snprintf returns <0 on encoding
 * error, and letting pos go negative would address outside the buffer. */
static inline int clamp_pos(int pos, size_t buf_size) {
    if (pos < 0) return 0;
    return (pos > (int)buf_size - 1) ? (int)buf_size - 1 : pos;
}

/* One nozzle per message, in exactly the shape the cloud already consumes for
 * the BlueSky fleet:
 *   {"HW_ID":"..","nozzles":{"1":{"data":".."}},"TS":".."}
 *
 * "data" is the Command 2 reply (totalizer / total amount / RSP) — the same key
 * BlueSky uses for its totalizer, because it is the same sale-critical value
 * the cloud differences.
 *
 * There is deliberately NO second key. An earlier revision published a "txn"
 * field alongside it carrying Command 1's reply; both that command and that
 * field are gone (see yenen.h), so a Yenen message is now indistinguishable in
 * structure from a BlueSky one and needs no Yenen-specific parser branch.
 *
 * NOTE for cloud integration: the legacy Yenen gateway published all four
 * nozzle blobs in ONE message under a single "NZ" key joined by "||" (see
 * Data_yenen-DU.pdf). This firmware deliberately does NOT reproduce that shape
 * — it follows this codebase's per-nozzle convention instead, so Yenen units
 * flow through the same parser, HW_ID scheme and diagnostics as every other
 * dispenser. Confirm with the cloud team before first deployment. */
void yn_build_nozzle_payload(const YenenNozzle_t *nz, int label,
                             const char *hw_id, const char *ts,
                             char *buf, size_t buf_size) {
    int pos = 0;

    pos += snprintf(buf + pos, buf_size - pos,
                    "{\"HW_ID\":\"%s\",\"nozzles\":{", hw_id);
    pos = clamp_pos(pos, buf_size);

    if (!nz->present) {
        pos += snprintf(buf + pos, buf_size - pos,
                        "\"%d\":{\"data\":\"404\"}", label);
        pos = clamp_pos(pos, buf_size);
    } else {
        char h_tot[YN_RESP_MAX * 3 + 1];
        yn_bytes_to_hex(nz->total, nz->total_len, h_tot, sizeof(h_tot));
        pos += snprintf(buf + pos, buf_size - pos,
                        "\"%d\":{\"data\":\"%s\"}", label, h_tot);
        pos = clamp_pos(pos, buf_size);
    }

    pos += snprintf(buf + pos, buf_size - pos, "},\"TS\":\"%s\"}", ts);
    pos = clamp_pos(pos, buf_size);

    /* clamp_pos prevents memory corruption but not broken JSON — a truncated
     * payload would be silently dropped by the cloud parser, so fall back to a
     * guaranteed-valid 404 shape (same rule as du_build_nozzle_payload). */
    if (pos >= (int)buf_size - 2) {
        ESP_LOGW(TAG, "nz%d payload truncated at %d/%zu bytes — publishing 404 instead",
                 label, pos, buf_size);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"nozzles\":{\"%d\":{\"data\":\"404\"}},\"TS\":\"%s\"}",
                 hw_id, label, ts);
    }
}

/* ── Remote command execution ─────────────────────────────────────────── */
static bool is_hex_ch(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

static int hex_str_to_bytes(const char *s, uint8_t *out, size_t out_max) {
    int n = 0;
    while (*s && n < (int)out_max) {
        while (*s && !is_hex_ch(*s)) s++;   /* skip separators / stray CRLF */
        if (!*s) break;
        if (!is_hex_ch(s[1])) break;        /* need a full 2-char byte */
        const char tmp[3] = {s[0], s[1], '\0'};
        out[n++] = (uint8_t)strtol(tmp, NULL, 16);
        s += 2;
    }
    return n;
}

bool yn_exec_passthrough(const char *cmd_hex, char *resp_buf, size_t resp_size) {
    uint8_t cmd[32];
    int cmd_len = hex_str_to_bytes(cmd_hex, cmd, sizeof(cmd));
    if (cmd_len == 0) return false;

    uint8_t resp[YN_RESP_MAX];
    int rlen = yenen_xfer(cmd, cmd_len, resp, YN_RESP_MAX);
    if (rlen == 0) return false;
    yn_bytes_to_hex(resp, rlen, resp_buf, resp_size);
    return true;
}

/* ── RSP (unit price) update ──────────────────────────────────────────────
 * Frame, sequence and rationale are in yenen.h. Everything below is confined
 * to this feature: no function above this line is modified or called
 * differently, so a gateway that never receives a price push behaves exactly
 * as it did before. */

/* Payload offsets, as byte indexes into the RAW frame. Appendix B of the cloud
 * guide numbers the fields from the payload (STX stripped), so each is the
 * documented offset + 1. Kept as two named constants rather than open-coded
 * arithmetic because getting either wrong reads a plausible-but-wrong field:
 * price is [6:10] and the dispensing status is [41:43] of the payload. */
#define YN_OFF_PRICE   (1 + 6)
#define YN_OFF_STATUS  (1 + 41)

/* Command K, 14 bytes. Same header as Command 2 — only the command byte and
 * the four price digits differ. */
static int yenen_frame_setrsp(uint8_t head, uint8_t side, uint16_t price_x100,
                              uint8_t *out /* 14 bytes */) {
    out[0]  = YN_STX;
    out[1]  = '0';
    out[2]  = '8';
    out[3]  = '0';
    out[4]  = (uint8_t)('0' + head);
    out[5]  = yenen_nzcode(head, side);
    out[6]  = YN_CMD_SET_RSP;
    out[7]  = (uint8_t)('0' + (price_x100 / 1000) % 10);
    out[8]  = (uint8_t)('0' + (price_x100 / 100)  % 10);
    out[9]  = (uint8_t)('0' + (price_x100 / 10)   % 10);
    out[10] = (uint8_t)('0' + (price_x100)        % 10);
    out[11] = YN_ETX;
    yenen_checksum(out, 12, &out[12]);
    return 14;
}

/* Four ASCII digits -> integer. Rejects anything non-numeric rather than
 * letting atoi()-style leniency turn a garbled field into a plausible price. */
static bool yenen_digits4(const uint8_t *p, uint16_t *out) {
    uint16_t v = 0;
    for (int i = 0; i < 4; i++) {
        if (p[i] < '0' || p[i] > '9') return false;
        v = (uint16_t)(v * 10 + (p[i] - '0'));
    }
    *out = v;
    return true;
}

/* Transaction for the WRITE. Separate from yenen_xfer() because the reply has
 * no STX/ETX framing to key the early exit off: it completes on the first byte
 * if that byte is a NAK, and after three bytes if it is an ACK. Sharing
 * yenen_xfer() would mean waiting out the full silence timeout on every single
 * price update — bus time taken from the pumps for no information. */
static int yenen_xfer_ack(const uint8_t *cmd, size_t cmd_len,
                          uint8_t *resp, size_t resp_max) {
    uint8_t tmp[64];
    while (uart_read_bytes(RS485_UART, tmp, sizeof(tmp), 0) > 0);

    uart_write_bytes(RS485_UART, cmd, cmd_len);

    int      got     = 0;
    uint32_t t0      = (uint32_t)(esp_timer_get_time() / 1000);
    int64_t  hard_dl = esp_timer_get_time()
                       + (int64_t)RS485_RECV_TIMEOUT_MS * 3 * 1000;
    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < RS485_RECV_TIMEOUT_MS
           && esp_timer_get_time() < hard_dl
           && got < (int)resp_max) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            resp[got++] = ch;
            t0 = (uint32_t)(esp_timer_get_time() / 1000);
            if (resp[0] == YN_NAK) break;          /* rejected — one byte      */
            if (resp[0] == YN_ACK && got >= 3) break; /* 06 30 31              */
        }
    }
    return got;
}

/* Read one nozzle's current price and dispensing state via Command 2.
 * Returns false if the nozzle never produced a valid frame for THIS address —
 * yenen_read_field applies the full validation, address echo included. */
static bool yenen_read_price_status(uint8_t head, uint8_t side,
                                    uint16_t *price_x100, bool *idle) {
    uint8_t cmd[10];
    int     n = yenen_frame_total(head, side, cmd);
    uint8_t resp[YN_RESP_MAX];

    int len = yenen_read_field(cmd, n, resp, "rsp-read", head, side, YN_RSP_RETRIES);
    if (len <= 0) return false;

    if (len < YN_OFF_STATUS + 2) {
        ESP_LOGW(TAG, "head%u/%c rsp-read: frame too short (%d bytes) to carry "
                      "price and status — treating as no response",
                 head, side ? 'B' : 'A', len);
        return false;
    }
    if (!yenen_digits4(&resp[YN_OFF_PRICE], price_x100)) {
        ESP_LOGW(TAG, "head%u/%c rsp-read: price field is not 4 digits — "
                      "treating as no response", head, side ? 'B' : 'A');
        return false;
    }
    /* Idle is BOTH status characters '0'. Tested as "is it zero", not "is it
     * 11": the non-zero values are field-inferred, not vendor-documented. */
    *idle = (resp[YN_OFF_STATUS] == '0' && resp[YN_OFF_STATUS + 1] == '0');
    return true;
}

const char *yn_rsp_result_str(YnRspResult_t r) {
    switch (r) {
        case YN_RSP_OK:            return "RSP_updated";
        case YN_RSP_BAD_ARG:       return "bad_value";
        case YN_RSP_NO_RESPONSE:   return "no_response";
        case YN_RSP_BUSY:          return "nozzle_busy";
        case YN_RSP_NAK:           return "nak";
        case YN_RSP_NO_ACK:        return "no_ack";
        case YN_RSP_VERIFY_FAILED: return "verify_failed";
    }
    return "unknown";
}

YnRspResult_t yn_set_rsp(uint8_t head, uint8_t side, uint16_t price_x100,
                         uint16_t *readback_x100) {
    if (readback_x100) *readback_x100 = 0;

    if (head < 1 || head > DU_YENEN_MAX || side > 1 ||
        price_x100 == 0 || price_x100 > 9999) {
        ESP_LOGW(TAG, "set_rsp: bad args head=%u side=%u price=%u",
                 head, side, price_x100);
        return YN_RSP_BAD_ARG;
    }

    /* 1 — idle gate. A price written mid-delivery re-rates a sale that is
     * already running, so a busy nozzle is refused here rather than written
     * and explained afterwards. */
    uint16_t before = 0;
    bool     idle   = false;
    if (!yenen_read_price_status(head, side, &before, &idle)) {
        ESP_LOGW(TAG, "head%u/%c set_rsp: nozzle did not answer the pre-read — "
                      "price NOT written", head, side ? 'B' : 'A');
        return YN_RSP_NO_RESPONSE;
    }
    if (!idle) {
        ESP_LOGW(TAG, "head%u/%c set_rsp: nozzle is dispensing — price NOT "
                      "written (retried while the command's window lasts)",
                 head, side ? 'B' : 'A');
        return YN_RSP_BUSY;
    }
    ESP_LOGI(TAG, "head%u/%c set_rsp: %u.%02u -> %u.%02u",
             head, side ? 'B' : 'A',
             (unsigned)(before / 100), (unsigned)(before % 100),
             (unsigned)(price_x100 / 100), (unsigned)(price_x100 % 100));

    /* 2 — write, and require the documented ACK. */
    uint8_t cmd[14];
    int     n = yenen_frame_setrsp(head, side, price_x100, cmd);
    uint8_t ack[YN_RESP_MAX];
    int     alen  = 0;
    bool    acked = false;

    {
        char txhex[3 * 14 + 1];
        int  pos = 0;
        for (int i = 0; i < n && pos + 4 < (int)sizeof(txhex); i++) {
            if (i > 0) txhex[pos++] = ',';
            pos += snprintf(txhex + pos, sizeof(txhex) - pos, "%02x", cmd[i]);
        }
        txhex[pos] = '\0';
        ESP_LOGI(TAG, "head%u/%c set_rsp TX (%d bytes): [%s]",
                 head, side ? 'B' : 'A', n, txhex);
    }

    for (int attempt = 1; attempt <= YN_RSP_RETRIES && !acked; attempt++) {
        alen = yenen_xfer_ack(cmd, (size_t)n, ack, YN_RESP_MAX);
        yn_log_raw("set_rsp", head, side, ack, alen);
        if (alen >= 3 && ack[0] == YN_ACK && ack[1] == '0' && ack[2] == '1') {
            acked = true;
            break;
        }
        if (attempt < YN_RSP_RETRIES) vTaskDelay(pdMS_TO_TICKS(YN_RETRY_MS));
    }
    if (!acked) {
        if (alen == 0) {
            ESP_LOGW(TAG, "head%u/%c set_rsp: no answer to the write",
                     head, side ? 'B' : 'A');
            return YN_RSP_NO_ACK;
        }
        /* Only a literal 0x15 is a REJECTION — that one is worth reporting as
         * such, because it means the price is definitely unchanged and there is
         * nothing to re-read. Any other non-ACK answer (a truncated 06 with no
         * "01", line noise) leaves the write UNCONFIRMED: it may well have
         * landed. Calling that a NAK would tell the operator the opposite of
         * what to do about it, so it degrades to no_ack instead. */
        if (ack[0] == YN_NAK) {
            ESP_LOGW(TAG, "head%u/%c set_rsp: write rejected by the pump (0x15)",
                     head, side ? 'B' : 'A');
            return YN_RSP_NAK;
        }
        ESP_LOGW(TAG, "head%u/%c set_rsp: unrecognised answer to the write "
                      "(%d bytes, first 0x%02x) — price unconfirmed",
                 head, side ? 'B' : 'A', alen, ack[0]);
        return YN_RSP_NO_ACK;
    }

    /* 3 — read back. The ACK confirms the frame was received, not that the
     * price was stored, so the value reported to the cloud is this one. */
    uint16_t after = 0;
    bool     idle2 = false;
    if (!yenen_read_price_status(head, side, &after, &idle2)) {
        ESP_LOGW(TAG, "head%u/%c set_rsp: ACKed but the read-back failed — "
                      "price is unconfirmed", head, side ? 'B' : 'A');
        return YN_RSP_VERIFY_FAILED;
    }
    if (readback_x100) *readback_x100 = after;
    if (after != price_x100) {
        ESP_LOGE(TAG, "head%u/%c set_rsp: ACKed but pump holds %u.%02u, not "
                      "%u.%02u", head, side ? 'B' : 'A',
                 (unsigned)(after / 100), (unsigned)(after % 100),
                 (unsigned)(price_x100 / 100), (unsigned)(price_x100 % 100));
        return YN_RSP_VERIFY_FAILED;
    }

    ESP_LOGI(TAG, "head%u/%c set_rsp: confirmed at %u.%02u",
             head, side ? 'B' : 'A',
             (unsigned)(after / 100), (unsigned)(after % 100));
    return YN_RSP_OK;
}
