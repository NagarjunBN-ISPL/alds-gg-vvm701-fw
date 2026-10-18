#include "dispenser.h"
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

/* ── BlueSky command frame generation ─────────────────────────────────────
 * Reverse-engineered and verified byte-exact against the original hardcoded
 * NZ1-NZ4 frames; generated for any address so unit count can scale.
 *
 * BlueSky: F5 <addr> A2 <cmd> <checksum>, checksum = addr ^ cmd ^ 0xD7.
 * addr is a global per-nozzle RS485 address, 1-based.
 */
#define BS_CMD_STATUS 0xD5
#define BS_CMD_TOTAL  0xC5
#define BS_CMD_RSP    0xB6
#define BS_CMD_KFACT  0xE2
/* BS_CMD_RFID (0xD7) intentionally removed — RFID is no longer polled. */

static void bluesky_frame(uint8_t addr, uint8_t cmd_byte, uint8_t *out /* 5 bytes */) {
    out[0] = 0xF5;
    out[1] = addr;
    out[2] = 0xA2;
    out[3] = cmd_byte;
    out[4] = (uint8_t)(addr ^ cmd_byte ^ 0xD7);
}

/* ── RS485 mode helper ────────────────────────────────────────────────── */
void du_rs485_set_bluesky_mode(void) {
    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(RS485_UART, &cfg);
}

static const char *TAG = "du";

/* ── Low-level RS485 transaction ─────────────────────────────────────────
 * Normal path: return as soon as the frame's self-declared length has been
 * received (see the length note in the loop below) — typically tens of ms.
 *
 * Fallback path, used whenever the header isn't a recognisable BlueSky frame
 * (line noise, a non-conforming device, a corrupted length byte): the original
 * wait-for-silence rule below, unchanged. That keeps this strictly no worse
 * than the previous behaviour in every case it can't speed up.
 *
 * 2000 ms per-byte-reset timeout (timer resets each time a byte arrives),
 * PLUS an absolute cap: on a noisy line, sporadic garbage bytes keep
 * resetting the per-byte timer, and without a hard ceiling a single
 * nozzle read can stretch toward a minute — several units of that in one
 * cycle would starve the task watchdog. With the cap, a noise storm
 * degrades to "404" data instead of a WDT panic reboot. */
static int rs485_xfer(const uint8_t *cmd, size_t cmd_len,
                      uint8_t *resp, size_t resp_max) {
    /* Flush RX */
    uint8_t tmp[64];
    while (uart_read_bytes(RS485_UART, tmp, sizeof(tmp), 0) > 0);

    uart_write_bytes(RS485_UART, cmd, cmd_len);

    int got      = 0;
    int expected = 0;   /* frame length once known from the header; 0 = unknown */
    uint32_t t0      = (uint32_t)(esp_timer_get_time() / 1000);
    int64_t  hard_dl = esp_timer_get_time()
                       + (int64_t)RS485_RECV_TIMEOUT_MS * 3 * 1000;
    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < RS485_RECV_TIMEOUT_MS
           && esp_timer_get_time() < hard_dl
           && got < (int)resp_max) {
        /* Feed the WDT inside the loop, matching modem_at()'s pattern. The
         * hard cap above bounds this at RS485_RECV_TIMEOUT_MS*3 = 6 s, well
         * inside the 120 s WDT — but that safety is arithmetic coincidence,
         * not design: raising RS485_RECV_TIMEOUT_MS past ~40 s would silently
         * make this loop able to trip the watchdog. Feeding here makes the
         * loop safe by construction regardless of that constant. */
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        uint8_t ch;
        if (uart_read_bytes(RS485_UART, &ch, 1, pdMS_TO_TICKS(10)) > 0) {
            resp[got++] = ch;
            t0 = (uint32_t)(esp_timer_get_time() / 1000); /* reset on each byte */

            /* BlueSky frames are self-describing: byte[2] is 0xA<n> where n is
             * the number of bytes that follow the 3-byte header, so the total
             * length is (byte[2] & 0x0F) + 3. Verified against the protocol
             * doc's own examples and against every frame captured on the bench
             * (a3→6, a4→7, a6→9, a7→10, ac→15) — the outgoing commands obey it
             * too (a2→5).
             *
             * Without this the loop had no way to know a reply was complete
             * except by waiting out RS485_RECV_TIMEOUT_MS of silence, so a
             * 15-byte response that lands in ~17 ms still cost the full 2 s.
             * Bench measurement: 2.07 s per command x 20 commands = 41.4 s of a
             * 46.7 s cycle spent waiting on data that had already arrived. */
            if (got == 3 && resp[0] == 0xF5 && (resp[2] & 0xF0) == 0xA0) {
                int len = (resp[2] & 0x0F) + 3;
                if (len >= 3 && len <= (int)resp_max) expected = len;
            }
            if (expected && got >= expected) break;
        }
    }
    return got;
}

/* ── BlueSky response validation ──────────────────────────────────────────
 * Every reply to a read has one layout, confirmed on field frames of all five
 * commands (status D5, totalizer C5, RSP B6, k-factor E2, RFID D7) from two
 * sites:
 *
 *     F5 <addr> A<n> <data...> <cmd> <chk>     n = bytes after byte 2
 *     f5 02 a4 65 82 b6 02                     nozzle 2, RSP 65.82
 *
 * <cmd> echoes the command polled, and <chk> is the XOR of every byte before
 * it with bit 7 cleared — bluesky_frame()'s addr ^ cmd ^ 0xD7 is the same rule
 * worked out for our 5-byte commands. A reply to a read is accepted only if
 * ALL of these hold:
 *   - F5 and the polled address                  (crosstalk, line noise)
 *   - byte 2 is A<n> and the frame is n+3 bytes  (a reply cut short)
 *   - n >= 3, i.e. at least one data byte        (our own command echoed back)
 *   - <cmd> is the command we sent               (a late reply to another read)
 *   - <chk> matches                              (a byte corrupted on the wire)
 *   - totalizer and RSP data are BCD digits 0-9  (the cloud decodes them as BCD)
 * A frame that fails is dropped; the caller retries, then reports "404".
 *
 * A prior "XOR of all bytes == 0x80" check was removed on 2026-07-08 because
 * real replies XORed to 0x00. Both were half right: with bit 7 cleared, a
 * valid frame XORs to 0x00 or 0x80 depending on its data.
 *
 * What this cannot catch: a well-formed frame carrying a wrong value from the
 * pump itself. f5,02,a4,00,65,b6,00 (RSP 00.65 from a pump set to 65.82,
 * Vinayakapuram 2026-09-23) has a valid checksum and passes. */
static uint8_t bluesky_chk(const uint8_t *buf, int n) {
    uint8_t x = 0;
    for (int i = 0; i < n; i++) x ^= buf[i];
    return (uint8_t)(x & 0x7F);
}

static bool bcd_ok(const uint8_t *p, int n) {
    for (int i = 0; i < n; i++)
        if ((p[i] >> 4) > 9 || (p[i] & 0x0F) > 9) return false;
    return true;
}

static bool bluesky_read_valid(const uint8_t *buf, int len, uint8_t addr,
                               uint8_t cmd, const char *what) {
    const char *why = NULL;
    if (len < 6)
        why = "short frame";
    else if (buf[0] != 0xF5 || buf[1] != addr)
        why = "bad header/addr echo";
    else if ((buf[2] & 0xF0) != 0xA0 || (buf[2] & 0x0F) + 3 != len)
        why = "length byte does not match the frame";
    else if (buf[len - 2] != cmd)
        why = "not a reply to this command";
    else if (buf[len - 1] != bluesky_chk(buf, len - 1))
        why = "bad checksum";
    else if ((cmd == BS_CMD_TOTAL || cmd == BS_CMD_RSP) && !bcd_ok(buf + 3, len - 5))
        why = "non-BCD digit";
    if (why) {
        ESP_LOGW(TAG, "BlueSky nz%u %s: %s (%d bytes) — dropped", addr, what, why, len);
        return false;
    }
    return true;
}

/* Unconditional raw-bytes log for field debugging — prints exactly what came
 * back over RS485 for this command, regardless of whether it passed
 * validation, so a station visit can see real wire data without needing a
 * logic analyzer. */
static void log_raw_frame(uint8_t addr, const char *what, const uint8_t *buf, int len) {
    if (len <= 0) {
        ESP_LOGI(TAG, "BlueSky nz%u %s: no response", addr, what);
        return;
    }
    char hex[DU_RESP_MAX * 3 + 1];
    du_bytes_to_hex(buf, len, hex, sizeof(hex));
    ESP_LOGI(TAG, "BlueSky nz%u %s: [%s]", addr, what, hex);
}

/* Read one BlueSky data field (totalizer / RSP / k-factor) into buf, retrying
 * up to max_tries times, and return the VALIDATED response length (0 if every
 * attempt failed or returned an invalid frame). Consolidates the read + raw
 * log + frame validation the individual field blocks used to repeat
 * inline, and adds the retry the sale-critical reads now need. Each attempt
 * flushes the RS485 RX inside rs485_xfer(), so a stale reply from a prior
 * attempt can't be mistaken for this one. */
static int du_read_field(uint8_t addr, uint8_t cmd_byte, uint8_t *buf,
                         int max_tries, const char *what) {
    uint8_t cmd[5];
    bluesky_frame(addr, cmd_byte, cmd);
    for (int attempt = 1; attempt <= max_tries; attempt++) {
        if (attempt > 1) vTaskDelay(pdMS_TO_TICKS(DU_STATUS_RETRY_MS));
        int len = rs485_xfer(cmd, 5, buf, DU_RESP_MAX);
        esp_task_wdt_reset();
        log_raw_frame(addr, what, buf, len);
        if (len > 0 && bluesky_read_valid(buf, len, addr, cmd_byte, what))
            return len;
        if (attempt < max_tries)
            ESP_LOGW(TAG, "BlueSky nz%u %s: no valid frame (attempt %d/%d) — retrying",
                     addr, what, attempt, max_tries);
    }
    return 0;
}

/* ── K-factor cache ───────────────────────────────────────────────────────
 * The last k-factor known to be in each pump, indexed by the 1-based nozzle
 * address (1..DU_MAX_DISPENSERS*2; slot 0 unused). Filled by a due poll-round
 * read. RAM only: empty after a reboot, and the first round reads it.
 *
 * s_kf_stale: a command from the cloud's rsp/kfactor tunnel reached a BlueSky
 * pump (du_kfactor_mark_stale). The gateway does not read those frames, so any
 * of them may have been a k-factor write, and the pump may now hold a value
 * the cache does not — so every round reads the k-factor, due or not, until a
 * read succeeds. That way a recalibration shows in the next nozzle message
 * instead of up to KFACTOR_POLL_INTERVAL_S later.
 *
 * Both callers — du_task's poll round and the main task's tunnel — run under
 * the RS485 bus lock, which serialises every access to these arrays. */
#define KF_SLOTS (DU_MAX_DISPENSERS * 2 + 1)
static uint8_t s_kf_cache[KF_SLOTS][DU_RESP_MAX];
static int     s_kf_cache_len[KF_SLOTS];
static bool    s_kf_stale[KF_SLOTS];

static bool kf_addr_ok(uint8_t addr) { return addr >= 1 && addr < KF_SLOTS; }

static void kf_cache_put(uint8_t addr, const uint8_t *kf, int len) {
    if (!kf_addr_ok(addr) || len <= 0 || len > DU_RESP_MAX) return;
    memcpy(s_kf_cache[addr], kf, (size_t)len);
    s_kf_cache_len[addr] = len;
    s_kf_stale[addr]     = false;
}

void du_kfactor_mark_stale(void) {
    /* Only the nozzles that have answered a k-factor read before: they will
     * answer the re-read too, at one frame's cost. One that never answered
     * keeps waiting for its scheduled read rather than spending its full retry
     * budget on every round. */
    for (int a = 1; a < KF_SLOTS; a++)
        if (s_kf_cache_len[a] > 0) s_kf_stale[a] = true;
}

/* ── BlueSky polling ──────────────────────────────────────────────────── */
void du_poll_bluesky_nozzle(uint8_t addr, DuNozzle_t *nz, bool read_kfactor) {
    memset(nz, 0, sizeof(*nz));
    nz->present = false;

    uint8_t cmd[5];

    /* ── Status gate ──────────────────────────────────────────────────────
     * The data reads happen ONLY if the nozzle reports idle. See the
     * DU_STATUS_RETRIES note in config.h for why (reading mid-delivery splits
     * one sale into two cloud transactions) and for why 0x00 counts as idle
     * alongside the documented 0x80.
     *
     * Two distinct failures are handled here and must not be conflated:
     *   - no valid frame at all  → nozzle absent / wiring fault → 404
     *   - valid frame, not idle  → nozzle is fuelling → skip reads, try again
     *                              next cycle (NOT a fault)
     * Retries cover the first case; the second is a normal operating state. */
    bluesky_frame(addr, BS_CMD_STATUS, cmd);

    for (int attempt = 1; attempt <= DU_STATUS_RETRIES; attempt++) {
        if (attempt > 1) vTaskDelay(pdMS_TO_TICKS(DU_STATUS_RETRY_MS));

        nz->status_len = rs485_xfer(cmd, 5, nz->status, DU_RESP_MAX);
        esp_task_wdt_reset();
        log_raw_frame(addr, "status", nz->status, nz->status_len);

        if (nz->status_len > 0 &&
            !bluesky_read_valid(nz->status, nz->status_len, addr, BS_CMD_STATUS, "status"))
            nz->status_len = 0;

        if (nz->status_len < 4) {
            ESP_LOGW(TAG, "BlueSky nz%u status: no valid frame (attempt %d/%d)",
                     addr, attempt, DU_STATUS_RETRIES);
            continue;                       /* comms problem — worth retrying */
        }

        nz->present = true;
        uint8_t st  = nz->status[3];
        if (st == 0x80 || st == 0x00) { nz->idle = true; break; }

        ESP_LOGI(TAG, "BlueSky nz%u status 0x%02x = delivering (attempt %d/%d)",
                 addr, st, attempt, DU_STATUS_RETRIES);
    }

    /* Nothing ever answered — physically absent or a dead link. Skipping the
     * data reads keeps an unpopulated slot at one status transaction's cost. */
    if (!nz->present) return;

    /* Answered, but still fuelling. Leave the totalizer alone until the sale
     * completes so the next reading lands on a settled value. */
    if (!nz->idle) {
        nz->busy = true;
        ESP_LOGI(TAG, "BlueSky nz%u delivering — data reads skipped this cycle", addr);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(30));

    /* ── Data reads — reached only with the nozzle confirmed idle ──────────
     * Each read validates its own frame; an unanswered or invalid read leaves
     * that field's length at 0, so the payload builder reports "404" for it
     * instead of forwarding noise. RFID is deliberately not read at all. */

    /* Totalizer — THE sale-critical value, so it is retried up to
     * DU_TOTAL_RETRIES times before giving up on this cycle. */
    nz->total_len = du_read_field(addr, BS_CMD_TOTAL, nz->total, DU_TOTAL_RETRIES, "total");
    vTaskDelay(pdMS_TO_TICKS(30));

    /* RSP — retried like the totalizer (DU_RSP_RETRIES, config.h). */
    nz->rsp_len = du_read_field(addr, BS_CMD_RSP, nz->rsp, DU_RSP_RETRIES, "rsp");
    vTaskDelay(pdMS_TO_TICKS(30));

    /* ── K-Factor: read only when due, carried forward otherwise ───────────
     * On a non-due round — or a due round whose retries all fail — the cached
     * value (see s_kf_cache) is copied into nz->kfact, so every published
     * message carries a stable k-factor without spending bus time re-reading a
     * value that changes only on recalibration. Zero-length until the first
     * good read, which the payload builder renders as "404". A tunnel command
     * since the last good read (s_kf_stale) makes the read due every round
     * until one succeeds. */
    bool addr_cacheable = kf_addr_ok(addr);
    bool stale          = addr_cacheable && s_kf_stale[addr];

    if (read_kfactor || stale) {
        nz->kfact_len = du_read_field(addr, BS_CMD_KFACT, nz->kfact, DU_KFACTOR_RETRIES, "kfact");
        if (nz->kfact_len > 0) {
            kf_cache_put(addr, nz->kfact, nz->kfact_len);
        } else if (addr_cacheable && s_kf_cache_len[addr] > 0) {
            /* Due, but every attempt failed — keep the last good value rather
             * than regress a known k-factor to 404 on a transient miss. */
            memcpy(nz->kfact, s_kf_cache[addr], (size_t)s_kf_cache_len[addr]);
            nz->kfact_len = s_kf_cache_len[addr];
            ESP_LOGW(TAG, "BlueSky nz%u kfact: read failed — carrying last good value forward%s",
                     addr, stale ? " (a tunnel command may have changed it; re-reading next round)" : "");
        }
    } else if (addr_cacheable && s_kf_cache_len[addr] > 0) {
        memcpy(nz->kfact, s_kf_cache[addr], (size_t)s_kf_cache_len[addr]);
        nz->kfact_len = s_kf_cache_len[addr];
    }
    /* RFID intentionally not read. */
}

/* ── Hex helpers ──────────────────────────────────────────────────────── */
void du_bytes_to_hex(const uint8_t *data, int len, char *out, size_t out_size) {
    int pos = 0;
    for (int i = 0; i < len && pos + 4 < (int)out_size; i++) {
        if (i > 0) out[pos++] = ',';
        pos += snprintf(out + pos, out_size - pos, "%02x", data[i]);
    }
    out[pos] = '\0';
}

/* snprintf returns the length it WANTED to write, not what actually fit. If
 * a section truncates, `pos` sails past buf_size and the next call's
 * (buf_size - pos) — both promoted to size_t — wraps to a huge value, turning
 * a truncated field into an out-of-bounds write. Same pattern already fixed
 * in genset.c; applied here since this function chains snprintf the same way. */
static inline int clamp_pos(int pos, size_t buf_size) {
    if (pos < 0) return 0;   /* snprintf returns <0 on encoding error — never let
                              * `pos` go negative, or buf+pos / buf_size-pos below
                              * would address outside the buffer. */
    return (pos > (int)buf_size - 1) ? (int)buf_size - 1 : pos;
}

/* ── JSON payload builder (one nozzle per message) ────────────────────────
 * The "nozzles" wrapper is deliberately retained so the cloud parser sees the
 * exact structure it already consumes; only the number of nozzles inside drops
 * to one:
 *   {"HW_ID":"..","nozzles":{"1":{"data":"..","rsp":"..","kfact":"..","rfid":"404"}},"TS":".."}
 *
 * This is also what the BlueSky protocol doc specifies (one message per
 * nozzle), and it halves the payload: worst case ~500 B instead of ~920 B,
 * putting every dispenser message comfortably under the modem's ~1400-byte
 * QMTPUBEX ceiling (MQTT_MODEM_PAYLOAD_SAFE_MAX) rather than near it.
 *
 * "rfid" IS EMITTED BUT NEVER READ. The fleet's cloud pipeline was built
 * against the original device shape, which carried an rfid field, and the
 * decision (2026-09-21) was to keep that key present so the ingested record
 * is byte-for-byte the shape the cloud already stores — while NOT spending a
 * bus transaction on it. It is therefore the constant "404", the same value a
 * failed read would produce, so a consumer that decodes it sees "no RFID" and
 * a consumer that ignores it is unaffected. "kfact" is the last value read on
 * the 15-min k-factor cadence, carried forward in between. */
void du_build_nozzle_payload(const DuNozzle_t *nz, int label,
                             const char *hw_id, const char *ts,
                             char *buf, size_t buf_size) {
    int pos = 0;

    pos += snprintf(buf + pos, buf_size - pos,
                    "{\"HW_ID\":\"%s\",\"nozzles\":{", hw_id);
    pos = clamp_pos(pos, buf_size);

    /* Busy publishes the same "404" shape as absent, so the cloud parser needs
     * no change to accept it — but the two are tracked separately in DuNozzle_t
     * and logged differently, because only one of them is a fault. If the cloud
     * ever wants to tell "pump is mid-sale" from "pump is unreachable", this is
     * the single line to change (e.g. {"state":"busy"}); it needs a matching
     * cloud-side parser update, so it is deliberately NOT done unilaterally. */
    if (!nz->present || nz->busy) {
        pos += snprintf(buf + pos, buf_size - pos,
                        "\"%d\":{\"data\":\"404\"}", label);
        pos = clamp_pos(pos, buf_size);
    } else {
        /* Reached only for a nozzle confirmed idle — any individual field whose
         * read got no valid response becomes "404". */
        char h_tot[128];
        char h_rsp[128];
        char h_kf [128];
        if (nz->total_len > 0) du_bytes_to_hex(nz->total, nz->total_len, h_tot, sizeof(h_tot));
        else strcpy(h_tot, "404");
        if (nz->rsp_len > 0)   du_bytes_to_hex(nz->rsp,   nz->rsp_len,   h_rsp, sizeof(h_rsp));
        else strcpy(h_rsp, "404");
        if (nz->kfact_len > 0) du_bytes_to_hex(nz->kfact, nz->kfact_len, h_kf,  sizeof(h_kf));
        else strcpy(h_kf, "404");
        pos += snprintf(buf + pos, buf_size - pos,
                        "\"%d\":{\"data\":\"%s\",\"rsp\":\"%s\",\"kfact\":\"%s\",\"rfid\":\"404\"}",
                        label, h_tot, h_rsp, h_kf);
        pos = clamp_pos(pos, buf_size);
    }

    pos += snprintf(buf + pos, buf_size - pos,
                    "},\"TS\":\"%s\"}", ts);
    pos = clamp_pos(pos, buf_size);

    /* clamp_pos prevents memory corruption but not broken JSON syntax — a
     * truncated payload (missing closing braces, possibly mid-string) would
     * be silently dropped by the cloud parser. Landing within 2 bytes of the
     * ceiling on a buffer sized with wide headroom over the worst case is
     * itself a strong "something's wrong" signal, so fall back to a
     * guaranteed-valid 404 shape instead. */
    if (pos >= (int)buf_size - 2) {
        ESP_LOGW(TAG, "nz%d payload truncated at %d/%zu bytes — publishing 404 instead",
                 label, pos, buf_size);
        snprintf(buf, buf_size,
                 "{\"HW_ID\":\"%s\",\"nozzles\":{\"%d\":{\"data\":\"404\"}},\"TS\":\"%s\"}",
                 hw_id, label, ts);
    }
}
