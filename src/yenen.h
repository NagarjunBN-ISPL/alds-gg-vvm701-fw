#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * yenen.h — Yenen dispenser driver (readings forwarded raw to the cloud).
 *
 * Same contract as the BlueSky driver in dispenser.h: this layer frames,
 * transacts and validates, but never interprets a field. The cloud owns the
 * decode (litres / amount / RSP / totalizer), exactly as it does for BlueSky,
 * radar and genset. Keeping interpretation out of the firmware is what lets a
 * protocol correction ship as a cloud change instead of a fleet-wide FOTA.
 *
 * ── ONE COMMAND PER NOZZLE ────────────────────────────────────────────────
 * Only Command 2 is issued (decision 2026-08-20):
 *
 *   Command 2 — totalizer / total amount / RSP
 *       02 30 38 30 <head> <nzcode> 41 03 <ck_hi> <ck_lo>
 *       = STX "08" "0" <head> <nzcode> "A" ETX <checksum>
 *
 * The vendor doc also describes a Command 1 (`16 30 <head> <nzcode>`, 4 bytes,
 * no checksum) returning the last transaction's litres and amount. It is no
 * longer sent, and the driver no longer has any notion of it. Three reasons,
 * in order of weight:
 *
 *   1. Command 2's reply already carries the same last-delivery litres and
 *      amount (bytes [18:27] of the 51-byte frame), so nothing is lost.
 *   2. Not every unit implements it. One bench pump answered it normally;
 *      another returned nothing to it at all while answering Command 2
 *      perfectly on the same nozzle milliseconds later, with the command bytes
 *      verified byte-identical to the doc.
 *   3. A silent command costs YN_TOTAL_RETRIES x RS485_RECV_TIMEOUT_MS = 6 s
 *      of dead air per nozzle per round. On a 4-nozzle site that was 24 s of a
 *      12 s cycle spent waiting for a reply that never comes — it starved the
 *      totalizer reads that actually matter.
 *
 * This also halves the worst-case RS485 bus hold per nozzle (see the two wait
 * budgets in config.h) and makes the published payload exactly the shape the
 * cloud already consumes for every other dispenser: one "data" field, nothing
 * else.
 *
 * ── Addressing ────────────────────────────────────────────────────────────
 * <head>   is one ASCII digit, '0' + head_number (head 1 -> '1').
 * <nzcode> is one ASCII digit identifying the nozzle side on that head:
 *              side A -> '0' + (head - 1)
 *              side B -> '5' + (head - 1)
 * Confirmed against every frame in the vendor doc:
 *      DU1 NZ1 -> '0'   DU1 NZ2 -> '5'
 *      DU2 NZ3 -> '1'   DU2 NZ4 -> '6'
 * This scheme is what caps DU_YENEN_MAX at 5: head 6 side A would be '5',
 * which already means head 1 side B. If the vendor documents a different
 * encoding above 5 heads, that cap is the single place to revisit.
 *
 * ── Checksum ──────────────────────────────────────────────────────────────
 * XOR of every byte from STX through ETX inclusive, then the two nibbles of
 * that result are each emitted as (nibble + 0x30) — NOT as ASCII hex.
 * That distinction is load-bearing: XOR 0x7C encodes as 37 3C ('7' '<'), not
 * 37 43 ('7' 'C'). Verified byte-exact against all four documented Command 2
 * frames (79 / 7C / 7B / 7C). See yenen_checksum().
 */

#define YN_RESP_MAX 64   /* longest documented reply = Command 2 = 51 bytes */

typedef struct {
    bool    present;                           /* a valid frame came back     */
    uint8_t total[YN_RESP_MAX]; int total_len; /* Command 2 reply (raw)       */
} YenenNozzle_t;

/* RS485 UART must already be installed before calling this. */
void yn_rs485_set_yenen_mode(void);

/* Poll ONE nozzle. head is 1-based (1..DU_YENEN_MAX — the dispenser number, du_number_of); side is 0 = A, 1 = B.
 * Issues Command 2 only; if every retry fails the reply is left zero-length
 * and published as "404". Per-nozzle rather than per-unit for the same reason
 * as BlueSky: the caller publishes each nozzle the instant its read completes
 * instead of waiting for its partner. */
void yn_poll_nozzle(uint8_t head, uint8_t side, YenenNozzle_t *out);

/* Build the JSON payload for a SINGLE nozzle (caller provides HW_ID and TS).
 * `label` is the global nozzle number (1-based) used as the JSON key.
 *
 *   {"HW_ID":"<imei+sfx>","nozzles":{"<label>":{"data":"<hex>"}},"TS":"<ts>"}
 *
 * "data" is the Command 2 reply as lowercase comma-separated hex, or "404".
 * This is byte-for-byte the shape the cloud already parses for the BlueSky
 * fleet — one "data" field per nozzle and nothing else. */
void yn_build_nozzle_payload(const YenenNozzle_t *nz, int label,
                             const char *hw_id, const char *ts,
                             char *buf, size_t buf_size);

/* ── Commands to the pump: the cloud's, not this driver's ─────────────────
 * This driver only READS (Command 2). An RSP (unit price) change — Command K,
 *   02 30 38 30 <'0'+head> <nzcode> 4B <p1><p2><p3><p4> 03 <ck_hi> <ck_lo>
 * answered 06 30 31 (ACK) or 15 (NAK) — is built by the cloud and carried
 * byte for byte by the rsp tunnel (rsp_tunnel.h); the gateway neither frames
 * it nor checks it (user decision 2026-10-18). The gateway-built price write
 * (setrsp, yn_set_rsp) is gone, and so is any k-factor topic for Yenen — the
 * protocol has no k-factor. */
