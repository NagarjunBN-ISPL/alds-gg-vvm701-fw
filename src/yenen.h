#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * yenen.h — Yenen dispenser driver (raw-byte passthrough to cloud).
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

/* Poll ONE nozzle. head is 1-based (1..DU_YENEN_MAX — the station-list position); side is 0 = A, 1 = B.
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

/* Raw hex passthrough — cmd_hex carries its own address bytes, so this is
 * command-agnostic. Puts the response hex into resp_buf.
 * Returns false on no reply. */
bool yn_exec_passthrough(const char *cmd_hex, char *resp_buf, size_t resp_size);

/* ── RSP (unit price) update — the one command this firmware WRITES ────────
 *
 * Frame (vendor doc "Protocol commands for Yenen Dispenser and its RSP
 * update"), 14 bytes:
 *
 *   02 30 38 30 <'0'+head> <nzcode> 4B <p1><p2><p3><p4> 03 <ck_hi> <ck_lo>
 *
 * It is Command 2's frame with the command byte 'A' (0x41, read) replaced by
 * 'K' (0x4B, write) and four ASCII price digits inserted before ETX. The price
 * is rupees x100 — 65.32 is "6532" — so the protocol ceiling is 99.99 and
 * there is no place for a third decimal.
 *
 * The checksum is the SAME XOR-then-nibble+0x30 scheme as every other frame,
 * which is worth stating because it is the one thing a hand-built frame
 * usually gets wrong. Verified against all eight worked examples in the doc:
 *   head1 A/B  65.32 -> 71 / 74      head1 A/B  65.65 -> 73 / 76
 *   head1 A/B  99.99 -> 73 / 76      head1 A/B  65.33 -> 70 / 75
 *
 * The reply is NOT an STX frame and carries no address echo, so none of the
 * response validation the poll path uses applies to it:
 *   06 30 31   accepted
 *   15         rejected (the doc's "considered as not ack")
 *
 * ── Why yn_set_rsp() is a sequence and not just that one write ─────────────
 * A bare write is unverifiable and unsafe here. The pump ACKs the frame it
 * received, not the price it stored, and the doc's own instruction is to check
 * the nozzle is idle before sending anything. So one update is:
 *
 *   1. read  (Command 2)  — is this nozzle idle, and what is the price now?
 *   2. write (Command K)  — set it, require the 06 30 31 ACK
 *   3. read  (Command 2)  — read the price back and compare
 *
 * Step 1 is what keeps a price change from landing mid-delivery, where it
 * would re-rate a sale already in progress. Step 3 is what lets the ack sent
 * to the cloud state the price the PUMP is holding rather than the price we
 * asked for. Both reads reuse the poll path's frame validation, including the
 * address echo — so a pump that answers for the wrong nozzle (seen in the
 * field, see yenen_resp_valid) can neither pass the idle gate nor confirm the
 * write.
 *
 * Call with the RS485 bus lock held and the Yenen line mode already set — the
 * same discipline every other transaction on this bus follows. */
typedef enum {
    YN_RSP_OK = 0,        /* ACKed and the read-back price matches            */
    YN_RSP_BAD_ARG,       /* address or price outside what the frame can hold */
    YN_RSP_NO_RESPONSE,   /* nozzle did not answer the pre-read               */
    YN_RSP_BUSY,          /* nozzle is mid-delivery — transient, worth retry  */
    YN_RSP_NAK,           /* pump answered 0x15, or something that isn't ACK  */
    YN_RSP_NO_ACK,        /* write got no answer at all                       */
    YN_RSP_VERIFY_FAILED, /* ACKed, but the read-back disagrees or failed     */
} YnRspResult_t;

/* Short machine-readable token for the result, for the cloud ack. */
const char *yn_rsp_result_str(YnRspResult_t r);

/* Set the unit price on ONE nozzle. head is 1-based, side 0 = A, 1 = B.
 * price_x100 is rupees x100 (6532 = 65.32), 1..9999.
 * *readback_x100 receives the price read back from the pump in step 3, or 0 if
 * the sequence never got that far. Safe to pass NULL. */
YnRspResult_t yn_set_rsp(uint8_t head, uint8_t side, uint16_t price_x100,
                         uint16_t *readback_x100);
