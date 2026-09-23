#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DU_RESP_MAX 32

typedef struct {
    bool    present;   /* answered the status poll at all                    */
    bool    idle;      /* status byte 3 == 0x80 or 0x00 — safe to read data  */
    bool    busy;      /* answered, but delivering — data reads were skipped.
                        * Distinct from !present (no comms): a busy nozzle is
                        * the healthiest possible state, so callers must not
                        * treat it as a fault when scoring link health. */
    uint8_t status[DU_RESP_MAX]; int status_len;
    uint8_t total[DU_RESP_MAX];  int total_len;
    uint8_t rsp[DU_RESP_MAX];    int rsp_len;
    uint8_t kfact[DU_RESP_MAX];  int kfact_len;
    /* RFID is no longer read at all (removed to keep each poll round short —
     * see du_poll_bluesky_nozzle); the field is gone from the struct and the
     * published payload. */
} DuNozzle_t;

/* RS485 UART must already be installed before calling this. */
void du_rs485_set_bluesky_mode(void);   /* 8E1 9600 */

/* Poll ONE nozzle at RS485 address `addr` (1-based, global across the bus).
 * A BlueSky unit occupies two consecutive addresses: unit_idx*2+1 and
 * unit_idx*2+2, so DU1 = 1,2 · DU2 = 3,4 · DU3 = 5,6 ...
 *
 * Deliberately per-nozzle rather than per-unit: the caller publishes each
 * nozzle the instant its read completes, so NZ1's data reaches the cloud while
 * NZ2 is still being read instead of waiting ~10 s for its partner. This also
 * matches the BlueSky protocol doc, which specifies one JSON message per
 * nozzle, and is the shape an event-driven (idle→busy) capture needs, since
 * two nozzles never finish a sale at the same moment.
 *
 * read_kfactor: true only on the rounds where the k-factor is actually due
 * (every KFACTOR_POLL_INTERVAL_S — the caller gates this). When false, or when
 * a due read fails all its retries, the last successfully-read k-factor for
 * this address is carried forward into out->kfact so every published message
 * still carries a stable value without spending bus time re-reading it. A
 * passthrough k-factor write refreshes that value from its read-back; if the
 * read-back fails, the next round reads the k-factor even when not due. The
 * totalizer is always read (with retries); RFID is never read. */
void du_poll_bluesky_nozzle(uint8_t addr, DuNozzle_t *out, bool read_kfactor);

/* Build the JSON payload for a SINGLE nozzle (caller provides HW_ID and TS).
 * `label` is the global nozzle number (1-based) used as the JSON key. */
void du_build_nozzle_payload(const DuNozzle_t *nz, int label,
                             const char *hw_id, const char *ts,
                             char *buf, size_t buf_size);

/* Convert raw bytes to comma-separated hex string. */
void du_bytes_to_hex(const uint8_t *data, int len, char *out, size_t out_size);

/* ── rsp / kfactor passthrough, hardened ─────────────────────────────────
 * The cloud builds the BlueSky frame (it owns the write command set and its
 * checksum); the device puts it on the bus and returns the pump's raw reply as
 * hex. What the device DOES check before a byte goes out, because each of
 * these was a way to change the price of the wrong pump, or of a pump mid-sale:
 *
 *   1. it is a BlueSky frame — F5, self-describing length (byte[2] = 0xA<n>)
 *      matching the byte count; anything else is refused ("bad_frame");
 *   2. its target address is one of THIS dispenser's two nozzles (addr_a /
 *      addr_b — the topic it arrived on names the dispenser), else
 *      "addr_not_this_dispenser". The passthrough is per-dispenser precisely
 *      so a frame cannot reach a pump the operator did not select;
 *   3. the target nozzle answers its status poll and reports IDLE — the same
 *      gate every poll-round read uses. A write landing mid-delivery is the
 *      one outcome nobody can undo, so a busy nozzle is "nozzle_busy" and a
 *      silent one "nozzle_absent"; the operator re-sends when the sale ends.
 *
 * Then the frame is sent, retried ONCE if the pump did not answer with a valid
 * frame echoing its address ("no_reply" if it never does). On success resp_buf
 * holds the reply as comma-hex, and when the frame carried data (a write) the
 * pump's current value is read back with `readback_cmd` (BS_CMD_RSP or
 * BS_CMD_KFACT) into readback_buf as comma-hex so the operator can see what the
 * pump now holds. readback_buf is "" when nothing was read back.
 *
 * The caller owns the RS485 bus lock and line mode. `reason` is a stable token
 * for the refusal, set only when false is returned. */
#define BS_CMD_RSP    0xB6
#define BS_CMD_KFACT  0xE2
bool du_exec_bluesky_passthrough(const char *cmd_hex, uint8_t addr_a, uint8_t addr_b,
                                 uint8_t readback_cmd,
                                 char *resp_buf, size_t resp_size,
                                 char *readback_buf, size_t readback_size,
                                 char *reason, size_t reason_size);
