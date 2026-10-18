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
 * still carries a stable value without spending bus time re-reading it. After
 * du_kfactor_mark_stale() the next round reads the k-factor even when not due.
 * The totalizer is always read (with retries); RFID is never read. */
void du_poll_bluesky_nozzle(uint8_t addr, DuNozzle_t *out, bool read_kfactor);

/* A command from the cloud's rsp/kfactor tunnel (rsp_tunnel.h) has just
 * reached a BlueSky pump. The gateway does not read tunnel frames, so it
 * cannot know whether one changed a k-factor: every nozzle that has answered
 * a k-factor read is re-read on its next round instead of waiting up to
 * KFACTOR_POLL_INTERVAL_S. Call with the RS485 bus lock held. */
void du_kfactor_mark_stale(void);

/* Build the JSON payload for a SINGLE nozzle (caller provides HW_ID and TS).
 * `label` is the global nozzle number (1-based) used as the JSON key. */
void du_build_nozzle_payload(const DuNozzle_t *nz, int label,
                             const char *hw_id, const char *ts,
                             char *buf, size_t buf_size);

/* Convert raw bytes to comma-separated hex string. */
void du_bytes_to_hex(const uint8_t *data, int len, char *out, size_t out_size);
