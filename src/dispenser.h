#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DU_RESP_MAX 32

typedef struct {
    bool    present;
    bool    idle;
    uint8_t status[DU_RESP_MAX]; int status_len;
    uint8_t total[DU_RESP_MAX];  int total_len;
    uint8_t rsp[DU_RESP_MAX];    int rsp_len;
    uint8_t kfact[DU_RESP_MAX];  int kfact_len;
    uint8_t rfid[DU_RESP_MAX];   int rfid_len;
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
 * two nozzles never finish a sale at the same moment. */
void du_poll_bluesky_nozzle(uint8_t addr, DuNozzle_t *out);

/* Build the JSON payload for a SINGLE nozzle (caller provides HW_ID and TS).
 * `label` is the global nozzle number (1-based) used as the JSON key. */
void du_build_nozzle_payload(const DuNozzle_t *nz, int label,
                             const char *hw_id, const char *ts,
                             char *buf, size_t buf_size);

/* Convert raw bytes to comma-separated hex string. */
void du_bytes_to_hex(const uint8_t *data, int len, char *out, size_t out_size);

/* Raw hex passthrough — cmd_hex carries its own target address, so this
   is protocol/unit-agnostic. Puts response hex into resp_buf.
   Returns false on no reply. */
bool du_exec_bluesky_rsp(const char *cmd_hex, char *resp_buf, size_t resp_size);

/* Execute a BlueSky K-Factor command (same passthrough as above). */
bool du_exec_bluesky_kfactor(const char *cmd_hex, char *resp_buf, size_t resp_size);
