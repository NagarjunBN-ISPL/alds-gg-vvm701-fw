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

/* unit_idx is 0-based (0 = first BlueSky unit). Each BlueSky unit occupies 2
 * RS485 addresses (unit_idx*2+1, unit_idx*2+2). */
void du_poll_bluesky_unit(int unit_idx, DuNozzle_t *out_a, DuNozzle_t *out_b);

/* Build JSON payload for one DU into buf (caller provides HW_ID and TS). */
void du_build_payload(const DuNozzle_t *nz_a, const DuNozzle_t *nz_b,
                      int label_a, int label_b,   /* global nozzle numbers */
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
