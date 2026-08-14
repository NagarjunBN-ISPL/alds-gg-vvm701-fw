#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Max DATA bytes captured from the registers read (Modbus header and CRC are
 * stripped — only the register payload is kept). Largest block in use is the
 * SGC 120: 78 registers x 2 = 156 bytes. */
#define GENSET_REG_DATA_MAX 160
/* Max DATA bytes from the status read: 2 registers x 2 = 4 bytes. */
#define GENSET_ST_DATA_MAX  8

/* ── Raw genset reading (cloud-side decode) ───────────────────────────────
 * Per the DG integration spec, a genset poll issues up to TWO Modbus reads:
 *
 *   1. "registers" — the measurement block (every model).
 *   2. "status"    — the DG status word(s) (every model except the SGC 120,
 *                    whose single overall read already covers everything).
 *
 * Each read's DATA bytes are stored here verbatim — no scaling, no decode —
 * and published as its OWN MQTT message (see genset_build_*_payload), so one
 * poll of a two-command model produces two messages under the same HW_ID.
 * The cloud owns the register maps and all conversion.
 *
 * The separate alarm command documented for GC1211/GC1111 is deliberately NOT
 * issued: alarm state is carried within the register/status blocks the cloud
 * already decodes, and skipping it keeps the genset off the shared RS485 bus
 * for one fewer transaction per poll. */
typedef struct {
    /* Command 1 — measurement registers */
    bool    reg_valid;
    uint8_t reg_data[GENSET_REG_DATA_MAX];
    int     reg_len;

    /* Command 2 — DG status. has_status reflects the MODEL (set even when the
     * reads fail), so the caller knows whether a status message is expected at
     * all; st_valid reflects whether this poll actually got it. */
    bool    has_status;
    bool    st_valid;
    uint8_t st_data[GENSET_ST_DATA_MAX];
    int     st_len;
} GensetData_t;

/* RS485 must be configured for 8N1 at the desired baud before calling.
 * `model` selects the read set (see GENSET_MODEL_* in config.h). Returns true
 * only if every read the model defines succeeded; individual results are in
 * reg_valid / st_valid so a partial success still publishes what it got. */
bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out);

/* Serialise each reading to its own JSON message in buf (includes HW_ID and
 * TS). Data is emitted as a comma-separated lowercase hex string — the same
 * shape radar and dispenser use. A failed read emits "404" for that key. */
void genset_build_registers_payload(const GensetData_t *data,
                                    const char *hw_id, const char *ts,
                                    char *buf, size_t buf_size);
void genset_build_status_payload(const GensetData_t *data,
                                 const char *hw_id, const char *ts,
                                 char *buf, size_t buf_size);
