#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Max DATA bytes captured from the registers read (Modbus header and CRC are
 * stripped — only the register payload is kept). Largest block in use is the
 * SGC 120: 78 registers x 2 = 156 bytes. */
#define GENSET_REG_DATA_MAX 160
/* Max DATA bytes from the alarm read: 3 registers x 2 = 6 bytes. */
#define GENSET_AL_DATA_MAX  8
/* Max DATA bytes from the status read: 2 registers x 2 = 4 bytes. */
#define GENSET_ST_DATA_MAX  8

/* ── Raw genset reading (cloud-side decode) ───────────────────────────────
 * Per the DG integration spec, a genset poll issues up to THREE Modbus reads,
 * in the spec's order:
 *
 *   1. "registers" — the measurement block (every model).
 *   2. "Alarms"    — the alarm registers, Mahindra GC1211/GC1111 only. The
 *                    capital A is the spec's spelling: keep it.
 *   3. "status"    — the DG status word(s) (every model except the SGC 120,
 *                    whose single overall read already holds its alarm and
 *                    status registers).
 *
 * Each read's DATA bytes are stored here verbatim — no scaling, no decode —
 * and published as its OWN MQTT message (see genset_build_*_payload), so one
 * Mahindra poll produces three messages under the same HW_ID and TS.
 * The cloud owns the register maps and all conversion. */
typedef struct {
    /* Command 1 — measurement registers */
    bool    reg_valid;
    uint8_t reg_data[GENSET_REG_DATA_MAX];
    int     reg_len;

    /* Command 2 — alarm registers. has_alarms reflects the MODEL (set even
     * when the read fails), so the caller knows whether an Alarms message is
     * expected at all; al_valid reflects whether this poll actually got it. */
    bool    has_alarms;
    bool    al_valid;
    uint8_t al_data[GENSET_AL_DATA_MAX];
    int     al_len;

    /* Command 3 — DG status. Same has_/valid split as the alarms above. */
    bool    has_status;
    bool    st_valid;
    uint8_t st_data[GENSET_ST_DATA_MAX];
    int     st_len;
} GensetData_t;

/* RS485 must be configured for 8N1 at the desired baud before calling.
 * `model` selects the read set (see GENSET_MODEL_* in config.h). Returns true
 * only if every read the model defines succeeded; individual results are in
 * reg_valid / al_valid / st_valid so a partial success still publishes what
 * it got. */
bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out);

/* Which messages a poll of `model` produces, with no bus traffic: sets
 * has_alarms / has_status from the model table and touches nothing else (an
 * unknown model expects neither). genset_poll() uses it itself; poll_genset()
 * calls it up front so a cycle in which the bus is never obtained still
 * publishes EVERY expected message as "404", not just the registers one. */
void genset_expect(uint8_t model, GensetData_t *out);

/* Serialise each reading to its own JSON message in buf (includes HW_ID and
 * TS). Data is emitted as a comma-separated lowercase hex string — the same
 * shape radar and dispenser use. A failed read emits "404" for that key. */
void genset_build_registers_payload(const GensetData_t *data,
                                    const char *hw_id, const char *ts,
                                    char *buf, size_t buf_size);
void genset_build_alarms_payload(const GensetData_t *data,
                                 const char *hw_id, const char *ts,
                                 char *buf, size_t buf_size);
void genset_build_status_payload(const GensetData_t *data,
                                 const char *hw_id, const char *ts,
                                 char *buf, size_t buf_size);
