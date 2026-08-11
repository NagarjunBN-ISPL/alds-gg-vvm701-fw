#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Upper bound on registers captured by a raw single-command read. A Modbus
 * read is itself capped at 125 registers, so this is the honest ceiling; the
 * models today read 67 (GC1200), 68 (GC1211/GC1111), 70 (GC902), 79 (SGC120). */
#define GENSET_RAW_MAX_REGS 125

/* One named field carved out of the raw register block: raw_regs[start ..
 * start+count-1] published under `key`, as a JSON array when as_array is true
 * or a bare scalar (count == 1) when false. The cloud owns all scaling; a slice
 * only names a register range, it never interprets it. */
typedef struct {
    const char *key;
    int         start;      /* raw_regs[] start index                    */
    int         count;      /* registers in the field (>=1; 1 for scalar) */
    bool        as_array;   /* true -> [ .. ]; false -> scalar            */
} GensetRawSlice_t;

/* ── Raw genset reading (model-agnostic) ──────────────────────────────────
 * A driver issues ONE Modbus read and stores the raw 16-bit register values
 * here — no scaling, no decode. genset_build_payload() emits them verbatim as
 * decimal JSON fields per raw_slices, and the cloud owns the register map and
 * all conversion. To add a model you write a genset_read_raw() wrapper (its
 * fc/start/count + a static GensetRawSlice_t[] slice list) and a driver-table
 * row in genset.c; this struct, the serialiser and main.c never change. */
typedef struct {
    bool     valid;
    bool     is_raw;                      /* set by the raw reader; all models raw */
    uint16_t raw_regs[GENSET_RAW_MAX_REGS];
    int      raw_reg_count;               /* registers parsed from the response   */
    const GensetRawSlice_t *raw_slices;   /* how to name-slice raw_regs into JSON  */
    int      raw_slice_count;
} GensetData_t;

/* RS485 must be configured for 8N1 at the desired baud before calling.
 * `model` selects the driver (see GENSET_MODEL_* in config.h). Returns false
 * (and leaves out->valid = false) if the model is unknown or the reads fail. */
bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out);

/* Serialise a reading to JSON in buf (includes HW_ID and TS). Model-agnostic. */
void genset_build_payload(const GensetData_t *data,
                          const char *hw_id, const char *ts,
                          char *buf, size_t buf_size);
