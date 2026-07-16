#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Maximum active alarms held in one reading. DEIF SGC 120 Mk II exposes 48
 * alarm bits (12 registers x 4 nibbles); Sedemac GC1211/GC1111 expose 40
 * (10 x 4). Sized to the largest so any single model's full alarm set fits. */
#define GENSET_MAX_ALARMS 48

typedef struct {
    const char *name;   /* alarm parameter name (points into a static table)  */
    uint8_t     state;  /* 2=Warning 3=Shutdown 4=ElecTrip 5=Notification      */
} GensetAlarm_t;

/* ── Normalised, model-agnostic genset reading ────────────────────────────
 * Every model driver fills this with already-scaled physical values;
 * genset_build_payload() then serialises it to JSON without needing to know
 * which controller produced it. To add a new genset model you write ONE read
 * function that fills this struct and register it in the driver table in
 * genset.c — the payload builder, the JSON schema and main.c never change. */
typedef struct {
    bool valid;

    /* Measurements — already scaled to physical units by the model driver. */
    uint16_t protocol_rev;
    float    lube_oil_pressure;
    int      engine_temp;
    uint16_t fuel_level;
    float    charging_alt_voltage;
    float    battery_voltage;
    uint16_t no_of_starts;
    uint32_t engine_run_time;
    uint16_t engine_speed;
    float    genset_freq;
    float    genset_voltage[3];   /* L1, L2, L3        */
    float    load_current[3];     /* L1, L2, L3        */
    float    genset_power[4];     /* L1, L2, L3, total */

    /* Optional: cumulative energy, regs 35-36 (Sedemac only) — 32-bit value
     * spanning two registers, word order NOT wire-verified (assumed
     * high-word-first per standard Modbus convention). Not currently read
     * by any DEIF SGC 120 driver. */
    bool     has_kwh;
    uint32_t kwh;

    /* Optional: raw values of the first three alarm registers (Sedemac only),
     * alongside — not instead of — the decoded `alarms[]` above. */
    bool     has_alr;
    uint16_t alr1, alr2, alr3;

    /* Active alarms — only Warning/Shutdown/ElecTrip/Notification are stored;
     * Disabled/Inactive are dropped at decode time. */
    GensetAlarm_t alarms[GENSET_MAX_ALARMS];
    int  alarm_count;
    bool alarms_truncated;        /* more alarms exist than were captured      */

    /* Status */
    bool        available;
    bool        running;
    bool        stopped_normally;
    bool        stopped_with_fault;
    const char *mode;             /* "Auto"/"Manual"/"Scheduler"/"Unknown"     */

    /* Optional: command-status readback (Sedemac only)                        */
    bool        has_command_status;
    const char *command_status;

    /* Optional: holding-register readback (Sedemac GC1211 family only)        */
    bool     has_holding;
    uint16_t holding_command;
    uint16_t holding_operating_mode;
} GensetData_t;

/* RS485 must be configured for 8N1 at the desired baud before calling.
 * `model` selects the driver (see GENSET_MODEL_* in config.h). Returns false
 * (and leaves out->valid = false) if the model is unknown or the reads fail. */
bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out);

/* Serialise a reading to JSON in buf (includes HW_ID and TS). Model-agnostic. */
void genset_build_payload(const GensetData_t *data,
                          const char *hw_id, const char *ts,
                          char *buf, size_t buf_size);
