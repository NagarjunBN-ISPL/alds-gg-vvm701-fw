#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    bool     valid;
    bool     has_holding;  /* batch3 read (GC1211 family only)              */
    uint16_t batch1[23];   /* reg 0..22  (FC04 GC1211 / FC03 GC1111)        */
    uint16_t batch2[12];   /* reg 57..68 (FC04 GC1211 / FC03 GC1111)        */
    uint16_t batch3[2];    /* FC03 holding reg 0..1 — GC1211 family only    */
} GensetData_t;

/* RS485 must be configured for 8N1 at the desired baud before calling.
 * model = GENSET_MODEL_GC1211 (FC04) or GENSET_MODEL_GC1111 (FC03). */
bool genset_poll(uint8_t slave, uint8_t model, GensetData_t *out);

/* Format scaled JSON into buf (includes HW_ID and TS). */
void genset_build_payload(const GensetData_t *data,
                          const char *hw_id, const char *ts,
                          char *buf, size_t buf_size);
