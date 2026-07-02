#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    bool     valid;
    uint16_t batch1[23];  /* FC04 reg 0..22  */
    uint16_t batch2[12];  /* FC04 reg 57..68 */
    uint16_t batch3[2];   /* FC03 reg 0..1   */
} GensetData_t;

/* RS485 must be configured for 8N1 at the desired baud before calling. */
bool genset_poll(uint8_t slave, GensetData_t *out);

/* Format scaled JSON into buf (includes HW_ID and TS). */
void genset_build_payload(const GensetData_t *data,
                          const char *hw_id, const char *ts,
                          char *buf, size_t buf_size);
