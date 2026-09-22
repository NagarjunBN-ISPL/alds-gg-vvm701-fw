#pragma once
#include <stdint.h>
#include <stdbool.h>

/* RS485 must be in 8N1 mode before calling these.
   slave is the Modbus slave address — the tank's numeric HW_ID suffix (or the
   field-convention fallback: Ztech from ZTECH_SLAVE_BASE, Pantech continuing
   after the last Ztech tank). See get_radar_slave() in main.c.
   Returns raw response bytes; len set to 0 on failure. */
bool radar_ztech_read(uint8_t slave, int tank_idx, uint8_t *rx_buf, int *rx_len);
bool radar_pantech_read(uint8_t slave, int tank_idx, uint8_t *rx_buf, int *rx_len);
