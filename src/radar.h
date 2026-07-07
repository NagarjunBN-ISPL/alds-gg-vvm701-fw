#pragma once
#include <stdint.h>
#include <stdbool.h>

/* RS485 must be in 8N1 mode before calling these.
   tank_idx is 0-based and independent per type: Ztech uses Modbus slave
   ZTECH_SLAVE_BASE+tank_idx (4,5), Pantech uses PANTECH_SLAVE_BASE+tank_idx
   (6,7). Returns raw response bytes; len set to 0 on failure. */
bool radar_ztech_read(int tank_idx, uint8_t *rx_buf, int *rx_len);
bool radar_pantech_read(int tank_idx, uint8_t *rx_buf, int *rx_len);
