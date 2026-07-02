#pragma once
#include <stdint.h>
#include <stdbool.h>

/* RS485 must be in 8N1 mode before calling these.
   tank_idx is 0-based and independent per type: Ztech uses Modbus slave
   4+tank_idx, Pantech uses slave 7+tank_idx (matches OTA_Integration).
   Returns raw response bytes; len set to 0 on failure. */
bool radar_ztech_read(int tank_idx, uint8_t *rx_buf, int *rx_len);
bool radar_pantech_read(int tank_idx, uint8_t *rx_buf, int *rx_len);
