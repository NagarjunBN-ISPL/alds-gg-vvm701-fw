#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t  du_bluesky_count;  /* 0-6                              */
    uint8_t  du_yenen_count;    /* 0-6                              */
    uint8_t  ztech_count;       /* 0-4 (ztech_count+pantech_count<=4) */
    uint8_t  pantech_count;     /* 0-4                              */
    uint8_t  genset_type;       /* 0=Off,1=SEDEMAC                  */
    uint8_t  genset_slave;      /* Modbus slave, default 1          */
    uint32_t genset_baud;       /* 4800/9600/19200, default 9600    */
    uint8_t  di_enable;         /* bitmask: bit0=DI1..bit3=DI4      */
    char     sfx_genset[3];     /* 2-char suffix + NUL              */
    char     sfx_di[3];
} DeviceConfig;

void nvs_config_defaults(DeviceConfig *cfg);
bool nvs_config_load(DeviceConfig *cfg);
bool nvs_config_save(const DeviceConfig *cfg);
void nvs_config_factory_reset(void);
