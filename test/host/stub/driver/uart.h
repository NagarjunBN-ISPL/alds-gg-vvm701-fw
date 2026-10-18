/* Host stub for driver/uart.h — just enough of the surface yenen.c uses.
 * The bytes go to the simulated pump or line of the suite that links it
 * (test_yenen.c, test_kfactor.c, test_rsp_tunnel.c, ...). */
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef int uart_port_t;
#define UART_NUM_0 0
#define UART_NUM_1 1
#define UART_NUM_2 2

#define UART_DATA_8_BITS       3
#define UART_PARITY_DISABLE    0
#define UART_PARITY_EVEN       2
#define UART_STOP_BITS_1       1
#define UART_HW_FLOWCTRL_DISABLE 0

typedef struct {
    int baud_rate;
    int data_bits;
    int parity;
    int stop_bits;
    int flow_ctrl;
} uart_config_t;

int uart_param_config(uart_port_t port, const uart_config_t *cfg);
int uart_write_bytes(uart_port_t port, const void *src, size_t size);
int uart_read_bytes(uart_port_t port, void *buf, uint32_t length, uint32_t ticks_to_wait);
