#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t g_di_task_handle;

typedef struct {
    char payload[256];
} DIEntry_t;

extern DIEntry_t      di_fifo[DI_FIFO_SIZE];
extern volatile int   di_head, di_tail, di_count;

void di_uart_init(void);
void di_set_enable(uint8_t mask);  /* call before starting di_task */
void di_task(void *param);         /* pin to Core 1 */

/* Drain buffered DI payloads and publish each; ts_latest used for TS field. */
void di_publish_buffer(const char *topic, const char *hw_id,
                       const char *ts_latest,
                       uint8_t di_enable);
