#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t g_di_task_handle;

/* One queued DI transition: the enabled inputs' states as JSON, and the UTC
 * time the change was seen (ts_ok false = the clock was not set yet then).
 * The longest payload is 4 x "DIn":-2147483648 + 3 commas + braces = 73 B. */
#define DI_PAYLOAD_MAX 96
typedef struct {
    char payload[DI_PAYLOAD_MAX];
    char ts[24];
    bool ts_ok;
} DIEntry_t;

extern DIEntry_t      di_fifo[DI_FIFO_SIZE];
extern volatile int   di_head, di_tail, di_count;

void di_uart_init(void);
void di_set_enable(uint8_t mask);  /* call before starting di_task */
void di_task(void *param);         /* pin to Core 1 */

/* Drain buffered DI payloads and publish each, with TS = the time its change
 * was seen; ts_latest (now) only for a change seen before the clock was set.
 * Returns true if all publishes succeeded (or nothing was buffered), false if
 * any publish failed — used to feed g_diag.di_ok. */
bool di_publish_buffer(const char *topic, const char *hw_id,
                       const char *ts_latest,
                       uint8_t di_enable);
