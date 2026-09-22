/* Virtual microsecond clock — advanced only by simulated blocking (UART waits
 * and vTaskDelay), so protocol timeouts are exercised deterministically and the
 * test runs instantly. */
#pragma once
#include <stdint.h>

int64_t esp_timer_get_time(void);
void    sim_clock_advance_ms(int64_t ms);
