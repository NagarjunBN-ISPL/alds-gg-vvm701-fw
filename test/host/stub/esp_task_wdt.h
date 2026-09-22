#pragma once

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NOT_FOUND 1

/* Counted so the test can prove the driver feeds the watchdog while it waits. */
extern int g_wdt_feeds;

esp_err_t esp_task_wdt_status(void *h);
esp_err_t esp_task_wdt_reset(void);
