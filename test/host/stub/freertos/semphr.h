#pragma once
#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;
#define pdTRUE  1
#define pdFALSE 0

SemaphoreHandle_t xSemaphoreCreateMutex(void);
int  xSemaphoreTake(SemaphoreHandle_t s, TickType_t t);
void xSemaphoreGive(SemaphoreHandle_t s);
