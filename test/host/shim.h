/* Minimal environment for the functions extracted from main.c. */
#pragma once
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "config.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TOPIC_MAX 200
static const char *TAG __attribute__((unused)) = "main";

extern DeviceConfig g_cfg;
extern char g_hw_du[DU_MAX_DISPENSERS][24];

bool rtc_timestamp(char *buf, size_t size);
bool mqtt_publish(const char *topic, const char *payload);
