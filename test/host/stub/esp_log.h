#pragma once
#include <stdio.h>

extern int g_log_quiet;

#define ESP_LOG_LINE(lvl, tag, fmt, ...) \
    do { if (!g_log_quiet) fprintf(stderr, "[%s] %s: " fmt "\n", lvl, tag, ##__VA_ARGS__); } while (0)

#define ESP_LOGI(tag, fmt, ...) ESP_LOG_LINE("I", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) ESP_LOG_LINE("W", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) ESP_LOG_LINE("E", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ESP_LOG_LINE("D", tag, fmt, ##__VA_ARGS__)
