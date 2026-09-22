#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_task_wdt.h"   /* esp_err_t / ESP_OK */

typedef struct {
    const char *base_path;
    const char *partition_label;
    bool        format_if_mount_failed;
    bool        dont_mount;
} esp_vfs_littlefs_conf_t;

esp_err_t esp_vfs_littlefs_register(const esp_vfs_littlefs_conf_t *conf);
esp_err_t esp_littlefs_info(const char *label, size_t *total, size_t *used);
const char *esp_err_to_name(esp_err_t e);

/* fault injection */
void simfs_mount_fails(int fail);
void simfs_set_capacity(size_t bytes);   /* 0 = unlimited */
