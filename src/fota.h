#pragma once
#include <stdbool.h>
#include <stddef.h>

/* Download firmware via HTTPS to EC200U UFS, write to OTA partition.
 *   pub_topic = full MQTT topic (used for logging context only; MQTT is torn
 *               down during the download, so this function never publishes).
 *   reason    = on failure, a short machine-readable cause token is written
 *               here (e.g. "http_404_err_0", "image_verify_failed") for the
 *               caller to include in its fota_failed message. Untouched on
 *               success. Pass NULL to ignore.
 * Returns true only if the OTA image was written and the boot partition set
 * (the caller then publishes fota_successful and reboots). */
bool fota_perform(const char *url, const char *pub_topic,
                  char *reason, size_t reason_size);
