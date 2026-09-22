#pragma once
#include <stdbool.h>
#include <stddef.h>

/* Longest URL fota_perform() can handle — it copies into a buffer of exactly
 * this size, so anything longer is silently truncated and would download from
 * a URL nobody asked for. Exposed so the caller can REJECT an over-long URL up
 * front (with a reason) rather than discover the truncation as a failed
 * download. Real URLs are well under 200 bytes. */
#define FOTA_URL_MAX 511

/* True if the image fota_perform() just staged belongs to ANOTHER product
 * (cross-product handover — see image_is_foreign in fota.c). Valid only after
 * fota_perform() returned true, until the restart that follows. The caller
 * must NOT spill the RAM queue to flash in that case: it would hand the other
 * firmware a backlog of this station's readings to republish as its own. */
bool fota_last_image_foreign(void);

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
