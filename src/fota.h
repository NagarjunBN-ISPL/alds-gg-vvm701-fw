#pragma once
#include <stdbool.h>
#include <stddef.h>

/* Longest URL fota_perform() can handle — it copies into a buffer of exactly
 * this size, so anything longer is silently truncated and would download from
 * a URL nobody asked for. Exposed so the caller can REJECT an over-long URL up
 * front (with a reason) rather than discover the truncation as a failed
 * download. 1535 since 2026-10-30 (was 511): an S3 link presigned with an
 * IAM-user key is ~360 characters, one signed with a role key (it carries
 * X-Amz-Security-Token) ~1.1-1.4 KB, other clouds' signed links up to ~800.
 * The whole update message must still fit mqtt.c's 2 KB PAYLOAD_MAX, and
 * 1535 leaves ~500 bytes of it for the other keys. */
#define FOTA_URL_MAX 1535

/* Plausible image sizes, from cng-cpil's fota.h (field-proven there). A reply
 * outside them is refused before anything is saved: an error page served with
 * a 200 is a few hundred bytes and would otherwise be flashed as firmware.
 * Today's images are ~450 KB. */
#define FOTA_MIN_IMAGE_BYTES  65536
#define FOTA_MAX_IMAGE_BYTES  (3 * 1024 * 1024)

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
 *   allow_handover = the push said "handover":true. Without it an image of
 *               ANOTHER VVM701 product is refused ("not_this_product") after
 *               it is verified and before the boot slot is switched, so a
 *               wrong file cannot turn a station into a different product.
 * Returns true only if the OTA image was written and the boot partition set
 * (the caller then reboots; the new image reports fota_successful once it has
 * confirmed itself). */
bool fota_perform(const char *url, const char *pub_topic,
                  char *reason, size_t reason_size, bool allow_handover);
