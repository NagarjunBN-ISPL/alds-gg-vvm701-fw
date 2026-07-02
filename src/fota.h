#pragma once
#include <stdbool.h>

/* Download firmware via HTTPS to EC200U UFS, write to OTA partition, reboot.
   pub_topic = full MQTT topic to publish fota_start / fota_success on.
   Returns true only if OTA written successfully (reboot happens inside). */
bool fota_perform(const char *url, const char *pub_topic);
