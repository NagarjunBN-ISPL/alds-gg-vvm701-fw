#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Overrides the compiled-in MQTT_HOST/PORT/USER/PASS defaults with
 * NVS-backed values. Call once, early in app_main() after nvs_config_load(),
 * before the first mqtt_connect(). tls=false connects in plaintext (broker
 * credentials and telemetry then travel unencrypted). */
void mqtt_set_credentials(const char *host, uint16_t port,
                           const char *user, const char *pass, bool tls);

/* Current broker password, for modem.c's AT-command log redaction. Not for
 * any other use — it's the live, possibly-runtime-overridden secret. */
const char *mqtt_get_pass(void);

bool mqtt_init(const char *imei);
bool mqtt_connect(void);
bool mqtt_subscribe(const char *topic);
bool mqtt_publish(const char *topic, const char *payload);
bool mqtt_poll(char *topic_out, size_t topic_size,
               char *payload_out, size_t payload_size);
void mqtt_disconnect(void);
bool mqtt_is_connected(void);
