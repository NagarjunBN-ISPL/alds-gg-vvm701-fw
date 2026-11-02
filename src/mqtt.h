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

/* Registered once at boot; called exactly once inside mqtt_publish() for
 * every publish attempt (while connected), so the caller can flash an
 * activity LED on each push. NULL = no-op. */
void mqtt_set_activity_cb(void (*cb)(void));

bool mqtt_init(const char *imei);
bool mqtt_connect(void);
bool mqtt_subscribe(const char *topic);
bool mqtt_publish(const char *topic, const char *payload);
bool mqtt_poll(char *topic_out, size_t topic_size,
               char *payload_out, size_t payload_size);
/* When the message mqtt_poll() last returned was taken off the UART
 * (esp_timer µs) — its arrival, which can be well before it is handled. */
int64_t mqtt_last_rx_us(void);
void mqtt_disconnect(void);
bool mqtt_is_connected(void);

/* Seconds since the current broker session was established, or 0 if the link
 * is down or has never come up this boot. Published as link.connected_s — the
 * field that separates a device up for days from one reconnecting constantly,
 * which are otherwise indistinguishable from a single diagnostics report. */
uint32_t mqtt_connected_s(void);
