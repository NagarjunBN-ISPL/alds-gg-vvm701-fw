#pragma once
#include <stdbool.h>
#include <stddef.h>

bool mqtt_init(const char *imei);
bool mqtt_connect(void);
bool mqtt_subscribe(const char *topic);
bool mqtt_publish(const char *topic, const char *payload);
bool mqtt_poll(char *topic_out, size_t topic_size,
               char *payload_out, size_t payload_size);
void mqtt_disconnect(void);
bool mqtt_is_connected(void);
