#pragma once
#include <stdbool.h>
#include <stddef.h>

/* Overrides the compiled-in MODEM_APN/USER/PASS defaults with NVS-backed
 * values. Call once, early in app_main() after nvs_config_load(), before the
 * first network_init(). Pass NULL/"" user & pass for an auth-less APN. */
void network_set_apn(const char *apn, const char *user, const char *pass);

/* Builds the Quectel data-context command "AT+QICSGP=1,1,\"apn\",\"user\",
 * \"pass\",<auth>" for the current APN (auth = 3 when a user is set, else 0).
 * Shared by network_init() and fota.c so both use identical, live settings. */
void network_build_qicsgp(char *out, size_t out_size);

bool network_init(void);
bool network_check(void);
