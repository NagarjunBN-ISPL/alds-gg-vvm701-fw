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

/* The same command for any Quectel context id ("AT+QICSGP=<cid>,1,..."). gps.c
 * uses it to give AGPS a context of its own with the live APN settings. */
void network_build_qicsgp_ctx(int cid, char *out, size_t out_size);

/* The APN in force right now (compiled default or the NVS override). */
const char *network_apn(void);

bool network_init(void);

/* Registration only: CEREG/CGREG says the modem is attached to a network.
 * Answers "is the modem alive and attached", NOT "can it open a socket" —
 * see network_data_bearer_up() for why those are different questions. */
bool network_check(void);

/* Registration AND an active PDP context ("+QIACT: 1,1").
 *
 * THIS is the question the connectivity recovery ladder must ask, and asking
 * the other one was a real field failure mode: when the network tears the PDP
 * context down (a "+QIURC: \"pdpdeact\"", which a pulled SIM also produces) the
 * modem stays REGISTERED but can no longer open a socket. Every QMTOPEN then
 * fails in a way that looks exactly like a broker outage, so a ladder gated on
 * registration concludes "not our fault", refuses to escalate, and never
 * reaches the level that runs AT+QIACT=1 — the one thing that would fix it. The
 * gateway stays offline indefinitely while reporting a healthy network.
 *
 * Costs one extra AT round-trip over network_check(), and only on the failure
 * path. Ported from the same fix in cng-cpil-vvm701-fw. */
bool network_data_bearer_up(void);
