#pragma once
#include <stdbool.h>

/* Filename on the modem's own UFS filesystem where the trusted root CA
 * bundle is stored, once uploaded. Both mqtt.c (SSL context 2) and fota.c
 * (SSL context 1) reference this same file via AT+QSSLCFG="cacert",<ctx>,...
 * — one upload, two contexts, both roots available to each. */
#define TLS_CA_FILE "UFS:ca_root.pem"

/* Uploads the embedded root CA bundle (Let's Encrypt's ISRG Root X1 for the
 * MQTT broker + Amazon Root CA 1 for the S3 FOTA host — see tls_ca.c) to the
 * modem's UFS and points both SSL contexts at it. Call once at boot, after
 * the modem is confirmed up and before the first TLS connection attempt —
 * mqtt_connect()/fota_download() both assume TLS_CA_FILE already exists by
 * the time they run.
 *
 * Re-uploads unconditionally on every boot rather than trying to detect
 * whether the file is already present and correct: it's a ~3KB write, and
 * "always write the known-good bytes" is simpler and harder to get wrong
 * than a freshness check that could itself be wrong. modem_upload_file()
 * deletes before writing, since QFUPL refuses to overwrite (the file DOES
 * survive ESP resets — the modem stays powered across them). */
bool tls_ca_provision(void);
