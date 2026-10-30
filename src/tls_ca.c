#include "tls_ca.h"
#include "modem.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "tls_ca";

/* ── Trusted root CA bundle ───────────────────────────────────────────────
 * FOURTEEN roots, concatenated into one PEM file (~18 KB): the five below,
 * then (added 2026-10-30, at the end) the other four Amazon Trust Services
 * roots and the five that Azure, DigitalOcean, Oracle and Google storage
 * chain to, so the FOTA host can move off AWS without a firmware update.
 * The TLS endpoints this firmware talks to are issued by different CAs,
 * verified by directly inspecting each one's live TLS handshake (all
 * re-checked 2026-09-28; S3 again 2026-10-30):
 *
 *   PROD MQTT broker (mqtt.iotronsys.com:8777, SSL ctx 2)
 *   DEV  MQTT broker (dev-fuelsense.iotronsys.com:8777, SSL ctx 2)
 *     leaf -> Let's Encrypt YE1 -> ISRG Root YE [-> ISRG Root X2 -> X1]
 *   FOTA, GitHub Pages (nagarjunbn-ispl.github.io, SSL ctx 1)
 *     leaf -> Let's Encrypt YR1 -> ISRG Root YR [-> ISRG Root X1]
 *   FOTA, S3 (devfota-fuelsense.s3.ap-south-1.amazonaws.com, SSL ctx 1)
 *     leaf -> Amazon RSA 2048 M01 -> Amazon Root CA 1
 *
 * The [bracketed] links are cross-signs, and BOTH expire 2032-09-02. Until
 * then X1/X2 alone would do; after it, a server sending only leaf + YE1/YR1
 * verifies against nothing but Root YE / Root YR themselves — which is why
 * they are here (appended last, so the three older roots load exactly as
 * before). With the old three-root bundle, both brokers and GitHub Pages fail
 * that short chain; with this one all pass (openssl, 2026-09-28).
 *
 * Shipping only the ISRG root (the first cut of this file) is what broke
 * FOTA with "HTTPS GET failed (err=732 http=-1)" — the S3 certificate was
 * correctly REJECTED because its root wasn't trusted. Both contexts point
 * at this same bundle, so any endpoint can move between these CAs without
 * a firmware change.
 *
 * Concatenated PEM ("CA bundle") is the standard way to supply multiple
 * roots and is what the modem's TLS stack expects; each cert is parsed in
 * turn until the list is exhausted.
 *
 * If any endpoint ever moves to a CA outside this list, its TLS
 * handshake will fail closed (connection refused, not silently accepted) —
 * the correct direction to fail, but it does mean a new root must be added
 * here first. */

/* ISRG Root X1 — trust anchor for Let's Encrypt (the MQTT broker).
 * Fetched 2026-07-17 from https://letsencrypt.org/certs/isrgrootx1.pem
 * (valid until 2035-06-04). */
static const char *k_ca_bundle =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n"
/* ISRG Root X2 — Let's Encrypt's ECDSA hierarchy, which is what the PRODUCTION
 * broker mqtt.iotronsys.com:8777 actually chains to (checked live 2026-09-12):
 *     leaf -> Let's Encrypt YE1 -> ISRG Root YE -> ISRG Root X2
 * The X2 the server sends is CROSS-SIGNED BY X1, so a store holding only X1 can
 * still reach a trust anchor — openssl does, confirmed against this host — but
 * that is a four-hop walk ending in a cross-signature, and the EC200U's TLS
 * stack is not openssl. With X2 present the modem terminates at the second cert
 * the server sends and the cross-signature never has to be resolved. Purely
 * additive: X1 stays for the dev broker's RSA hierarchy and for S3's roots.
 * Fetched from https://letsencrypt.org/certs/isrg-root-x2.pem (self-signed,
 * valid until 2040-09-17); SHA-256 69:72:9B:8E:...:CB:14:70 (same block the
 * field-proven cng-cpil-vvm701-fw ships). The broker offers NO RSA cert — an
 * RSA-only client is refused — so this root is not optional for production. */
"-----BEGIN CERTIFICATE-----\n"
"MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw\n"
"CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg\n"
"R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00\n"
"MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT\n"
"ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw\n"
"EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW\n"
"+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9\n"
"ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T\n"
"AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI\n"
"zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW\n"
"tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1\n"
"/q4AaOeMSQ+2b1tbFfLn\n"
"-----END CERTIFICATE-----\n"
/* Amazon Root CA 1 — trust anchor for AWS S3 (the FOTA host).
 * Fetched 2026-07-17 from https://www.amazontrust.com/repository/AmazonRootCA1.pem
 * (valid until 2038-01-17). */
"-----BEGIN CERTIFICATE-----\n"
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
"rqXRfboQnoZsG4q5WTP468SQvvG5\n"
"-----END CERTIFICATE-----\n"
/* ISRG Root YE — Let's Encrypt's Gen-Y ECDSA P-384 root, self-signed, valid
 * 2025-09-03 .. 2045-09-02. Both brokers now chain leaf -> YE1 -> Root YE, and
 * that reaches ISRG Root X2 above only through a cross-sign that EXPIRES
 * 2032-09-02 — after which, without this root, every gateway would fail its
 * MQTT handshake and could not even be reached by FOTA to fix it.
 * From https://letsencrypt.org/certs/gen-y/root-ye.pem, 2026-09-28; SHA-256
 * E1:4F:FC:AD:5B:00:25:73:10:06:CA:A4:3A:12:1A:22:D8:E9:70:0F:4F:B9:CF:85:2F:02:A7:08:AA:5D:56:66
 * (same fingerprint on crt.sh; same key as the X2-signed Root YE the brokers
 * serve today). */
"-----BEGIN CERTIFICATE-----\n"
"MIIB2TCCAWCgAwIBAgIRAKQCa6LvbHwg1AR+XmWmk4AwCgYIKoZIzj0EAwMwLjEL\n"
"MAkGA1UEBhMCVVMxDTALBgNVBAoTBElTUkcxEDAOBgNVBAMTB1Jvb3QgWUUwHhcN\n"
"MjUwOTAzMDAwMDAwWhcNNDUwOTAyMjM1OTU5WjAuMQswCQYDVQQGEwJVUzENMAsG\n"
"A1UEChMESVNSRzEQMA4GA1UEAxMHUm9vdCBZRTB2MBAGByqGSM49AgEGBSuBBAAi\n"
"A2IABDwS/6vhrcVqcbBo+wgdI3fwn9x7DNJJOY/lTOti0vkwuRN87RhEhTH17E7X\n"
"yFjWsPYhIPt/wzOqxTd2b+4ZJNy9ID04YywF9U5zasDVyGSNErVNtz8uSGh5izW8\n"
"7j77GaNCMEAwDgYDVR0PAQH/BAQDAgEGMA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0O\n"
"BBYEFKPIJlqOoUzQNWP8myPIOq5W809WMAoGCCqGSM49BAMDA2cAMGQCMHhMr8N9\n"
"LdL1VQKs9BdV81r76eXRB6mtjuNjzk6/lBsPNToWLTDzGYgtQKO1jl63uAIwGV7m\n"
"onyF377c+MM1oqVNs17sgu7F9YKZwgLmVbeOMDbKAXHtKMDLbiGllCcs8f47\n"
"-----END CERTIFICATE-----\n"
/* ISRG Root YR — the Gen-Y RSA-4096 root, self-signed, valid 2025-09-03 ..
 * 2045-09-02. The GitHub Pages FOTA host chains leaf -> YR1 -> Root YR, which
 * reaches ISRG Root X1 above only through a cross-sign that also expires
 * 2032-09-02, and an RSA broker certificate would land here too.
 * From https://letsencrypt.org/certs/gen-y/root-yr.pem, 2026-09-28; SHA-256
 * E5:7B:7E:6F:15:0C:41:91:02:E8:D5:C0:55:72:9F:F9:67:B9:D1:A8:29:BF:00:CE:C8:9C:A6:04:EB:F4:A8:6F
 * (same fingerprint on crt.sh). */
"-----BEGIN CERTIFICATE-----\n"
"MIIFKTCCAxGgAwIBAgIRAOxGNJNgz0sP+KmC2Tqpyj0wDQYJKoZIhvcNAQELBQAw\n"
"LjELMAkGA1UEBhMCVVMxDTALBgNVBAoTBElTUkcxEDAOBgNVBAMTB1Jvb3QgWVIw\n"
"HhcNMjUwOTAzMDAwMDAwWhcNNDUwOTAyMjM1OTU5WjAuMQswCQYDVQQGEwJVUzEN\n"
"MAsGA1UEChMESVNSRzEQMA4GA1UEAxMHUm9vdCBZUjCCAiIwDQYJKoZIhvcNAQEB\n"
"BQADggIPADCCAgoCggIBANvGJnN78CTJdWL3+eGfsLN5TrNBJs+VH9hRXqRbwxu9\n"
"sGNiB0BD1fcOxbSUQCJIM1xE13Db+5Cw1w0s0EBYsvuIP/6joF0w8cuImbgR1OGg\n"
"YbSQ4OpzI+DG8SGuTlcE873OCS+kh3srlo6vl43M5OJg4Aeo1sfHp6kTJDoIiFBN\n"
"JAY+OKfX/FUvYKuhjT+no49lmqmupSBI5PkBQiqrEGtWU5uxU/cQWHGu8jSjFBzn\n"
"ZqvbNPLMXMLFxCb3WTfrJBXXjqvWG+v4bjzxjjeAtOlU7qarRDvNOyAuQYLln904\n"
"M+faKx8hnLCpJ15ZqaEgcNlY+9MMWcC5yvL2A2j3l9+2buggZX+dOE91zYmIdawT\n"
"vSZuVvlbRrAlLxIB6pwMBjneXCjYQ8+3BCCjssbSNpZU3hTcBDdhfAlEDlYr6pEa\n"
"tnMdmDT5BqnKC92bd0EhM1fbLHioLccLCuievT8ZkPhZrq7Mii7gNXAcUEAR8+lz\n"
"Yal+9zTg7C5DALyVOeG/CqfRAMn1KSHCR0NSA6P8tn/mGRlnCct5rtVCLnVySVpU\n"
"6H1qGg3DgTOuskf8eahTMiYbI5ezPJmO5ertalskQ1utp74+eDy92PI4ftHKTbq9\n"
"IWhH4YZKh3WnJEIt+oQvlYZbY8tpEroKrFB6PFGzrJIDRyts4HqvuH52RFj2zv/B\n"
"AgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNVHRMBAf8EBTADAQH/MB0GA1Ud\n"
"DgQWBBTe51tg0CJtQCh9Pw0B/qS1UrRRlDANBgkqhkiG9w0BAQsFAAOCAgEAWHnf\n"
"713Bdkq7t5yN2dNIgQakUb94X9WuyhMEHHkgx4oDpSUlnG0w4g94MoqaEUE31ZjR\n"
"LU7L5LD1g9ujFHTQu8AD215AHMVQFbm6j8hQxdXHAzDajFNQnOlDJrLjzIx176oy\n"
"AjvUtejZx2NNmdb5fd0WGVGsCdoAJ3N8ozo7ajE8t6vfxStZb4BQ9WYJGHUDrv2N\n"
"i5tJF6CNiPnlzs3BUfECRbE4JSk+jvy8+VoGiFE8qsH/j78x2fjgQhAQFV7P7Zxy\n"
"dBTZ1wEkNpZNW2qnaK1SKBLa+xf6E06YRIq5uaI+HWH8SY1y5VbRgzq40EKg3yxP\n"
"06fz+uYAUIFJoLNfhwRCc3Q6pQVuMX3yAjHAes4gk4moGcLQ5p7HAh39yeylZc1J\n"
"41sx/jKwLIkPE6Rr1Nf4pxdsxf9SA4yOEiAkDgq04DVxn8hgYFdUtBCuiuVC2heA\n"
"EiqVEa+8QZjuw8Gj0EbHXcRd1nInvGqRS1o9Is7YBdQN57X1AYveGBNNqjICSb7c\n"
"awuw1EawTDrs13VUlJVEsbQ0/O/1aaV73mCdOQ8azqL2KTv1Ewu1xbquE2S+kdQU\n"
"To9TUwat3wUA6cwXh1EfpS/3fJ0aGah5hdpRyoCLDlsSn8tkrjMfFFX0viC+GxHc\n"
"sI1ANRYvqSFC2X1VRZfDg+wD6E21BccmifG4yWc=\n"
"-----END CERTIFICATE-----\n"
/* The rest of the Amazon Trust Services roots, which AWS asks clients to
 * trust as a set. S3 is issued under Amazon Root CA 1 today (above); these
 * keep FOTA downloads verifying if S3 moves to another Amazon root or serves
 * an ECDSA certificate, and Starfield G2 is the root CA 1 is cross-signed by.
 * Appended last, so every root above loads exactly as before. Fetched
 * 2026-10-30 from https://www.amazontrust.com/repository/ and checked
 * against Mozilla's root store (same SHA-256 in both). */
/* Amazon Root CA 2 — RSA 4096, valid until 2040-05-26; SHA-256
 * 1B:A5:B2:AA:8C:65:40:1A:82:96:01:18:F8:0B:EC:4F:62:30:4D:83:CE:C4:71:3A:19:C3:9C:01:1E:A4:6D:B4 */
"-----BEGIN CERTIFICATE-----\n"
"MIIFQTCCAymgAwIBAgITBmyf0pY1hp8KD+WGePhbJruKNzANBgkqhkiG9w0BAQwF\n"
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
"b24gUm9vdCBDQSAyMB4XDTE1MDUyNjAwMDAwMFoXDTQwMDUyNjAwMDAwMFowOTEL\n"
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
"b3QgQ0EgMjCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK2Wny2cSkxK\n"
"gXlRmeyKy2tgURO8TW0G/LAIjd0ZEGrHJgw12MBvIITplLGbhQPDW9tK6Mj4kHbZ\n"
"W0/jTOgGNk3Mmqw9DJArktQGGWCsN0R5hYGCrVo34A3MnaZMUnbqQ523BNFQ9lXg\n"
"1dKmSYXpN+nKfq5clU1Imj+uIFptiJXZNLhSGkOQsL9sBbm2eLfq0OQ6PBJTYv9K\n"
"8nu+NQWpEjTj82R0Yiw9AElaKP4yRLuH3WUnAnE72kr3H9rN9yFVkE8P7K6C4Z9r\n"
"2UXTu/Bfh+08LDmG2j/e7HJV63mjrdvdfLC6HM783k81ds8P+HgfajZRRidhW+me\n"
"z/CiVX18JYpvL7TFz4QuK/0NURBs+18bvBt+xa47mAExkv8LV/SasrlX6avvDXbR\n"
"8O70zoan4G7ptGmh32n2M8ZpLpcTnqWHsFcQgTfJU7O7f/aS0ZzQGPSSbtqDT6Zj\n"
"mUyl+17vIWR6IF9sZIUVyzfpYgwLKhbcAS4y2j5L9Z469hdAlO+ekQiG+r5jqFoz\n"
"7Mt0Q5X5bGlSNscpb/xVA1wf+5+9R+vnSUeVC06JIglJ4PVhHvG/LopyboBZ/1c6\n"
"+XUyo05f7O0oYtlNc/LMgRdg7c3r3NunysV+Ar3yVAhU/bQtCSwXVEqY0VThUWcI\n"
"0u1ufm8/0i2BWSlmy5A5lREedCf+3euvAgMBAAGjQjBAMA8GA1UdEwEB/wQFMAMB\n"
"Af8wDgYDVR0PAQH/BAQDAgGGMB0GA1UdDgQWBBSwDPBMMPQFWAJI/TPlUq9LhONm\n"
"UjANBgkqhkiG9w0BAQwFAAOCAgEAqqiAjw54o+Ci1M3m9Zh6O+oAA7CXDpO8Wqj2\n"
"LIxyh6mx/H9z/WNxeKWHWc8w4Q0QshNabYL1auaAn6AFC2jkR2vHat+2/XcycuUY\n"
"+gn0oJMsXdKMdYV2ZZAMA3m3MSNjrXiDCYZohMr/+c8mmpJ5581LxedhpxfL86kS\n"
"k5Nrp+gvU5LEYFiwzAJRGFuFjWJZY7attN6a+yb3ACfAXVU3dJnJUH/jWS5E4ywl\n"
"7uxMMne0nxrpS10gxdr9HIcWxkPo1LsmmkVwXqkLN1PiRnsn/eBG8om3zEK2yygm\n"
"btmlyTrIQRNg91CMFa6ybRoVGld45pIq2WWQgj9sAq+uEjonljYE1x2igGOpm/Hl\n"
"urR8FLBOybEfdF849lHqm/osohHUqS0nGkWxr7JOcQ3AWEbWaQbLU8uz/mtBzUF+\n"
"fUwPfHJ5elnNXkoOrJupmHN5fLT0zLm4BwyydFy4x2+IoZCn9Kr5v2c69BoVYh63\n"
"n749sSmvZ6ES8lgQGVMDMBu4Gon2nL2XA46jCfMdiyHxtN/kHNGfZQIG6lzWE7OE\n"
"76KlXIx3KadowGuuQNKotOrN8I1LOJwZmhsoVLiJkO/KdYE+HvJkJMcYr07/R54H\n"
"9jVlpNMKVv/1F2Rs76giJUmTtt8AF9pYfl3uxRuw0dFfIRDH+fO6AgonB8Xx1sfT\n"
"4PsJYGw=\n"
"-----END CERTIFICATE-----\n"
/* Amazon Root CA 3 — ECDSA P-256, valid until 2040-05-26; SHA-256
 * 18:CE:6C:FE:7B:F1:4E:60:B2:E3:47:B8:DF:E8:68:CB:31:D0:2E:BB:3A:DA:27:15:69:F5:03:43:B4:6D:B3:A4 */
"-----BEGIN CERTIFICATE-----\n"
"MIIBtjCCAVugAwIBAgITBmyf1XSXNmY/Owua2eiedgPySjAKBggqhkjOPQQDAjA5\n"
"MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6b24g\n"
"Um9vdCBDQSAzMB4XDTE1MDUyNjAwMDAwMFoXDTQwMDUyNjAwMDAwMFowOTELMAkG\n"
"A1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJvb3Qg\n"
"Q0EgMzBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABCmXp8ZBf8ANm+gBG1bG8lKl\n"
"ui2yEujSLtf6ycXYqm0fc4E7O5hrOXwzpcVOho6AF2hiRVd9RFgdszflZwjrZt6j\n"
"QjBAMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMB0GA1UdDgQWBBSr\n"
"ttvXBp43rDCGB5Fwx5zEGbF4wDAKBggqhkjOPQQDAgNJADBGAiEA4IWSoxe3jfkr\n"
"BqWTrBqYaGFy+uGh0PsceGCmQ5nFuMQCIQCcAu/xlJyzlvnrxir4tiz+OpAUFteM\n"
"YyRIHN8wfdVoOw==\n"
"-----END CERTIFICATE-----\n"
/* Amazon Root CA 4 — ECDSA P-384, valid until 2040-05-26; SHA-256
 * E3:5D:28:41:9E:D0:20:25:CF:A6:90:38:CD:62:39:62:45:8D:A5:C6:95:FB:DE:A3:C2:2B:0B:FB:25:89:70:92 */
"-----BEGIN CERTIFICATE-----\n"
"MIIB8jCCAXigAwIBAgITBmyf18G7EEwpQ+Vxe3ssyBrBDjAKBggqhkjOPQQDAzA5\n"
"MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6b24g\n"
"Um9vdCBDQSA0MB4XDTE1MDUyNjAwMDAwMFoXDTQwMDUyNjAwMDAwMFowOTELMAkG\n"
"A1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJvb3Qg\n"
"Q0EgNDB2MBAGByqGSM49AgEGBSuBBAAiA2IABNKrijdPo1MN/sGKe0uoe0ZLY7Bi\n"
"9i0b2whxIdIA6GO9mif78DluXeo9pcmBqqNbIJhFXRbb/egQbeOc4OO9X4Ri83Bk\n"
"M6DLJC9wuoihKqB1+IGuYgbEgds5bimwHvouXKNCMEAwDwYDVR0TAQH/BAUwAwEB\n"
"/zAOBgNVHQ8BAf8EBAMCAYYwHQYDVR0OBBYEFNPsxzplbszh2naaVvuc84ZtV+WB\n"
"MAoGCCqGSM49BAMDA2gAMGUCMDqLIfG9fhGt0O9Yli/W651+kI0rz2ZVwyzjKKlw\n"
"CkcO8DdZEv8tmZQoTipPNU0zWgIxAOp1AE47xDqUEpHJWEadIRNyp4iciuRMStuW\n"
"1KyLa2tJElMzrdfkviT8tQp21KW8EA==\n"
"-----END CERTIFICATE-----\n"
/* Starfield Services Root Certificate Authority - G2 — RSA 2048, valid until 2037-12-31; SHA-256
 * 56:8D:69:05:A2:C8:87:08:A4:B3:02:51:90:ED:CF:ED:B1:97:4A:60:6A:13:C6:E5:29:0F:CB:2A:E6:3E:DA:B5 */
"-----BEGIN CERTIFICATE-----\n"
"MIID7zCCAtegAwIBAgIBADANBgkqhkiG9w0BAQsFADCBmDELMAkGA1UEBhMCVVMx\n"
"EDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxJTAjBgNVBAoT\n"
"HFN0YXJmaWVsZCBUZWNobm9sb2dpZXMsIEluYy4xOzA5BgNVBAMTMlN0YXJmaWVs\n"
"ZCBTZXJ2aWNlcyBSb290IENlcnRpZmljYXRlIEF1dGhvcml0eSAtIEcyMB4XDTA5\n"
"MDkwMTAwMDAwMFoXDTM3MTIzMTIzNTk1OVowgZgxCzAJBgNVBAYTAlVTMRAwDgYD\n"
"VQQIEwdBcml6b25hMRMwEQYDVQQHEwpTY290dHNkYWxlMSUwIwYDVQQKExxTdGFy\n"
"ZmllbGQgVGVjaG5vbG9naWVzLCBJbmMuMTswOQYDVQQDEzJTdGFyZmllbGQgU2Vy\n"
"dmljZXMgUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgLSBHMjCCASIwDQYJKoZI\n"
"hvcNAQEBBQADggEPADCCAQoCggEBANUMOsQq+U7i9b4Zl1+OiFOxHz/Lz58gE20p\n"
"OsgPfTz3a3Y4Y9k2YKibXlwAgLIvWX/2h/klQ4bnaRtSmpDhcePYLQ1Ob/bISdm2\n"
"8xpWriu2dBTrz/sm4xq6HZYuajtYlIlHVv8loJNwU4PahHQUw2eeBGg6345AWh1K\n"
"Ts9DkTvnVtYAcMtS7nt9rjrnvDH5RfbCYM8TWQIrgMw0R9+53pBlbQLPLJGmpufe\n"
"hRhJfGZOozptqbXuNC66DQO4M99H67FrjSXZm86B0UVGMpZwh94CDklDhbZsc7tk\n"
"6mFBrMnUVN+HL8cisibMn1lUaJ/8viovxFUcdUBgF4UCVTmLfwUCAwEAAaNCMEAw\n"
"DwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwHQYDVR0OBBYEFJxfAN+q\n"
"AdcwKziIorhtSpzyEZGDMA0GCSqGSIb3DQEBCwUAA4IBAQBLNqaEd2ndOxmfZyMI\n"
"bw5hyf2E3F/YNoHN2BtBLZ9g3ccaaNnRbobhiCPPE95Dz+I0swSdHynVv/heyNXB\n"
"ve6SbzJ08pGCL72CQnqtKrcgfU28elUSwhXqvfdqlS5sdJ/PHLTyxQGjhdByPq1z\n"
"qwubdQxtRbeOlKyWN7Wg0I8VRw7j6IPdj/3vQQF3zCepYoUz8jcI73HPdwbeyBkd\n"
"iEDPfUYd/x7H4c7/I9vG+o1VTqkC50cRRj70/b17KSa7qWFiNyi2LSr2EIZkyXCn\n"
"0q23KXB56jzaYyWf/Wi3MOxw+3WKt21gZ7IeyLnp2KhvAotnDU0mV3HaIPzBSlCN\n"
"sSi6\n"
"-----END CERTIFICATE-----\n"
/* Roots of the other storage clouds, so FOTA files can move off AWS without
 * a firmware update first (live chains checked 2026-10-30): Azure Blob is
 * issued under Microsoft TLS RSA Root G2, sent cross-signed by DigiCert
 * Global Root G2; DigitalOcean Spaces and Oracle Object Storage chain
 * straight to DigiCert G2; Google Cloud Storage to GTS Root R4 (RSA-only
 * clients: R1). DigiCert G2 and GTS R1/R4 match Mozilla's root store; the two
 * Microsoft TLS roots are from Microsoft's own repository
 * (www.microsoft.com/pkiops/certs), and the RSA one carries the key Azure
 * serves today. Appended last, so every root above loads exactly as before. */
/* DigiCert Global Root G2 — RSA 2048, valid until 2038-01-15; SHA-256
 * CB:3C:CB:B7:60:31:E5:E0:13:8F:8D:D3:9A:23:F9:DE:47:FF:C3:5E:43:C1:14:4C:EA:27:D4:6A:5A:B1:CB:5F */
"-----BEGIN CERTIFICATE-----\n"
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n"
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n"
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n"
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n"
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n"
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n"
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n"
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n"
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n"
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n"
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n"
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n"
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n"
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n"
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n"
"MrY=\n"
"-----END CERTIFICATE-----\n"
/* Microsoft TLS RSA Root G2 — RSA 4096, valid until 2040-04-10; SHA-256
 * 6A:17:05:83:DB:58:41:51:E1:C4:54:EE:CA:2A:64:CC:5D:8E:48:4A:5B:D1:15:6E:72:0B:44:58:65:4E:E9:E5 */
"-----BEGIN CERTIFICATE-----\n"
"MIIFgDCCA2igAwIBAgIQZIbjsmkYD79AQDkuLlNLmzANBgkqhkiG9w0BAQwFADBR\n"
"MQswCQYDVQQGEwJVUzEeMBwGA1UEChMVTWljcm9zb2Z0IENvcnBvcmF0aW9uMSIw\n"
"IAYDVQQDExlNaWNyb3NvZnQgVExTIFJTQSBSb290IEcyMB4XDTI1MDQxMDE4MzYz\n"
"MFoXDTQwMDQxMDE4NDM1MVowUTELMAkGA1UEBhMCVVMxHjAcBgNVBAoTFU1pY3Jv\n"
"c29mdCBDb3Jwb3JhdGlvbjEiMCAGA1UEAxMZTWljcm9zb2Z0IFRMUyBSU0EgUm9v\n"
"dCBHMjCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAN/qi59H4SgQca9B\n"
"1hnbklfea6vmLt6tOmCDtxOYrFu5Mgt7659yJ2tdQKu+GAaTsg3SEu24dPV/L+qo\n"
"F5T2MxZXit4gfCHdt9qs3k5ksFrNS3Xz7wJkTe1S2qOPBOiR+Nf7Pxn9TkU5Xugp\n"
"w9byZ6+zAJTBjeHwShiYOfrACYQzQQUDBav0deFTpSGSXERM5mBXpJACl8fV2JME\n"
"MkrthVHt1sW6oD82ii9vTDmoBGkihDt1FPfZPrSp0AdBOIF+Rkgd6BnmcLigqhjd\n"
"cr9+fbVsThlwEYQEE4HUYv2ca/Ec3f/sp+Dmx40xS7/yazPM0Yy2/rEbo0qoXw5/\n"
"MXY4PwdHbAGorKUU2oKwXx9svIkawBIqy3uXHyguMRaKU0x1Zvl9AdbCYKpljzGe\n"
"csvMmy3lRBmTV6tujsLZLmWwci3nocOhfkkdWWfWCdQYEm3AlUV4yfFgauT27gYk\n"
"O3NKWhHCPpdhwHju7xOnMOJKOC3uWBcTILc2Kt8LMrTCOLGR3RKwRbZ8wLEdYqGd\n"
"3XESFZYBnQoJ9bHds9ngOEI09mm2w68Wc/7UQ/hD5Fj5T7Pz7NY9lDpAOBWrDIbQ\n"
"ZM4aY+iElFUJWrYEttE+mhXHZBGA0BuCDk3/RZ6yZQclH9uayMbs1luFUASfHMtd\n"
"IIJnnnni3+jy6G/EIHIwhuWddSc1AgMBAAGjVDBSMA4GA1UdDwEB/wQEAwIBhjAP\n"
"BgNVHRMBAf8EBTADAQH/MB0GA1UdDgQWBBTekYZIt6ExWTHxS18HqdyIedqodjAQ\n"
"BgkrBgEEAYI3FQEEAwIBADANBgkqhkiG9w0BAQwFAAOCAgEAdDOaUHmugVYlz5tv\n"
"URHeVRKQk2wz4C4kDF5kYjRONhm1oBTztcZ406PtsDsKXpI+chsuCoPev5+o0zmo\n"
"DtgjNQcFj7bzxhrUQyQXk5obQPB53cnAD3MaBYeLCrrbJIi0hL3N6n8b7XalNAe6\n"
"bg7qczpIwu3xnX/3jgUT28vTfShczLK+oHkQKibrcuSmoZcedsQ8flaa8+Y9z+LV\n"
"BBlqPNc45eO7HgUP3UgrfDmd6U9Nd6i9SSif57koWzb0cyCCVhbVwVpNxO37B8xo\n"
"N67DdFOrlygAA0aE0do5nk8QSj+ZQis1qDmVt36BPmyv/byprsCSTMI0JTsXp5uL\n"
"QAoj8orWuAA/Ul5Wu3nri4n4JIngzeYO+s8ISGbNypagtcsvb0QT5sY4nvEEkPET\n"
"SLVIQij9/4FIYlFVlJ9nSV9sSP7H0e2RoLQbttBruxNgoKifkv0zVcyBTzPjmkej\n"
"xTOZnDVvfB+kS189DzljD/otUI6sGZT9v5fZjljMgGlAZzi2xhfrHKqvhFZSn2Al\n"
"MnuQZvLjlzEiIsG1Oj2JFOG0kGq1MmKwljhVBvuKryqDTHrqGa3E9JQattAXUoet\n"
"+QzlJ5xIPcay1LE+MA+TM6wC5aCgUmmg8sVILr9GwyZF5urX+Usp4ta44Em3upSv\n"
"eM+HJyMso/L+NgtC+eHXJLur5sg=\n"
"-----END CERTIFICATE-----\n"
/* Microsoft TLS ECC Root G2 — ECDSA P-384, valid until 2040-04-10; SHA-256
 * 87:75:5C:FE:88:BD:B0:D1:09:9D:CD:ED:3E:AE:11:4B:A9:76:E6:64:B3:24:8E:E3:DC:64:9E:35:7F:17:E8:A7 */
"-----BEGIN CERTIFICATE-----\n"
"MIICMTCCAbegAwIBAgIQcuICK8WywbBNJQVuYuJ2eTAKBggqhkjOPQQDAzBRMQsw\n"
"CQYDVQQGEwJVUzEeMBwGA1UEChMVTWljcm9zb2Z0IENvcnBvcmF0aW9uMSIwIAYD\n"
"VQQDExlNaWNyb3NvZnQgVExTIEVDQyBSb290IEcyMB4XDTI1MDQxMDIwNTIzNloX\n"
"DTQwMDQxMDIwNTg1OVowUTELMAkGA1UEBhMCVVMxHjAcBgNVBAoTFU1pY3Jvc29m\n"
"dCBDb3Jwb3JhdGlvbjEiMCAGA1UEAxMZTWljcm9zb2Z0IFRMUyBFQ0MgUm9vdCBH\n"
"MjB2MBAGByqGSM49AgEGBSuBBAAiA2IABFHCOIT2ay617TcWSuXj100AhDtAVPwJ\n"
"5vTFp6ZrVkROrWCeFZiOA+7iss11PVhkwBYM31rcLJ1AW75LVCrOjzy/lbyOrInv\n"
"aT4XBvd9wbgCZSPL9R0a41Oh9Ad+CotdWaNUMFIwDgYDVR0PAQH/BAQDAgGGMA8G\n"
"A1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFG+rftr/l0Ny7Dtnd96CYTWIR0KFMBAG\n"
"CSsGAQQBgjcVAQQDAgEAMAoGCCqGSM49BAMDA2gAMGUCMQDYM1rR4cySss0f8PKM\n"
"sT2ytNTKmNzaiQX0G1dx35RNInYFUElNkCmSLL8vN+W5deICMAGSaVidU7LIVWZb\n"
"uslQWiqBto7PEdDyfoTcfzbfwy3aTfgisNizizSfnTEcrA8RGQ==\n"
"-----END CERTIFICATE-----\n"
/* GTS Root R1 — RSA 4096, valid until 2036-06-22; SHA-256
 * D9:47:43:2A:BD:E7:B7:FA:90:FC:2E:6B:59:10:1B:12:80:E0:E1:C7:E4:E4:0F:A3:C6:88:7F:FF:57:A7:F4:CF */
"-----BEGIN CERTIFICATE-----\n"
"MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw\n"
"CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU\n"
"MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw\n"
"MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp\n"
"Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA\n"
"A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo\n"
"27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w\n"
"Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw\n"
"TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl\n"
"qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH\n"
"szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8\n"
"Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk\n"
"MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92\n"
"wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p\n"
"aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN\n"
"VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID\n"
"AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E\n"
"FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb\n"
"C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe\n"
"QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy\n"
"h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4\n"
"7HLwEXWdyzRSjeZ2axfG34arJ45JK3VmgRAhpuo+9K4l/3wV3s6MJT/KYnAK9y8J\n"
"ZgfIPxz88NtFMN9iiMG1D53Dn0reWVlHxYciNuaCp+0KueIHoI17eko8cdLiA6Ef\n"
"MgfdG+RCzgwARWGAtQsgWSl4vflVy2PFPEz0tv/bal8xa5meLMFrUKTX5hgUvYU/\n"
"Z6tGn6D/Qqc6f1zLXbBwHSs09dR2CQzreExZBfMzQsNhFRAbd03OIozUhfJFfbdT\n"
"6u9AWpQKXCBfTkBdYiJ23//OYb2MI3jSNwLgjt7RETeJ9r/tSQdirpLsQBqvFAnZ\n"
"0E6yove+7u7Y/9waLd64NnHi/Hm3lCXRSHNboTXns5lndcEZOitHTtNCjv0xyBZm\n"
"2tIMPNuzjsmhDYAPexZ3FL//2wmUspO8IFgV6dtxQ/PeEMMA3KgqlbbC1j+Qa3bb\n"
"bP6MvPJwNQzcmRk13NfIRmPVNnGuV/u3gm3c\n"
"-----END CERTIFICATE-----\n"
/* GTS Root R4 — ECDSA P-384, valid until 2036-06-22; SHA-256
 * 34:9D:FA:40:58:C5:E2:63:12:3B:39:8A:E7:95:57:3C:4E:13:13:C8:3F:E6:8F:93:55:6C:D5:E8:03:1B:3C:7D */
"-----BEGIN CERTIFICATE-----\n"
"MIICCTCCAY6gAwIBAgINAgPlwGjvYxqccpBQUjAKBggqhkjOPQQDAzBHMQswCQYD\n"
"VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEUMBIG\n"
"A1UEAxMLR1RTIFJvb3QgUjQwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAwMDAw\n"
"WjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2Vz\n"
"IExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjQwdjAQBgcqhkjOPQIBBgUrgQQAIgNi\n"
"AATzdHOnaItgrkO4NcWBMHtLSZ37wWHO5t5GvWvVYRg1rkDdc/eJkTBa6zzuhXyi\n"
"QHY7qca4R9gq55KRanPpsXI5nymfopjTX15YhmUPoYRlBtHci8nHc8iMai/lxKvR\n"
"HYqjQjBAMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW\n"
"BBSATNbrdP9JNqPV2Py1PsVq8JQdjDAKBggqhkjOPQQDAwNpADBmAjEA6ED/g94D\n"
"9J+uHXqnLrmvT/aDHQ4thQEd0dlq7A/Cr8deVl5c1RxYIigL9zC2L7F8AjEA8GE8\n"
"p/SgguMh1YQdc4acLa/KNJvxn7kjNuK8YAOdgLOaVsjh4rsUecrNIdSUtUlD\n"
"-----END CERTIFICATE-----\n";

bool tls_ca_provision(void) {
    size_t len = strlen(k_ca_bundle);
    if (!modem_upload_file(TLS_CA_FILE, k_ca_bundle, len)) {
        ESP_LOGE(TAG, "CA cert upload failed — TLS validation cannot be enabled this boot");
        return false;
    }

    /* MQTT (SSL context 2) and FOTA (SSL context 1) are configured
     * independently on this modem — each needs its own cacert reference
     * even though both point at the same uploaded file. */
    char cmd[64];
    bool ok = true;

    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"cacert\",2,\"%s\"", TLS_CA_FILE);
    ok &= modem_at(cmd, "OK", 3000, NULL, 0);
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"cacert\",1,\"%s\"", TLS_CA_FILE);
    ok &= modem_at(cmd, "OK", 3000, NULL, 0);

    if (!ok)
        ESP_LOGW(TAG, "cacert reference rejected on one or both SSL contexts — check the log above for which");
    else
        ESP_LOGI(TAG, "CA cert provisioned for MQTT (ctx 2) and FOTA (ctx 1)");
    return ok;
}

/* A FOTA that power-cycles the modem re-uploads the bundle in the middle of
 * the update (fota.c), and an upload cut short there leaves the modem with no
 * CA, or half of one: every TLS handshake fails, MQTT included, and only the
 * next boot used to upload it again — a gateway that could no longer be
 * reached to be told anything. So after a failed update the stored size is
 * checked first, and the bundle goes up again only when it is missing or
 * wrong: the upload deletes before it writes, so re-uploading an intact file
 * would risk leaving none at all for no gain (as alds-sr's ca_ensure). */
bool tls_ca_ensure(void) {
    const int want = (int)strlen(k_ca_bundle);
    char cmd[64], resp[160];
    snprintf(cmd, sizeof(cmd), "AT+QFLST=\"%s\"", TLS_CA_FILE);
    if (modem_at(cmd, "+QFLST:", 5000, resp, sizeof(resp))) {
        const char *p     = strstr(resp, "+QFLST:");             /* +QFLST: "<file>",<size> */
        const char *comma = p ? strchr(p, ',') : NULL;
        if (comma && atoi(comma + 1) == want) return true;
    }
    ESP_LOGW(TAG, "CA file missing or not %d bytes in the modem — uploading it again", want);
    for (int i = 0; i < 2; i++) {
        if (tls_ca_provision()) return true;
    }
    ESP_LOGE(TAG, "CA file could not be restored — TLS connects fail until the next boot uploads it");
    return false;
}
