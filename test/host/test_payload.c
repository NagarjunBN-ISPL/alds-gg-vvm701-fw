/* The BlueSky nozzle payload, built by the REAL du_build_nozzle_payload()
 * (extracted from src/dispenser.c) from the bytes a field gateway actually
 * returned, compared against the record the cloud stores for that gateway.
 *
 * The reference is a Sentry record from IMEI 860710081332556, nozzle 1, taken
 * 2026-09-21. The one deliberate difference from the field device is that this
 * firmware does not read RFID: the key is present, its value is the constant
 * "404". Everything else — key names, hex casing, separators, the full-frame
 * header/trailer, HW_ID and TS placement — must match exactly. */
#include "shim.h"
#include "dispenser.h"
#include "extracted_payload.inc"

int g_log_quiet = 1;

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static int hex_to_bytes(const char *s, uint8_t *out) {
    int n = 0;
    while (*s) {
        unsigned v; if (sscanf(s, "%2x", &v) != 1) break;
        out[n++] = (uint8_t)v; s += 2; if (*s == ',') s++;
    }
    return n;
}

int main(void) {
    printf("== BlueSky nozzle payload vs the field (Sentry) record ==\n");
    DuNozzle_t nz; memset(&nz, 0, sizeof(nz));
    nz.present = true; nz.idle = true;
    nz.total_len = hex_to_bytes("f5,01,ac,00,02,15,18,26,23,30,19,35,34,c5,3f", nz.total);
    nz.rsp_len   = hex_to_bytes("f5,01,a4,68,86,b6,08", nz.rsp);
    nz.kfact_len = hex_to_bytes("f5,01,a4,23,60,e2,71", nz.kfact);
    CHECK(nz.total_len == 15 && nz.rsp_len == 7 && nz.kfact_len == 7, "test vector decode");

    char buf[DU_PAYLOAD_MAX];
    du_build_nozzle_payload(&nz, 1, "86071008133255601", "2026-09-21 12:55:06", buf, sizeof(buf));
    printf("       %s\n", buf);

    const char *want =
        "{\"HW_ID\":\"86071008133255601\",\"nozzles\":{\"1\":{"
        "\"data\":\"f5,01,ac,00,02,15,18,26,23,30,19,35,34,c5,3f\","
        "\"rsp\":\"f5,01,a4,68,86,b6,08\","
        "\"kfact\":\"f5,01,a4,23,60,e2,71\","
        "\"rfid\":\"404\"}},"
        "\"TS\":\"2026-09-21 12:55:06\"}";
    CHECK(strcmp(buf, want) == 0, "payload differs from the field shape:\n     got  %s\n     want %s", buf, want);

    /* Every key the field record carries is present, and no extra one. */
    CHECK(strstr(buf, "\"data\":")  && strstr(buf, "\"rsp\":") &&
          strstr(buf, "\"kfact\":") && strstr(buf, "\"rfid\":"), "a field key is missing");
    CHECK(strstr(buf, "\"rfid\":\"404\""), "rfid must be the constant 404 (not polled)");

    /* Nozzle 2 of the same record, with a failed rsp read: that field alone is 404. */
    memset(&nz, 0, sizeof(nz)); nz.present = true; nz.idle = true;
    nz.total_len = hex_to_bytes("f5,02,ac,04,30,83,81,09,40,83,66,07,33,c5,30", nz.total);
    nz.kfact_len = hex_to_bytes("f5,02,a4,23,65,e2,77", nz.kfact);
    du_build_nozzle_payload(&nz, 2, "86071008133255601", "2026-09-21 12:55:06", buf, sizeof(buf));
    CHECK(strstr(buf, "\"2\":{\"data\":\"f5,02,ac,04,30,83,81,09,40,83,66,07,33,c5,30\",\"rsp\":\"404\",\"kfact\":\"f5,02,a4,23,65,e2,77\",\"rfid\":\"404\"}"),
          "per-field 404 shape wrong: %s", buf);

    /* Absent or busy: the compact 404 shape, no other keys. */
    memset(&nz, 0, sizeof(nz));
    du_build_nozzle_payload(&nz, 3, "86071008133255602", "2026-09-21 12:55:06", buf, sizeof(buf));
    CHECK(strcmp(buf, "{\"HW_ID\":\"86071008133255602\",\"nozzles\":{\"3\":{\"data\":\"404\"}},\"TS\":\"2026-09-21 12:55:06\"}") == 0,
          "absent shape wrong: %s", buf);

    /* Worst case stays far under the modem ceiling. */
    memset(&nz, 0, sizeof(nz)); nz.present = true; nz.idle = true;
    nz.total_len = nz.rsp_len = nz.kfact_len = DU_RESP_MAX;
    memset(nz.total, 0xAB, DU_RESP_MAX); memset(nz.rsp, 0xCD, DU_RESP_MAX); memset(nz.kfact, 0xEF, DU_RESP_MAX);
    du_build_nozzle_payload(&nz, 12, "860710081332556", "2026-09-21 12:55:06", buf, sizeof(buf));
    CHECK(strlen(buf) < MQTT_MODEM_PAYLOAD_SAFE_MAX && strstr(buf, "\"rfid\":\"404\""),
          "worst case %zu bytes", strlen(buf));
    printf("       worst-case nozzle payload %zu bytes\n", strlen(buf));

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
