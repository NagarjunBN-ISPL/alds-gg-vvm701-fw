/* Exercises the credential-enforcement logic EXTRACTED from src/nvs_config.c. */
#include "shim.h"
#include "extracted_nvs.inc"

int g_log_quiet = 1;

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

/* A gateway that has been living at "client A" for a year. */
static void make_client_a(DeviceConfig *c) {
    memset(c, 0, sizeof(*c));
    snprintf(c->mqtt_host,       sizeof(c->mqtt_host),       "mqtt.clienta.example");
    c->mqtt_port = 1883;
    snprintf(c->mqtt_user,       sizeof(c->mqtt_user),       "clienta");
    snprintf(c->mqtt_pass,       sizeof(c->mqtt_pass),       "aSecret");
    snprintf(c->mqtt_base_topic, sizeof(c->mqtt_base_topic), "clienta");
    c->mqtt_tls = false;
    snprintf(c->apn,             sizeof(c->apn),             "CLIENTA.APN");
    snprintf(c->apn_user,        sizeof(c->apn_user),        "au");
    snprintf(c->apn_pass,        sizeof(c->apn_pass),        "ap");
    /* its _bak baseline and an in-flight trial */
    snprintf(c->mqtt_host_bak,   sizeof(c->mqtt_host_bak),   "old.clienta.example");
    c->mqtt_port_bak = 8883;
    c->conn_pending  = true;
    /* the site's physical equipment — must survive untouched */
    c->du_count    = 5;
    c->du_make[3]  = DU_MAKE_YENEN;
    c->du_make[4]  = DU_MAKE_YENEN;
    c->pantech_count    = 1;
    snprintf(c->sfx_pantech[0], sizeof(c->sfx_pantech[0]), "07");
    c->ztech_count      = 2;
    c->genset_type      = GENSET_SEDEMAC;
    c->genset_model     = GENSET_MODEL_SGC120;
    c->genset_slave     = 11;
    c->genset_baud      = 19200;
    c->di_enable        = 0x0F;
    c->gps_enable       = true;
    snprintf(c->du_sfx[0],  sizeof(c->du_sfx[0]),  "01");
    snprintf(c->du_sfx[4],  sizeof(c->du_sfx[4]),  "05");
    snprintf(c->sfx_genset, sizeof(c->sfx_genset), "11");
    c->cred_fp = 0xDEADBEEF;   /* stamped by client A's build */
}

static bool equipment_intact(const DeviceConfig *c) {
    return c->du_count == 5 && c->du_make[3] == DU_MAKE_YENEN && c->du_make[4] == DU_MAKE_YENEN &&
           c->pantech_count == 1 && strcmp(c->sfx_pantech[0], "07") == 0 &&
           c->ztech_count == 2 && c->genset_type == GENSET_SEDEMAC &&
           c->genset_model == GENSET_MODEL_SGC120 && c->genset_slave == 11 &&
           c->genset_baud == 19200 && c->di_enable == 0x0F && c->gps_enable &&
           strcmp(c->du_sfx[0], "01") == 0 &&
           strcmp(c->du_sfx[4], "05") == 0 &&
           strcmp(c->sfx_genset, "11") == 0;
}

static bool is_compiled_creds(const DeviceConfig *c) {
    return strcmp(c->mqtt_host, MQTT_HOST) == 0 &&
           c->mqtt_port == MQTT_PORT &&
           strcmp(c->mqtt_user, MQTT_USER) == 0 &&
           strcmp(c->mqtt_pass, MQTT_PASS) == 0 &&
           strcmp(c->mqtt_base_topic, MQTT_TOPIC_BASE) == 0 &&
           c->mqtt_tls == MQTT_TLS_DEFAULT &&
           strcmp(c->apn, MODEM_APN) == 0 &&
           strcmp(c->apn_user, MODEM_APN_USER) == 0 &&
           strcmp(c->apn_pass, MODEM_APN_PASS) == 0;
}

int main(void) {
    printf("== firmware-credential enforcement (extracted from nvs_config.c) ==\n");
    uint32_t fp = nvs_config_compiled_cred_fp();

    /* 1. THE DISASTER CASE: client A's gateway reflashed with client B's build. */
    printf("[1] gateway carrying another deployment's config\n");
    {
        DeviceConfig c; make_client_a(&c);
        bool dest = false;
        bool forced = nvs_config_enforce_firmware_creds(&c, &dest);
        CHECK(forced, "credentials were NOT taken over — this is the leak case");
        CHECK(dest, "destination change not reported — the queue would not be purged");
        CHECK(is_compiled_creds(&c), "credentials are not the firmware's");
        CHECK(strcmp(c.mqtt_host, "mqtt.clienta.example") != 0, "still pointing at client A");
        CHECK(c.cred_fp == fp, "fingerprint not stamped");
        CHECK(equipment_intact(&c), "equipment configuration was disturbed");
        /* the rollback baseline must not still point at client A */
        CHECK(strcmp(c.mqtt_host_bak, MQTT_HOST) == 0,
              "_bak still holds %s — a failed trial would roll back to the OLD client",
              c.mqtt_host_bak);
        CHECK(c.mqtt_port_bak == MQTT_PORT, "_bak port stale");
        CHECK(c.conn_pending == false, "an old deployment's trial is still armed");
    }

    /* 2. Same build again — NVS must be left completely alone. */
    printf("[2] reboot / reflash of the same build\n");
    {
        DeviceConfig c; make_client_a(&c);
        nvs_config_enforce_firmware_creds(&c, NULL);   /* first adoption */
        DeviceConfig before = c;
        bool dest = true;
        bool forced = nvs_config_enforce_firmware_creds(&c, &dest);
        CHECK(!forced, "re-applied on an unchanged build — needless flash write every boot");
        CHECK(!dest, "reported a destination change when nothing changed");
        CHECK(memcmp(&before, &c, sizeof(c)) == 0, "config mutated on a no-op call");
    }

    /* 3. THE setconfig REQUIREMENT: a runtime credential push must survive
     *    reboots of the same firmware. */
    printf("[3] setconfig-pushed credentials persist\n");
    {
        DeviceConfig c; make_client_a(&c);
        nvs_config_enforce_firmware_creds(&c, NULL);
        /* operator now pushes a new broker via setconfig */
        snprintf(c.mqtt_host, sizeof(c.mqtt_host), "new.broker.example");
        c.mqtt_port = 9001;
        snprintf(c.mqtt_pass, sizeof(c.mqtt_pass), "rotated-password");
        snprintf(c.apn, sizeof(c.apn), "NEW.APN");
        /* ... and the device reboots, twice */
        for (int boot = 0; boot < 2; boot++) {
            bool forced = nvs_config_enforce_firmware_creds(&c, NULL);
            CHECK(!forced, "boot %d: firmware overwrote a setconfig push", boot);
        }
        CHECK(strcmp(c.mqtt_host, "new.broker.example") == 0,
              "setconfig host was reverted to %s", c.mqtt_host);
        CHECK(c.mqtt_port == 9001, "setconfig port reverted");
        CHECK(strcmp(c.mqtt_pass, "rotated-password") == 0, "rotated password reverted");
        CHECK(strcmp(c.apn, "NEW.APN") == 0, "setconfig APN reverted");
    }

    /* 4. Never-stamped NVS (every already-deployed gateway) adopts once. */
    printf("[4] gateway provisioned before this feature existed\n");
    {
        DeviceConfig c; make_client_a(&c);
        c.cred_fp = 0;                       /* short blob -> field defaults to 0 */
        CHECK(nvs_config_enforce_firmware_creds(&c, NULL), "did not adopt on fp=0");
        CHECK(c.cred_fp == fp, "did not stamp");
        CHECK(nvs_config_enforce_firmware_creds(&c, NULL) == false, "adopted twice");
    }

    /* 5. APN-only change must NOT purge the queue (same destination). */
    printf("[5] APN-only change keeps the backlog\n");
    {
        DeviceConfig c; memset(&c, 0, sizeof(c));
        snprintf(c.mqtt_host,       sizeof(c.mqtt_host),       MQTT_HOST);
        c.mqtt_port = MQTT_PORT;
        snprintf(c.mqtt_base_topic, sizeof(c.mqtt_base_topic), MQTT_TOPIC_BASE);
        snprintf(c.apn,             sizeof(c.apn),             "SOME.OTHER.APN");
        c.cred_fp = 0x1234;
        bool dest = true;
        CHECK(nvs_config_enforce_firmware_creds(&c, &dest), "did not apply");
        CHECK(!dest, "APN-only change reported as a destination change — "
                     "the offline queue would be destroyed for nothing");
    }

    /* 6. A base-topic-only change IS a destination change. */
    printf("[6] base-topic change purges\n");
    {
        DeviceConfig c; memset(&c, 0, sizeof(c));
        snprintf(c.mqtt_host,       sizeof(c.mqtt_host),       MQTT_HOST);
        c.mqtt_port = MQTT_PORT;
        snprintf(c.mqtt_base_topic, sizeof(c.mqtt_base_topic), "someoneelse");
        c.cred_fp = 0x1234;
        bool dest = false;
        nvs_config_enforce_firmware_creds(&c, &dest);
        CHECK(dest, "a different topic tree is a different destination");
    }

    /* 7. Fingerprint properties: never 0, and sensitive to every field. */
    printf("[7] fingerprint sensitivity\n");
    {
        CHECK(fp != 0, "fingerprint is 0 — collides with the never-stamped sentinel");

        /* Recompute with one field altered at a time, in the same order. */
        const char *host = MQTT_HOST, *user = MQTT_USER, *pass = MQTT_PASS;
        const char *base = MQTT_TOPIC_BASE, *apn = MODEM_APN;
        const char *au = MODEM_APN_USER, *ap = MODEM_APN_PASS;
        uint16_t port = MQTT_PORT;
        uint8_t  tls  = MQTT_TLS_DEFAULT ? 1u : 0u;

        struct { const char *what; const char *h,*u,*p,*b,*a,*au,*ap; uint16_t port; uint8_t tls; } v[] = {
            {"host",       "other.host", user, pass, base, apn, au, ap, port, tls},
            {"port",       host, user, pass, base, apn, au, ap, (uint16_t)(port+1), tls},
            {"user",       host, "other", pass, base, apn, au, ap, port, tls},
            {"pass",       host, user, "other", base, apn, au, ap, port, tls},
            {"base topic", host, user, pass, "other", apn, au, ap, port, tls},
            {"apn",        host, user, pass, base, "OTHER", au, ap, port, tls},
            {"apn user",   host, user, pass, base, apn, "x", ap, port, tls},
            {"apn pass",   host, user, pass, base, apn, au, "x", port, tls},
            {"tls",        host, user, pass, base, apn, au, ap, port, (uint8_t)!tls},
        };
        for (unsigned i = 0; i < sizeof(v)/sizeof(v[0]); i++) {
            uint32_t h = 2166136261u;
            h = fnv1a_str(h, v[i].h);
            h = fnv1a(h, &v[i].port, sizeof(v[i].port));
            h = fnv1a_str(h, v[i].u);
            h = fnv1a_str(h, v[i].p);
            h = fnv1a_str(h, v[i].b);
            h = fnv1a_str(h, v[i].a);
            h = fnv1a_str(h, v[i].au);
            h = fnv1a_str(h, v[i].ap);
            h = fnv1a(h, &v[i].tls, sizeof(v[i].tls));
            if (!h) h = 1u;
            CHECK(h != fp, "changing the %s does not change the fingerprint", v[i].what);
        }
        /* And the unmodified recomputation must reproduce it exactly. */
        uint32_t h = 2166136261u;
        h = fnv1a_str(h, host); h = fnv1a(h, &port, sizeof(port));
        h = fnv1a_str(h, user); h = fnv1a_str(h, pass); h = fnv1a_str(h, base);
        h = fnv1a_str(h, apn);  h = fnv1a_str(h, au);   h = fnv1a_str(h, ap);
        h = fnv1a(h, &tls, sizeof(tls));
        if (!h) h = 1u;
        CHECK(h == fp, "fingerprint is not reproducible");
        printf("       this build's credential fingerprint: %08x\n", fp);
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
