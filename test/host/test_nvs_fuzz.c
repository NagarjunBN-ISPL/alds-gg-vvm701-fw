#include <stddef.h>
/* Runs the REAL src/nvs_config.c against corrupt, foreign, truncated and
 * oversized NVS content, under ASan + UBSan. The question being answered is
 * narrow and important: can anything in flash make the config loader read out
 * of bounds, divide by zero, or hand the rest of the firmware a value that
 * indexes past an array?
 *
 * The stub allocates each blob to its EXACT stored length, so a read one byte
 * past what NVS holds is a hard ASan failure rather than something that happens
 * to land in adjacent memory. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

#include "config.h"
#include "nvs_config.h"
#include "nvs.h"

int g_log_quiet = 1;

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        if (fails < 20) printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

/* xorshift — deterministic, so a failure is reproducible from the seed. */
static uint32_t rng_s = 0x12345678;
static uint32_t rnd(void) {
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 17; rng_s ^= rng_s << 5;
    return rng_s;
}

static bool str_terminated(const char *p, size_t n) {
    for (size_t i = 0; i < n; i++) if (p[i] == '\0') return true;
    return false;
}

/* Suffixes are two DIGITS, 01..99, or empty — every one is also an address. */
static bool sfx_ok(const char *s) {
    if (s[0] == '\0') return true;
    return strlen(s) == 2 && isdigit((unsigned char)s[0]) && isdigit((unsigned char)s[1]) &&
           !(s[0] == '0' && s[1] == '0');
}

/* Everything the rest of the firmware assumes about a loaded config. Each of
 * these, if violated, is a concrete crash or corruption downstream:
 * the counts index g_hw_* arrays, genset_model indexes the model table,
 * genset_baud divides the UART clock, and the strings are printed with %s. */
static void assert_invariants(const DeviceConfig *c, const char *ctx) {
    CHECK(c->du_count <= DU_MAX_DISPENSERS, "%s: du_count=%u indexes past g_hw_du", ctx, c->du_count);
    for (int i = 0; i < DU_MAX_DISPENSERS; i++)
        CHECK(c->du_make[i] <= DU_MAKE_MAX, "%s: du_make[%d]=%u outside the enum", ctx, i, c->du_make[i]);
    /* THE dispenser-plan contract du_task relies on: every ON slot (suffix
     * set) names a dispenser the bus can address (BlueSky 1..6, Yenen 1..5),
     * no two slots name the same one, and du_count is exactly the slots ON. A
     * violation here is a nozzle polled at address 0/13+, one pump published
     * under two HW_IDs, or "configured" disagreeing with what is polled. */
    {
        bool used[DU_MAX_DISPENSERS + 1] = {0};
        int  on = 0;
        for (int i = 0; i < DU_MAX_DISPENSERS; i++) {
            if (!du_slot_on(c, i)) continue;
            on++;
            int n = du_number_of(c, i);
            int max_n = (c->du_make[i] == DU_MAKE_YENEN) ? DU_YENEN_MAX : DU_MAX_DISPENSERS;
            CHECK(n >= 1 && n <= max_n, "%s: DU slot %d -> dispenser %d (sfx '%s', make %u) unaddressable",
                  ctx, i + 1, n, c->du_sfx[i], c->du_make[i]);
            if (n >= 1 && n <= DU_MAX_DISPENSERS) {
                CHECK(!used[n], "%s: DU slot %d names dispenser %d twice", ctx, i + 1, n);
                used[n] = true;
            }
        }
        CHECK(c->du_count == on, "%s: du_count %u but %d slots ON", ctx, c->du_count, on);
        CHECK(c->du_plan == DU_PLAN_SLOTS, "%s: du_plan %u after load (must be slots)", ctx, c->du_plan);
    }
    CHECK(c->ztech_count      <= RADAR_MAX,      "%s: ztech=%u", ctx, c->ztech_count);
    CHECK(c->pantech_count    <= RADAR_MAX,      "%s: pantech=%u", ctx, c->pantech_count);
    CHECK(c->reserved_u8 == 0,                   "%s: reserved_u8=%u", ctx, c->reserved_u8);
    CHECK(c->genset_type  <= GENSET_SEDEMAC,     "%s: genset_type=%u", ctx, c->genset_type);
    CHECK(c->genset_model <= GENSET_MODEL_MAX,   "%s: genset_model=%u indexes past the model table", ctx, c->genset_model);
    CHECK(c->genset_slave >= 1 && c->genset_slave <= 247, "%s: genset_slave=%u", ctx, c->genset_slave);
    CHECK(c->genset_baud == 4800 || c->genset_baud == 9600 || c->genset_baud == 19200,
          "%s: genset_baud=%lu would divide the UART clock by junk", ctx, (unsigned long)c->genset_baud);
    CHECK(c->di_enable <= 0x0F, "%s: di_enable=0x%02x", ctx, c->di_enable);

    /* Radar slaves are per-tank (suffix = address) since the GGY scheme; the
     * old fixed-range collision rule no longer applies. Uniqueness of the
     * resolved addresses is enforced by validate_bus_addresses() in main.c on
     * every setconfig, not by the loader. */

    /* Strings: terminated inside their own arrays, or every later %s/strlen
     * walks into the next field. */
    CHECK(str_terminated(c->apn, sizeof(c->apn)), "%s: apn unterminated", ctx);
    CHECK(str_terminated(c->apn_user, sizeof(c->apn_user)), "%s: apn_user unterminated", ctx);
    CHECK(str_terminated(c->apn_pass, sizeof(c->apn_pass)), "%s: apn_pass unterminated", ctx);
    CHECK(str_terminated(c->mqtt_host, sizeof(c->mqtt_host)), "%s: mqtt_host unterminated", ctx);
    CHECK(str_terminated(c->mqtt_user, sizeof(c->mqtt_user)), "%s: mqtt_user unterminated", ctx);
    CHECK(str_terminated(c->mqtt_pass, sizeof(c->mqtt_pass)), "%s: mqtt_pass unterminated", ctx);
    CHECK(str_terminated(c->mqtt_base_topic, sizeof(c->mqtt_base_topic)), "%s: base topic unterminated", ctx);
    CHECK(str_terminated(c->mqtt_host_bak, sizeof(c->mqtt_host_bak)), "%s: host_bak unterminated", ctx);
    CHECK(str_terminated(c->mqtt_pass_bak, sizeof(c->mqtt_pass_bak)), "%s: pass_bak unterminated", ctx);
    CHECK(str_terminated(c->mqtt_base_topic_bak, sizeof(c->mqtt_base_topic_bak)), "%s: base_bak unterminated", ctx);

    /* Connectivity must be USABLE, not merely safe — a blank host or a zero
     * port would leave the gateway unable to reach anything at all. */
    CHECK(c->mqtt_host[0] != '\0', "%s: empty broker host", ctx);
    CHECK(c->mqtt_port != 0, "%s: broker port 0", ctx);
    CHECK(c->mqtt_base_topic[0] != '\0', "%s: empty base topic", ctx);
    CHECK(c->apn[0] != '\0', "%s: empty APN", ctx);

    /* Bools must hold exactly 0 or 1. Anything else is undefined to read, so
     * check the raw byte rather than the bool itself. */
    {
        const unsigned char *p = (const unsigned char *)c;
        size_t off[] = {
            offsetof(DeviceConfig, gps_enable), offsetof(DeviceConfig, mqtt_tls),
            offsetof(DeviceConfig, mqtt_tls_bak), offsetof(DeviceConfig, conn_pending),
            offsetof(DeviceConfig, conn_rollback_notify),
        };
        for (unsigned i = 0; i < sizeof(off)/sizeof(off[0]); i++)
            CHECK(p[off[i]] <= 1, "%s: bool at offset %zu holds %u — UB to read",
                  ctx, off[i], p[off[i]]);
    }

    /* Suffixes reach JSON payloads verbatim. */
    CHECK(sfx_ok(c->sfx_genset), "%s: bad sfx_genset", ctx);
    CHECK(sfx_ok(c->sfx_di), "%s: bad sfx_di", ctx);
    for (int i = 0; i < DU_MAX_DISPENSERS; i++) CHECK(sfx_ok(c->du_sfx[i]), "%s: bad du_sfx[%d]", ctx, i);
    for (int i = 0; i < RADAR_MAX; i++) {
        CHECK(sfx_ok(c->sfx_ztech[i]), "%s: bad sfx_ztech[%d]", ctx, i);
        CHECK(sfx_ok(c->sfx_pantech[i]), "%s: bad sfx_pantech[%d]", ctx, i);
    }
}

int main(int argc, char **argv) {
    int iters = (argc > 1) ? atoi(argv[1]) : 60000;
    printf("== NVS fault injection against the real nvs_config.c (%d cases) ==\n", iters);

    DeviceConfig c;

    /* ── named edge cases ─────────────────────────────────────────────────── */
    printf("[1] structural edge cases\n");
    {
        /* no NVS at all */
        simnvs_reset(); simnvs_no_blob();
        nvs_config_load(&c); assert_invariants(&c, "empty NVS");

        /* zero-length blob */
        simnvs_reset(); simnvs_set_blob_raw("", 0); simnvs_set_tag(PRODUCT_ID);
        nvs_config_load(&c); assert_invariants(&c, "zero-length blob");

        /* one byte */
        simnvs_reset(); simnvs_set_blob_raw("\xff", 1); simnvs_set_tag(PRODUCT_ID);
        nvs_config_load(&c); assert_invariants(&c, "1-byte blob");

        /* all-0xFF blob of exact size — erased flash */
        {
            unsigned char *b = malloc(sizeof(DeviceConfig));
            memset(b, 0xFF, sizeof(DeviceConfig));
            simnvs_reset(); simnvs_set_blob_raw(b, sizeof(DeviceConfig)); simnvs_set_tag(PRODUCT_ID);
            nvs_config_load(&c); assert_invariants(&c, "all-0xFF blob");
            free(b);
        }
        /* all-zero blob */
        {
            unsigned char *b = calloc(1, sizeof(DeviceConfig));
            simnvs_reset(); simnvs_set_blob_raw(b, sizeof(DeviceConfig)); simnvs_set_tag(PRODUCT_ID);
            nvs_config_load(&c); assert_invariants(&c, "all-zero blob");
            free(b);
        }
        /* blob LARGER than the struct — a future build's config on old firmware */
        {
            size_t n = sizeof(DeviceConfig) * 2;
            unsigned char *b = malloc(n);
            for (size_t i = 0; i < n; i++) b[i] = (unsigned char)rnd();
            simnvs_reset(); simnvs_set_blob_raw(b, n); simnvs_set_tag(PRODUCT_ID);
            nvs_config_load(&c); assert_invariants(&c, "oversized blob");
            free(b);
        }
        /* NVS open fails outright */
        simnvs_reset(); simnvs_open_fails(1);
        nvs_config_load(&c); assert_invariants(&c, "nvs_open fails");
        simnvs_open_fails(0);

        /* every write fails (worn-out / full flash) */
        simnvs_reset(); simnvs_set_tag("some-other-product"); simnvs_writes_fail(1);
        nvs_config_load(&c); assert_invariants(&c, "all writes fail");
        simnvs_writes_fail(0);

        /* foreign product tag with a full random blob behind it */
        {
            unsigned char *b = malloc(sizeof(DeviceConfig));
            for (size_t i = 0; i < sizeof(DeviceConfig); i++) b[i] = (unsigned char)rnd();
            simnvs_reset(); simnvs_set_blob_raw(b, sizeof(DeviceConfig));
            simnvs_set_tag("cng-cpil-vvm701/1");
            nvs_config_load(&c);
            assert_invariants(&c, "foreign product");
            CHECK(c.du_count == 0 &&
                  c.ztech_count == 0 && c.pantech_count == 0 &&
                  c.genset_type == GENSET_OFF && c.di_enable == 0,
                  "foreign product: equipment config survived the wipe");
            free(b);
        }
    }

    /* ── random blobs of random length, with random product tags ──────────── */
    printf("[2] %d random blobs\n", iters);
    unsigned char buf[sizeof(DeviceConfig) * 2];
    for (int it = 0; it < iters; it++) {
        size_t n = rnd() % (sizeof(DeviceConfig) + 8);
        for (size_t i = 0; i < n; i++) buf[i] = (unsigned char)rnd();

        simnvs_reset();
        if (n) simnvs_set_blob_raw(buf, n); else simnvs_no_blob();

        switch (rnd() % 4) {
            case 0: simnvs_set_tag(PRODUCT_ID);          break;
            case 1: simnvs_set_tag(NULL);                break;  /* legacy unit */
            case 2: simnvs_set_tag("cng-cpil-vvm701/1"); break;  /* other product */
            default: {
                char junk[48];
                size_t jn = rnd() % (sizeof(junk) - 1);
                for (size_t i = 0; i < jn; i++) junk[i] = (char)(0x20 + (rnd() % 0x5f));
                junk[jn] = '\0';
                simnvs_set_tag(junk);
                break;
            }
        }
        if ((rnd() % 50) == 0) simnvs_writes_fail(1);

        nvs_config_load(&c);
        assert_invariants(&c, "random blob");

        /* The loader must also be idempotent: re-loading what it just persisted
         * cannot drift further. */
        DeviceConfig c2;
        nvs_config_load(&c2);
        assert_invariants(&c2, "reload");

        /* And credential enforcement on top of whatever came out. */
        bool dest = false;
        nvs_config_enforce_firmware_creds(&c2, &dest);
        assert_invariants(&c2, "post-enforce");
        CHECK(c2.cred_fp == nvs_config_compiled_cred_fp(), "fp not stamped after enforce");
        simnvs_writes_fail(0);
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
