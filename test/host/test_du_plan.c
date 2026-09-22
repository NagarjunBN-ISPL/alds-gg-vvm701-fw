/* The dispenser-plan rules of alds-gg: THE SUFFIX IS THE DISPENSER NUMBER IS
 * THE BUS ADDRESS. Three things are pinned here against the production code:
 *
 *   1. du_number_of()        (src/nvs_config.c, compiled whole) — the ONE
 *                            place a suffix becomes a number;
 *   2. nvs_config_load()     the way-out guard: a stored plan with an
 *                            unaddressable or duplicated entry is CUT, not
 *                            patched, and the surviving prefix is intact;
 *   3. validate_suffixes() + validate_bus_addresses() (EXTRACTED verbatim
 *                            from src/main.c) — the way-in guard: what a
 *                            setconfig may and may not say.
 *
 * Every refusal reason is asserted by the token the cloud will see. */
#include "shim.h"
#include <ctype.h>
#include "nvs.h"
#include "extracted_plan.inc"

int g_log_quiet = 1;
DeviceConfig g_cfg;
char g_hw_du[DU_MAX_DISPENSERS][24];

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static void plan(DeviceConfig *c, int count, const char *const *sfx, const uint8_t *make) {
    memset(c, 0, sizeof(*c));
    c->du_count = (uint8_t)count;
    for (int i = 0; i < count; i++) {
        if (sfx && sfx[i]) snprintf(c->du_sfx[i], sizeof(c->du_sfx[i]), "%s", sfx[i]);
        if (make) c->du_make[i] = make[i];
    }
    c->genset_slave = GENSET_SLAVE_DEFAULT;
    c->genset_baud  = 9600;
}

/* ── 1. du_number_of ───────────────────────────────────────────────────── */
static void t_number(void) {
    printf("[1] du_number_of: suffix else position\n");
    DeviceConfig c;
    const char *s1[] = {"", "", "", "", "", ""};
    plan(&c, 6, s1, NULL);
    for (int i = 0; i < 6; i++)
        CHECK(du_number_of(&c, i) == i + 1, "no suffix: entry %d should be dispenser %d", i, i + 1);

    const char *s2[] = {"03", "01", "06", "05", "", "02"};
    plan(&c, 6, s2, NULL);
    int want[] = {3, 1, 6, 5, 5, 2};   /* entry 5 falls back to position 5 */
    for (int i = 0; i < 6; i++)
        CHECK(du_number_of(&c, i) == want[i], "suffix %s at entry %d -> %d (want %d)",
              s2[i], i, du_number_of(&c, i), want[i]);

    /* garbage that never survives the loader but may reach the validator */
    const char *s3[] = {"A1", "7", "1a", "0x", "00", " 1"};
    plan(&c, 6, s3, NULL);
    for (int i = 0; i < 6; i++)
        CHECK(du_number_of(&c, i) == 0, "'%s' must resolve to 0 (got %d)", s3[i], du_number_of(&c, i));

    CHECK(du_number_of(NULL, 0) == 0 && du_number_of(&c, -1) == 0 && du_number_of(&c, DU_MAX_DISPENSERS) == 0,
          "out-of-range arguments must resolve to 0");
}

/* ── 2. the loader cuts a bad plan ─────────────────────────────────────── */
static void save_and_load(const DeviceConfig *in, DeviceConfig *out) {
    simnvs_reset();
    simnvs_set_tag(PRODUCT_ID);
    simnvs_set_blob_raw(in, sizeof(*in));
    nvs_config_load(out);
}

static void t_loader(void) {
    printf("[2] nvs_config_load cuts an unaddressable or duplicated plan\n");
    DeviceConfig in, out;

    /* a good plan with gaps survives intact */
    { const char *s[] = {"01", "03", "05"}; plan(&in, 3, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 3 && du_number_of(&out, 1) == 3 && du_number_of(&out, 2) == 5,
          "gapped plan 01/03/05 must load intact (got count %u)", out.du_count);

    /* a lettered suffix is cleared to the position — and that may collide */
    { const char *s[] = {"02", "A1"}; plan(&in, 2, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_sfx[1][0] == '\0', "lettered suffix must be cleared");
    /* cleared to position 2, which collides with entry 1 = 02, so the plan is cut */
    CHECK(out.du_count == 1, "duplicate after clearing must cut the plan to 1 (got %u)", out.du_count);

    /* out of range: 07 */
    { const char *s[] = {"01", "07", "02"}; plan(&in, 3, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 1, "07 is unaddressable: plan must be cut to 1 (got %u)", out.du_count);

    /* Yenen at 06 */
    { const char *s[] = {"01", "06"}; uint8_t m[] = {DU_MAKE_BLUESKY, DU_MAKE_YENEN}; plan(&in, 2, s, m); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 1, "Yenen 06 has no nozzle code: plan must be cut to 1 (got %u)", out.du_count);

    /* BlueSky at 06 is fine */
    { const char *s[] = {"01", "06"}; plan(&in, 2, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 2, "BlueSky 06 is addressable (got %u)", out.du_count);

    /* duplicate explicit suffixes */
    { const char *s[] = {"02", "02"}; plan(&in, 2, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 1, "02/02 must be cut to 1 (got %u)", out.du_count);

    /* the cut never touches entries before the bad one */
    { const char *s[] = {"04", "02", "04"}; plan(&in, 3, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 2 && du_number_of(&out, 0) == 4 && du_number_of(&out, 1) == 2,
          "prefix before the duplicate must survive (count %u)", out.du_count);
}

/* ── 3. the validator refuses what the loader would cut ────────────────── */
static bool ok_plan(int count, const char *const *sfx, const uint8_t *make, char *err, size_t n) {
    DeviceConfig c;
    plan(&c, count, sfx, make);
    err[0] = '\0';
    return validate_suffixes(&c, err, n) && validate_bus_addresses(&c, err, n);
}

static void t_validator(void) {
    printf("[3] validate_suffixes / validate_bus_addresses\n");
    char err[160];

    { const char *s[] = {"", "", "", "", "", ""};
      CHECK(ok_plan(6, s, NULL, err, sizeof(err)), "six unsuffixed dispensers: %s", err); }
    { const char *s[] = {"01", "03", "05"};
      CHECK(ok_plan(3, s, NULL, err, sizeof(err)), "gapped 01/03/05: %s", err); }
    { const char *s[] = {"06"};
      CHECK(ok_plan(1, s, NULL, err, sizeof(err)), "BlueSky 06: %s", err); }
    { const char *s[] = {"05"}; uint8_t m[] = {DU_MAKE_YENEN};
      CHECK(ok_plan(1, s, m, err, sizeof(err)), "Yenen 05: %s", err); }

    { const char *s[] = {"06"}; uint8_t m[] = {DU_MAKE_YENEN};
      CHECK(!ok_plan(1, s, m, err, sizeof(err)) && strstr(err, "Yenen") && strstr(err, "01..05"),
            "Yenen 06 must be refused naming the range (got '%s')", err); }
    { const char *s[] = {"07"};
      CHECK(!ok_plan(1, s, NULL, err, sizeof(err)) && strstr(err, "01..06"),
            "07 must be refused naming the range (got '%s')", err); }
    { const char *s[] = {"00"};
      CHECK(!ok_plan(1, s, NULL, err, sizeof(err)), "00 must be refused (got '%s')", err); }
    { const char *s[] = {"A1"};
      CHECK(!ok_plan(1, s, NULL, err, sizeof(err)) && strstr(err, "2 digits"),
            "letters must be refused as non-digits (got '%s')", err); }
    { const char *s[] = {"1"};
      CHECK(!ok_plan(1, s, NULL, err, sizeof(err)), "1-char suffix must be refused (got '%s')", err); }
    { const char *s[] = {"02", "02"};
      CHECK(!ok_plan(2, s, NULL, err, sizeof(err)) && strstr(err, "duplicate"),
            "02/02 must be refused as duplicate (got '%s')", err); }
    { const char *s[] = {"02", ""};
      CHECK(!ok_plan(2, s, NULL, err, sizeof(err)) && strstr(err, "duplicate"),
            "entry 2 defaulting to 02 collides with explicit 02 (got '%s')", err); }

    /* cross-unit: dispenser 04 vs a Ztech tank at slave 4 shares the HW_ID */
    {
        DeviceConfig c; const char *s[] = {"", "", "", ""};
        plan(&c, 4, s, NULL); c.ztech_count = 1;
        err[0] = '\0';
        CHECK(!validate_suffixes(&c, err, sizeof(err)) && strstr(err, "DU4") && strstr(err, "Ztech1"),
              "DU4 vs Ztech1 both 04 must be refused by name (got '%s')", err);
        snprintf(c.sfx_ztech[0], sizeof(c.sfx_ztech[0]), "07");
        CHECK(validate_suffixes(&c, err, sizeof(err)) && validate_bus_addresses(&c, err, sizeof(err)),
              "moving the tank to 07 resolves it: %s", err);
    }
    /* radar suffixes are digits too */
    {
        DeviceConfig c; plan(&c, 0, NULL, NULL); c.ztech_count = 1;
        snprintf(c.sfx_ztech[0], sizeof(c.sfx_ztech[0]), "Z1");
        CHECK(!validate_suffixes(&c, err, sizeof(err)) && strstr(err, "2 digits"),
              "lettered radar suffix must be refused (got '%s')", err);
    }
    /* genset on a radar's slave */
    {
        DeviceConfig c; plan(&c, 0, NULL, NULL); c.ztech_count = 1;
        c.genset_type = GENSET_SEDEMAC; c.genset_slave = 4;
        snprintf(c.sfx_genset, sizeof(c.sfx_genset), "04");
        CHECK(!validate_suffixes(&c, err, sizeof(err)) || !validate_bus_addresses(&c, err, sizeof(err)),
              "genset at slave 4 vs Ztech1 at 4 must be refused");
    }
    /* DI default 11 vs a dispenser can never collide (max 06); vs genset 11 does */
    {
        DeviceConfig c; plan(&c, 0, NULL, NULL); c.di_enable = 1;
        c.genset_type = GENSET_SEDEMAC; c.genset_slave = 11;
        snprintf(c.sfx_genset, sizeof(c.sfx_genset), "11");
        CHECK(!validate_suffixes(&c, err, sizeof(err)) && strstr(err, "duplicate"),
              "genset 11 vs DI 11 must be refused (got '%s')", err);
    }
}

int main(void) {
    printf("== alds-gg dispenser plan: suffix = number = address ==\n");
    t_number();
    t_loader();
    t_validator();
    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
