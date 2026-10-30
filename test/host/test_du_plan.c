/* The dispenser-plan rules of alds-gg: SIX SLOTS; A SLOT IS ON EXACTLY WHEN ITS
 * SUFFIX IS SET; THE SUFFIX IS THE DISPENSER NUMBER IS THE BUS ADDRESS (the
 * btpl meter rule, 2026-10-16). Pinned here against the production code:
 *
 *   1. du_number_of() / du_slot_on()   (src/nvs_config.c, compiled whole) —
 *                            the ONE place a suffix becomes a number;
 *   2. nvs_config_load()     the way-out guard: a bad slot is turned OFF, never
 *                            re-addressed; and a plan stored by a build before
 *                            the slot rule (a LIST, "" = position) is read as
 *                            slots polling exactly the same dispensers;
 *   3. validate_suffixes() + validate_bus_addresses() (EXTRACTED verbatim
 *                            from src/main.c) — the way-in guard;
 *   4. the setconfig field readers (EXTRACTED) — a typo is refused, never
 *                            quietly re-read as a different valid value;
 *   5. setcfg_du_slots()     (EXTRACTED) — duCount is a cross-check, and the
 *                            station scenario: dispensers at slave ids 1, 2, 3,
 *                            the first switched off, the other two still
 *                            publish as 02 and 03;
 *   6. nvs_config_clear_equipment() (erase_config) — every equipment field
 *                            back to its default, the APN/broker run kept byte
 *                            for byte, and the next boot "unconfigured" on the
 *                            same network and broker.
 *
 * Every refusal reason is asserted by the token the cloud will see. */
#include "shim.h"
#include <ctype.h>
#include <stddef.h>
#include "nvs.h"
#include "extracted_plan.inc"
#include "extracted_setcfg.inc"

int g_log_quiet = 1;
DeviceConfig g_cfg;
char g_hw_du[DU_MAX_DISPENSERS][24];

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

/* Slots 0..n-1 get sfx[i] ("" or NULL = OFF) and make[i]; du_count = the
 * slots ON, as the loader and setconfig keep it. du_plan stays DU_PLAN_LIST
 * (0) — callers that store a slot-format blob set DU_PLAN_SLOTS themselves. */
static void plan(DeviceConfig *c, int n, const char *const *sfx, const uint8_t *make) {
    memset(c, 0, sizeof(*c));
    int on = 0;
    for (int i = 0; i < n; i++) {
        if (sfx && sfx[i]) snprintf(c->du_sfx[i], sizeof(c->du_sfx[i]), "%s", sfx[i]);
        if (make) c->du_make[i] = make[i];
        if (c->du_sfx[i][0]) on++;
    }
    c->du_count     = (uint8_t)on;
    c->genset_slave = GENSET_SLAVE_DEFAULT;
    c->genset_baud  = 9600;
}

/* ── 1. du_number_of / du_slot_on ──────────────────────────────────────── */
static void t_number(void) {
    printf("[1] du_number_of: the suffix, or 0 (slot OFF)\n");
    DeviceConfig c;
    const char *s1[] = {"", "", "", "", "", ""};
    plan(&c, 6, s1, NULL);
    for (int i = 0; i < 6; i++) {
        CHECK(du_number_of(&c, i) == 0, "no suffix: slot %d must be OFF (got %d)", i + 1, du_number_of(&c, i));
        CHECK(!du_slot_on(&c, i), "no suffix: slot %d must report OFF", i + 1);
    }

    const char *s2[] = {"03", "01", "06", "05", "", "02"};
    plan(&c, 6, s2, NULL);
    int  want[] = {3, 1, 6, 5, 0, 2};   /* slot 5 is OFF — never its position */
    bool on[]   = {1, 1, 1, 1, 0, 1};
    for (int i = 0; i < 6; i++) {
        CHECK(du_number_of(&c, i) == want[i], "suffix '%s' at slot %d -> %d (want %d)",
              s2[i], i + 1, du_number_of(&c, i), want[i]);
        CHECK(du_slot_on(&c, i) == on[i], "slot %d ON must be %d", i + 1, on[i]);
    }

    /* garbage that never survives the loader but may reach the validator:
     * the slot is ON (a suffix is set) but names no dispenser */
    const char *s3[] = {"A1", "7", "1a", "0x", "00", " 1"};
    plan(&c, 6, s3, NULL);
    for (int i = 0; i < 6; i++) {
        CHECK(du_number_of(&c, i) == 0, "'%s' must resolve to 0 (got %d)", s3[i], du_number_of(&c, i));
        CHECK(du_slot_on(&c, i), "'%s' is set, so the slot is ON (the validator refuses it)", s3[i]);
    }

    CHECK(du_number_of(NULL, 0) == 0 && du_number_of(&c, -1) == 0 && du_number_of(&c, DU_MAX_DISPENSERS) == 0,
          "out-of-range arguments must resolve to 0");
    CHECK(!du_slot_on(NULL, 0) && !du_slot_on(&c, -1) && !du_slot_on(&c, DU_MAX_DISPENSERS),
          "out-of-range arguments must report OFF");
}

/* ── 2. the loader ─────────────────────────────────────────────────────── */
static void save_and_load(const DeviceConfig *in, DeviceConfig *out) {
    simnvs_reset();
    simnvs_set_tag(PRODUCT_ID);
    simnvs_set_blob_raw(in, sizeof(*in));
    nvs_config_load(out);
}

static void slots_blob(DeviceConfig *c, int n, const char *const *sfx, const uint8_t *make) {
    plan(c, n, sfx, make);
    c->du_plan = DU_PLAN_SLOTS;
}

static void t_loader(void) {
    printf("[2] nvs_config_load: bad slots OFF, old list plans read as slots\n");
    DeviceConfig in, out;

    /* THE station case: dispenser 1 switched off, 2 and 3 polled */
    { const char *s[] = {"", "02", "03"}; slots_blob(&in, 3, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 2 && !du_slot_on(&out, 0) && du_number_of(&out, 1) == 2 &&
          du_number_of(&out, 2) == 3 && out.du_plan == DU_PLAN_SLOTS,
          "slots ''/02/03 must load as dispensers 2 and 3 only (count %u, slot1 %d)",
          out.du_count, du_number_of(&out, 0));

    /* a gapped plan survives intact */
    { const char *s[] = {"01", "03", "05"}; slots_blob(&in, 3, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 3 && du_number_of(&out, 1) == 3 && du_number_of(&out, 2) == 5,
          "slots 01/03/05 must load intact (got count %u)", out.du_count);

    /* a lettered suffix turns that slot OFF — never back to its position */
    { const char *s[] = {"02", "A1"}; slots_blob(&in, 2, s, NULL); }
    save_and_load(&in, &out);
    CHECK(!du_slot_on(&out, 1) && out.du_count == 1 && du_number_of(&out, 0) == 2,
          "lettered slot 2 must go OFF, slot 1 stays dispenser 2 (count %u)", out.du_count);

    /* out of range: 07 — only that slot goes OFF, the ones after it stay */
    { const char *s[] = {"01", "07", "02"}; slots_blob(&in, 3, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 2 && !du_slot_on(&out, 1) && du_number_of(&out, 2) == 2,
          "07 is unaddressable: slot 2 OFF, slots 1 and 3 kept (count %u)", out.du_count);

    /* Yenen at 06 */
    { const char *s[] = {"01", "06"}; uint8_t m[] = {DU_MAKE_BLUESKY, DU_MAKE_YENEN}; slots_blob(&in, 2, s, m); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 1 && !du_slot_on(&out, 1),
          "Yenen 06 has no nozzle code: slot 2 must go OFF (count %u)", out.du_count);

    /* BlueSky at 06 is fine */
    { const char *s[] = {"01", "06"}; slots_blob(&in, 2, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 2, "BlueSky 06 is addressable (got %u)", out.du_count);

    /* duplicate suffixes: the later slot goes OFF */
    { const char *s[] = {"02", "02"}; slots_blob(&in, 2, s, NULL); }
    save_and_load(&in, &out);
    CHECK(out.du_count == 1 && du_slot_on(&out, 0) && !du_slot_on(&out, 1),
          "02/02: slot 2 must go OFF (count %u)", out.du_count);

    /* a stored count that disagrees with the suffixes is re-derived */
    { const char *s[] = {"", "04", "", "06"}; slots_blob(&in, 4, s, NULL); in.du_count = 5; }
    save_and_load(&in, &out);
    CHECK(out.du_count == 2, "count must follow the suffixes: 2 slots ON (got %u)", out.du_count);

    /* ── plans stored before the slot rule (du_plan 0 = a list) ── */
    /* no suffixes: entry N = dispenser N, exactly as the old build polled */
    { const char *s[] = {"", "", ""}; plan(&in, 3, s, NULL); in.du_count = 3; }
    save_and_load(&in, &out);
    CHECK(out.du_count == 3 && du_number_of(&out, 0) == 1 && du_number_of(&out, 1) == 2 &&
          du_number_of(&out, 2) == 3 && out.du_plan == DU_PLAN_SLOTS,
          "old list of 3, no suffixes, must become slots 01/02/03 (count %u)", out.du_count);
    CHECK(strcmp(out.du_sfx[0], "01") == 0 && strcmp(out.du_sfx[2], "03") == 0,
          "the converted slots must carry their suffixes (got '%s'/'%s')", out.du_sfx[0], out.du_sfx[2]);

    /* explicit suffixes kept, empty ones = position, past the list = never polled */
    { const char *s[] = {"03", "", "05"}; plan(&in, 3, s, NULL); in.du_count = 2; }
    save_and_load(&in, &out);
    CHECK(out.du_count == 2 && du_number_of(&out, 0) == 3 && du_number_of(&out, 1) == 2 &&
          !du_slot_on(&out, 2),
          "old list 03/''(+05 past the count) must become dispensers 3 and 2 only (count %u)",
          out.du_count);

    /* an empty entry whose position another entry names: the old build cut the
     * plan there; the slot rule turns that one slot OFF */
    { const char *s[] = {"02", ""}; plan(&in, 2, s, NULL); in.du_count = 2; }
    save_and_load(&in, &out);
    CHECK(out.du_count == 1 && du_number_of(&out, 0) == 2 && !du_slot_on(&out, 1),
          "old list 02/'' (position 2 = duplicate) must keep only dispenser 2 (count %u)", out.du_count);

    /* an old list of 0 with stale suffixes polls nothing — still nothing */
    { const char *s[] = {"01", "02"}; plan(&in, 2, s, NULL); in.du_count = 0; }
    save_and_load(&in, &out);
    CHECK(out.du_count == 0 && !du_slot_on(&out, 0) && !du_slot_on(&out, 1),
          "old list of 0 must stay empty (count %u)", out.du_count);

    /* THE FOTA case: a blob SHORTER than this struct (written before du_plan
     * existed) loads as the old list, and is NOT rewritten at boot — a rollback
     * to the old build must still find a blob it can read. */
    {
        const char *s[] = {"", ""};
        plan(&in, 2, s, NULL); in.du_count = 2;
        size_t old_len = offsetof(DeviceConfig, du_plan);
        simnvs_reset();
        simnvs_set_tag(PRODUCT_ID);
        simnvs_set_blob_raw(&in, old_len);
        nvs_config_load(&out);
        CHECK(out.du_count == 2 && du_number_of(&out, 0) == 1 && du_number_of(&out, 1) == 2,
              "short old blob must load as dispensers 1 and 2 (count %u)", out.du_count);
        CHECK(simnvs_blob_len() == old_len,
              "the short blob must not be rewritten at boot (len %zu, want %zu)",
              simnvs_blob_len(), old_len);
    }
}

/* ── 3. the validator ──────────────────────────────────────────────────── */
static bool ok_plan(int n, const char *const *sfx, const uint8_t *make, char *err, size_t sz) {
    DeviceConfig c;
    plan(&c, n, sfx, make);
    err[0] = '\0';
    return validate_suffixes(&c, err, sz) && validate_bus_addresses(&c, err, sz);
}

static void t_validator(void) {
    printf("[3] validate_suffixes / validate_bus_addresses\n");
    char err[160];

    { const char *s[] = {"01", "02", "03", "04", "05", "06"};
      CHECK(ok_plan(6, s, NULL, err, sizeof(err)), "six dispensers 01..06: %s", err); }
    { const char *s[] = {"", "02", "03"};
      CHECK(ok_plan(3, s, NULL, err, sizeof(err)), "dispenser 1 OFF, 02/03 ON: %s", err); }
    { const char *s[] = {"", "", "", "", "", ""};
      CHECK(ok_plan(6, s, NULL, err, sizeof(err)), "every slot OFF is a valid (empty) plan: %s", err); }
    { const char *s[] = {"01", "03", "05"};
      CHECK(ok_plan(3, s, NULL, err, sizeof(err)), "gapped 01/03/05: %s", err); }
    { const char *s[] = {"06"};
      CHECK(ok_plan(1, s, NULL, err, sizeof(err)), "BlueSky 06: %s", err); }
    { const char *s[] = {"05"}; uint8_t m[] = {DU_MAKE_YENEN};
      CHECK(ok_plan(1, s, m, err, sizeof(err)), "Yenen 05: %s", err); }
    { const char *s[] = {"02", ""};
      CHECK(ok_plan(2, s, NULL, err, sizeof(err)), "an OFF slot claims no suffix (no position fallback): %s", err); }

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
    { const char *s[] = {"", "03", "", "03"};
      CHECK(!ok_plan(4, s, NULL, err, sizeof(err)) && strstr(err, "DU2") && strstr(err, "DU4"),
            "two slots naming dispenser 3 must be refused by slot name (got '%s')", err); }

    /* cross-unit: dispenser 04 vs a Ztech tank at slave 4 shares the HW_ID */
    {
        DeviceConfig c; const char *s[] = {"01", "02", "03", "04"};
        plan(&c, 4, s, NULL); c.ztech_count = 1;
        err[0] = '\0';
        CHECK(!validate_suffixes(&c, err, sizeof(err)) && strstr(err, "DU4") && strstr(err, "Ztech1"),
              "DU4 vs Ztech1 both 04 must be refused by name (got '%s')", err);
        snprintf(c.sfx_ztech[0], sizeof(c.sfx_ztech[0]), "07");
        CHECK(validate_suffixes(&c, err, sizeof(err)) && validate_bus_addresses(&c, err, sizeof(err)),
              "moving the tank to 07 resolves it: %s", err);
        /* switching dispenser 4 OFF frees 04 for the tank's auto suffix */
        c.sfx_ztech[0][0] = '\0';
        c.du_sfx[3][0] = '\0';
        CHECK(validate_suffixes(&c, err, sizeof(err)) && validate_bus_addresses(&c, err, sizeof(err)),
              "DU4 OFF: Ztech1 may take 04: %s", err);
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

/* ── 4. setconfig field readers ────────────────────────────────────────── */
static bool gaddr(DeviceConfig *c, const char *json, char *err, size_t n) {
    err[0] = '\0';
    return setcfg_genset_addr(json, c, err, n);
}

static void t_setcfg(void) {
    printf("[4] setconfig readers: genset address, suffix length, switches\n");
    char err[160];
    DeviceConfig c;

    /* THE bug: a stored "10" must not swallow a later gensetSlave-only push */
    plan(&c, 0, NULL, NULL); c.genset_type = GENSET_SEDEMAC;
    c.genset_slave = 10; snprintf(c.sfx_genset, sizeof(c.sfx_genset), "10");
    CHECK(gaddr(&c, "{\"gensetSlave\":4}", err, sizeof(err)) && c.genset_slave == 4 &&
          strcmp(c.sfx_genset, "04") == 0,
          "gensetSlave 4 alone must move the DG to 4/04 (got %u/%s, '%s')", c.genset_slave, c.sfx_genset, err);
    CHECK(gaddr(&c, "{\"sfxGenset\":\"03\"}", err, sizeof(err)) && c.genset_slave == 3 &&
          strcmp(c.sfx_genset, "03") == 0,
          "sfxGenset 03 alone must set slave 3 (got %u/%s)", c.genset_slave, c.sfx_genset);
    CHECK(gaddr(&c, "{\"sfxGenset\":\"08\"}", err, sizeof(err)) && c.genset_slave == 8 &&
          strcmp(c.sfx_genset, "08") == 0,
          "sfxGenset 08 alone must set slave 8 (got %u/%s)", c.genset_slave, c.sfx_genset);
    CHECK(gaddr(&c, "{\"gensetSlave\":7,\"sfxGenset\":\"07\"}", err, sizeof(err)) && c.genset_slave == 7,
          "an agreeing pair must apply (got %u, '%s')", c.genset_slave, err);
    CHECK(!gaddr(&c, "{\"gensetSlave\":4,\"sfxGenset\":\"03\"}", err, sizeof(err)) &&
          strstr(err, "disagree") && c.genset_slave == 7,
          "a disagreeing pair must be refused, nothing changed (got '%s', slave %u)", err, c.genset_slave);
    CHECK(gaddr(&c, "{\"gensetSlave\":\"12\"}", err, sizeof(err)) && c.genset_slave == 12 &&
          strcmp(c.sfx_genset, "12") == 0,
          "a quoted gensetSlave must apply (got %u/%s)", c.genset_slave, c.sfx_genset);

    /* out of range is refused, never wrapped (300 used to land on slave 44) */
    CHECK(!gaddr(&c, "{\"gensetSlave\":300}", err, sizeof(err)) && strstr(err, "1..247") && c.genset_slave == 12,
          "gensetSlave 300 must be refused (got '%s', slave %u)", err, c.genset_slave);
    CHECK(!gaddr(&c, "{\"gensetSlave\":0}", err, sizeof(err)), "gensetSlave 0 must be refused");
    CHECK(!gaddr(&c, "{\"gensetSlave\":-1}", err, sizeof(err)), "gensetSlave -1 must be refused");

    /* above 99: allowed, but no 2-digit suffix (the HW_ID falls back to 10) */
    CHECK(gaddr(&c, "{\"gensetSlave\":120}", err, sizeof(err)) && c.genset_slave == 120 && !c.sfx_genset[0],
          "gensetSlave 120 must apply with no suffix (got %u/'%s')", c.genset_slave, c.sfx_genset);

    /* sfxGenset "" = back to the slave's own number; neither key = untouched */
    c.genset_slave = 9; snprintf(c.sfx_genset, sizeof(c.sfx_genset), "09");
    CHECK(gaddr(&c, "{\"sfxGenset\":\"\"}", err, sizeof(err)) && c.genset_slave == 9 &&
          strcmp(c.sfx_genset, "09") == 0, "sfxGenset \"\" must fall back to the slave (got %s)", c.sfx_genset);
    CHECK(gaddr(&c, "{\"gensetType\":1}", err, sizeof(err)) && c.genset_slave == 9 &&
          strcmp(c.sfx_genset, "09") == 0, "a push without either key must not touch the pair");

    /* lettered: copied through, and validate_suffixes() refuses it */
    CHECK(gaddr(&c, "{\"sfxGenset\":\"G1\"}", err, sizeof(err)) && strcmp(c.sfx_genset, "G1") == 0 &&
          c.genset_slave == 9, "a lettered sfxGenset must reach the validator untouched");
    CHECK(!validate_suffixes(&c, err, sizeof(err)) && strstr(err, "Genset"),
          "the validator must refuse the lettered genset suffix (got '%s')", err);

    /* suffix length: "045" must not be clamped into the valid "04" */
    char s3[3] = "07";
    CHECK(!setcfg_get_sfx("{\"sfxZtech1\":\"045\"}", "sfxZtech1", s3, sizeof(s3), err, sizeof(err)) &&
          strstr(err, "sfxZtech1") && strcmp(s3, "07") == 0,
          "045 must be refused, field untouched (got '%s', field %s)", err, s3);
    CHECK(!setcfg_get_sfx("{\"du1Sfx\":5}", "du1Sfx", s3, sizeof(s3), err, sizeof(err)) && strcmp(s3, "07") == 0,
          "an unquoted suffix must be refused, not read as absent (got '%s')", err);
    CHECK(setcfg_get_sfx("{\"sfxDI\":\"05\"}", "sfxDI", s3, sizeof(s3), err, sizeof(err)) && strcmp(s3, "05") == 0,
          "05 must be taken (got %s)", s3);
    CHECK(setcfg_get_sfx("{\"sfxDI\":\"\"}", "sfxDI", s3, sizeof(s3), err, sizeof(err)) && !s3[0],
          "an empty suffix must clear the field (back to the default)");
    snprintf(s3, sizeof(s3), "08");
    CHECK(setcfg_get_sfx("{\"duCount\":1}", "sfxDI", s3, sizeof(s3), err, sizeof(err)) && strcmp(s3, "08") == 0,
          "an absent key must leave the field untouched");

    /* switches: out of range refused, quoted accepted, absent untouched */
    int v = 1;
    CHECK(!setcfg_int_in_range("{\"gensetType\":2}", "gensetType", 0, 1, &v, err, sizeof(err)) &&
          v == 1 && strstr(err, "gensetType"), "gensetType 2 must be refused (got '%s')", err);
    CHECK(!setcfg_int_in_range("{\"diEnable\":16}", "diEnable", 0, 15, &v, err, sizeof(err)) && v == 1,
          "diEnable 16 must be refused (got '%s')", err);
    CHECK(setcfg_int_in_range("{\"diEnable\":\"15\"}", "diEnable", 0, 15, &v, err, sizeof(err)) && v == 15,
          "a quoted diEnable must apply (got %d)", v);
    CHECK(setcfg_int_in_range("{\"x\":1}", "diEnable", 0, 15, &v, err, sizeof(err)) && v == 15,
          "an absent switch must stay as it was");

    /* quoted counts: used to read as absent and keep the old value */
    int n = 0;
    CHECK(setcfg_int_in_range("{\"duCount\":\"2\"}", "duCount", 0, 6, &n, err, sizeof(err)) && n == 2,
          "a quoted duCount must read as 2 (got %d)", n);
    n = 5;
    CHECK(!setcfg_int_in_range("{\"duCount\":\"x\"}", "duCount", 0, 6, &n, err, sizeof(err)) && n == 5,
          "a garbled count must be refused and keep the old value (got %d)", n);
}

/* ── 5. setcfg_du_slots: the slot rule on the way in ──────────────────── */
static bool slots(DeviceConfig *c, const char *json, char *err, size_t n) {
    err[0] = '\0';
    return setcfg_du_slots(json, c, err, n);
}

static void t_slots(void) {
    printf("[5] setcfg_du_slots: slot ON only with its suffix, duCount a cross-check\n");
    char err[160];
    DeviceConfig c;

    /* THE station: three BlueSky dispensers at slave ids 1, 2, 3 — the first
     * switched off, the second and third must publish on 02 and 03 */
    plan(&c, 0, NULL, NULL);
    CHECK(slots(&c, "{\"duCount\":2,\"du1Sfx\":\"\",\"du2Make\":\"bluesky\",\"du2Sfx\":\"02\","
                    "\"du3Make\":\"bluesky\",\"du3Sfx\":\"03\"}", err, sizeof(err)) &&
          c.du_count == 2 && c.du_plan == DU_PLAN_SLOTS,
          "DU1 off, DU2/DU3 on must be accepted (got '%s', count %u)", err, c.du_count);
    CHECK(validate_suffixes(&c, err, sizeof(err)) && validate_bus_addresses(&c, err, sizeof(err)),
          "that plan must pass the validators: %s", err);
    {
        /* what du_task, build_hw_ids() and the nozzle labels will do with it */
        int polled = 0, nums[DU_MAX_DISPENSERS] = {0};
        for (int i = 0; i < DU_MAX_DISPENSERS; i++)
            if (du_slot_on(&c, i)) nums[polled++] = du_number_of(&c, i);
        CHECK(polled == 2 && nums[0] == 2 && nums[1] == 3,
              "exactly dispensers 2 and 3 polled (got %d: %d,%d)", polled, nums[0], nums[1]);
        CHECK(2 * nums[0] - 1 == 3 && 2 * nums[0] == 4 && 2 * nums[1] - 1 == 5 && 2 * nums[1] == 6,
              "BlueSky nozzle addresses must be 3/4 and 5/6 — dispenser 1's 1/2 untouched");
        char hw[16];
        snprintf(hw, sizeof(hw), "%02d", du_number_of(&c, 1));
        CHECK(strcmp(hw, "02") == 0 && strcmp(c.du_sfx[1], "02") == 0 && strcmp(c.du_sfx[2], "03") == 0,
              "HW_ID suffixes must be 02 and 03 (got %s / %s)", c.du_sfx[1], c.du_sfx[2]);
    }

    /* the same station configured compactly (slots 1 and 2 naming 02 and 03) */
    plan(&c, 0, NULL, NULL);
    CHECK(slots(&c, "{\"duCount\":2,\"du1Sfx\":\"02\",\"du2Sfx\":\"03\"}", err, sizeof(err)) &&
          c.du_count == 2 && du_number_of(&c, 0) == 2 && du_number_of(&c, 1) == 3,
          "compact 02/03 must poll dispensers 2 and 3 as well (got '%s')", err);

    /* duCount must match the slots ON */
    plan(&c, 0, NULL, NULL);
    CHECK(!slots(&c, "{\"duCount\":3,\"du2Sfx\":\"02\",\"du3Sfx\":\"03\"}", err, sizeof(err)) &&
          strstr(err, "duCount 3 but 2"),
          "duCount 3 with 2 suffixes must be refused naming both (got '%s')", err);
    plan(&c, 0, NULL, NULL);
    CHECK(!slots(&c, "{\"duCount\":2}", err, sizeof(err)) && strstr(err, "duCount 2 but 0"),
          "a count with no suffix polls nothing — refused (got '%s')", err);

    /* switching one dispenser off on a running station: one key */
    { const char *s[] = {"01", "02", "03"}; plan(&c, 3, s, NULL); c.du_plan = DU_PLAN_SLOTS; }
    CHECK(slots(&c, "{\"du1Sfx\":\"\"}", err, sizeof(err)) && c.du_count == 2 && !du_slot_on(&c, 0),
          "du1Sfx \"\" alone must switch dispenser 1 off (got '%s', count %u)", err, c.du_count);
    { const char *s[] = {"01", "02", "03"}; plan(&c, 3, s, NULL); c.du_plan = DU_PLAN_SLOTS; }
    CHECK(!slots(&c, "{\"duCount\":2}", err, sizeof(err)) && strstr(err, "duCount 2 but 3"),
          "a lower duCount alone switches nothing off — refused (got '%s')", err);
    CHECK(c.du_count == 3, "a refused push leaves the staged count alone");

    /* make and suffix readers */
    plan(&c, 0, NULL, NULL);
    CHECK(!slots(&c, "{\"du1Make\":\"tokheim\",\"du1Sfx\":\"01\"}", err, sizeof(err)) &&
          strstr(err, "du_make_invalid"), "an unknown make must be refused (got '%s')", err);
    CHECK(slots(&c, "{\"du1Make\":\"YENEN\",\"du1Sfx\":\"01\"}", err, sizeof(err)) &&
          c.du_make[0] == DU_MAKE_YENEN, "make is case-insensitive (got '%s')", err);
    CHECK(!slots(&c, "{\"du2Sfx\":\"045\"}", err, sizeof(err)) && strstr(err, "du2Sfx"),
          "a 3-char suffix must be refused naming the key (got '%s')", err);
    CHECK(!slots(&c, "{\"du2Sfx\":2}", err, sizeof(err)) && strstr(err, "du2Sfx"),
          "an unquoted suffix must be refused (got '%s')", err);

    /* a push without any dispenser key leaves the plan as stored */
    { const char *s[] = {"", "02", "03"}; plan(&c, 3, s, NULL); c.du_plan = DU_PLAN_SLOTS; }
    CHECK(slots(&c, "{\"gpsEnable\":true}", err, sizeof(err)) && c.du_count == 2 &&
          !du_slot_on(&c, 0) && du_number_of(&c, 2) == 3,
          "an unrelated push must not touch the slots (got '%s')", err);
}

/* ── 6. erase_config: the equipment goes, the network and broker stay ───── */
static void t_erase_config(void) {
    printf("[6] erase_config: equipment back to defaults, connectivity byte for byte\n");
    DeviceConfig c, before, def, boot;
    const char *sfx[] = {"01", "02", "", "04", "", ""};
    const uint8_t make[] = {DU_MAKE_BLUESKY, DU_MAKE_YENEN, DU_MAKE_BLUESKY,
                            DU_MAKE_BLUESKY, DU_MAKE_BLUESKY, DU_MAKE_BLUESKY};
    slots_blob(&c, 6, sfx, make);
    c.ztech_count   = 2;
    c.pantech_count = 1;
    c.genset_type   = GENSET_SEDEMAC;
    c.genset_model  = 2;
    c.genset_slave  = 12;
    c.genset_baud   = 19200;
    c.di_enable     = 0x0F;
    c.gps_enable    = true;
    snprintf(c.sfx_genset,     sizeof(c.sfx_genset),     "12");
    snprintf(c.sfx_di,         sizeof(c.sfx_di),         "11");
    snprintf(c.sfx_ztech[0],   sizeof(c.sfx_ztech[0]),   "05");
    snprintf(c.sfx_pantech[0], sizeof(c.sfx_pantech[0]), "07");
    /* A station on its own APN and broker, with a rollback copy and a pending
     * rollback notice: none of it may move. */
    snprintf(c.apn,                 sizeof(c.apn),                 "custom.apn");
    snprintf(c.apn_user,            sizeof(c.apn_user),            "u");
    snprintf(c.apn_pass,            sizeof(c.apn_pass),            "p");
    snprintf(c.mqtt_host,           sizeof(c.mqtt_host),           "broker.example.com");
    snprintf(c.mqtt_user,           sizeof(c.mqtt_user),           "site");
    snprintf(c.mqtt_pass,           sizeof(c.mqtt_pass),           "secret");
    snprintf(c.mqtt_base_topic,     sizeof(c.mqtt_base_topic),     "client");
    snprintf(c.apn_bak,             sizeof(c.apn_bak),             "old.apn");
    snprintf(c.mqtt_host_bak,       sizeof(c.mqtt_host_bak),       "old.example.com");
    snprintf(c.mqtt_base_topic_bak, sizeof(c.mqtt_base_topic_bak), "oldbase");
    c.mqtt_port            = 1883;
    c.mqtt_tls             = false;
    c.mqtt_port_bak        = 8883;
    c.mqtt_tls_bak         = true;
    c.conn_rollback_notify = true;
    c.cred_fp              = 0x5A5A1234u;
    memcpy(&before, &c, sizeof(c));

    nvs_config_clear_equipment(&c);
    nvs_config_defaults(&def);

    CHECK(c.du_count == 0, "duCount must be 0 (got %u)", c.du_count);
    for (int i = 0; i < DU_MAX_DISPENSERS; i++)
        CHECK(!c.du_sfx[i][0] && c.du_make[i] == def.du_make[i],
              "DU slot %d must be OFF with the default make (got '%s', make %u)", i + 1, c.du_sfx[i], c.du_make[i]);
    CHECK(c.ztech_count == 0 && c.pantech_count == 0, "radar tanks must be 0 (got %u/%u)",
          c.ztech_count, c.pantech_count);
    CHECK(c.genset_type == GENSET_OFF && c.genset_model == def.genset_model &&
          c.genset_slave == def.genset_slave && c.genset_baud == def.genset_baud,
          "genset must be OFF on the default model/slave/baud (got %u/%u/%u/%lu)", c.genset_type,
          c.genset_model, c.genset_slave, (unsigned long)c.genset_baud);
    CHECK(c.di_enable == 0 && !c.gps_enable, "DI and GPS must be off (got 0x%02x, %d)", c.di_enable, c.gps_enable);
    CHECK(!c.sfx_genset[0] && !c.sfx_di[0], "genset/DI suffixes must be cleared (got '%s'/'%s')",
          c.sfx_genset, c.sfx_di);
    for (int i = 0; i < RADAR_MAX; i++)
        CHECK(!c.sfx_ztech[i][0] && !c.sfx_pantech[i][0] && !c.reserved_sfx[i][0],
              "tank %d suffixes must be cleared", i + 1);
    CHECK(c.du_plan == DU_PLAN_SLOTS, "the empty plan must be stored as slots, never as an old list (got %u)",
          c.du_plan);

    const size_t from = offsetof(DeviceConfig, apn);
    const size_t to   = offsetof(DeviceConfig, cred_fp) + sizeof(c.cred_fp);
    CHECK(memcmp((const char *)&c + from, (const char *)&before + from, to - from) == 0,
          "APN, broker, rollback copy, trial flags and credential stamp must be kept byte for byte");
    CHECK(strcmp(c.apn, "custom.apn") == 0 && strcmp(c.mqtt_host, "broker.example.com") == 0 &&
          c.mqtt_port == 1883 && strcmp(c.mqtt_base_topic, "client") == 0 && !c.mqtt_tls &&
          c.conn_rollback_notify && c.cred_fp == 0x5A5A1234u,
          "the connectivity values must read back unchanged");

    DeviceConfig again;
    memcpy(&again, &c, sizeof(c));
    nvs_config_clear_equipment(&again);
    CHECK(memcmp(&again, &c, sizeof(c)) == 0, "erasing an erased configuration must change nothing");

    /* The next boot: the saved blob loads as "unconfigured" (main.c's rule),
     * on the same network and broker. */
    save_and_load(&c, &boot);
    CHECK(boot.du_count == 0 && boot.ztech_count == 0 && boot.pantech_count == 0 &&
          boot.genset_type == GENSET_OFF && !boot.di_enable && !boot.gps_enable,
          "the next boot must come up unconfigured (du %u zt %u pt %u gen %u di 0x%02x gps %d)",
          boot.du_count, boot.ztech_count, boot.pantech_count, boot.genset_type, boot.di_enable,
          boot.gps_enable);
    for (int i = 0; i < DU_MAX_DISPENSERS; i++)
        CHECK(!du_slot_on(&boot, i), "after the boot DU slot %d must be OFF", i + 1);
    CHECK(strcmp(boot.apn, "custom.apn") == 0 && strcmp(boot.mqtt_host, "broker.example.com") == 0 &&
          boot.mqtt_port == 1883 && strcmp(boot.mqtt_base_topic, "client") == 0 && !boot.mqtt_tls &&
          strcmp(boot.mqtt_host_bak, "old.example.com") == 0 && boot.cred_fp == 0x5A5A1234u,
          "after the boot the network and broker must be the ones it had (apn '%s' host '%s' port %u)",
          boot.apn, boot.mqtt_host, boot.mqtt_port);
}

int main(void) {
    printf("== alds-gg dispenser plan: slot ON with its suffix, suffix = number = address ==\n");
    t_number();
    t_loader();
    t_validator();
    t_setcfg();
    t_slots();
    t_erase_config();
    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
