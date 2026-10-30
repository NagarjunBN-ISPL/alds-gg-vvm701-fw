/* The setconfig field readers, EXTRACTED verbatim from src/main.c: a typo in
 * a push is refused with a reason, never quietly re-read as a different valid
 * value that the device would then ack config_successful.
 *
 *   setcfg_genset_addr()   sfxGenset <-> gensetSlave, synced by what the push
 *                          carries (a stored "10" once swallowed a later
 *                          {"gensetSlave":4});
 *   setcfg_get_sfx()       "045" is not clamped into "04", an unquoted suffix
 *                          is not read as absent;
 *   setcfg_int_in_range()  every number key: whole numbers only (quoted is
 *                          fine), out of range REFUSED — never clamped,
 *                          defaulted or ignored;
 *   setcfg_get_bool()      switches: true/false/1/0, quoted or not, else
 *                          refused;
 *   setcfg_get_text()      strings: not a string, or longer than the field,
 *                          refused — never cut to size;
 *   setcfg_check_members() the payload as a whole: one complete object, only
 *                          known keys, each once, plain values;
 *   factory_reset_refusal() the reset's two checks: this gateway's own IMEI in
 *                          "confirm", and no unsent readings unless "force". */
#include "shim.h"
#include <ctype.h>
#define SETCFG_KEYS_MAX 64   /* = main.c's, next to s_setcfg_known_keys */
#include "extracted_setcfg_all.inc"   /* td uses the smaller extracted_setcfg.inc */

int g_log_quiet = 1;

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static void stored(DeviceConfig *c, int slave, const char *sfx) {
    memset(c, 0, sizeof(*c));
    c->genset_type  = GENSET_SEDEMAC;
    c->genset_slave = (uint8_t)slave;
    snprintf(c->sfx_genset, sizeof(c->sfx_genset), "%s", sfx);
}

static bool gaddr(DeviceConfig *c, const char *json, char *err, size_t n) {
    err[0] = '\0';
    return setcfg_genset_addr(json, c, err, n);
}

int main(void) {
    char err[160];
    DeviceConfig c;

    printf("[1] genset address: sfxGenset <-> gensetSlave\n");
    stored(&c, 10, "10");
    CHECK(gaddr(&c, "{\"gensetSlave\":4}", err, sizeof(err)) && c.genset_slave == 4 &&
          strcmp(c.sfx_genset, "04") == 0,
          "gensetSlave 4 alone must move the DG to 4/04 (got %u/%s, '%s')", c.genset_slave, c.sfx_genset, err);
    CHECK(gaddr(&c, "{\"sfxGenset\":\"03\"}", err, sizeof(err)) && c.genset_slave == 3 &&
          strcmp(c.sfx_genset, "03") == 0, "sfxGenset 03 alone must set slave 3 (got %u/%s)", c.genset_slave, c.sfx_genset);
    CHECK(gaddr(&c, "{\"gensetSlave\":7,\"sfxGenset\":\"07\"}", err, sizeof(err)) && c.genset_slave == 7,
          "an agreeing pair must apply (got %u, '%s')", c.genset_slave, err);
    CHECK(!gaddr(&c, "{\"gensetSlave\":4,\"sfxGenset\":\"03\"}", err, sizeof(err)) &&
          strstr(err, "disagree") && c.genset_slave == 7,
          "a disagreeing pair must be refused, nothing changed (got '%s', slave %u)", err, c.genset_slave);
    CHECK(gaddr(&c, "{\"gensetSlave\":\"12\"}", err, sizeof(err)) && c.genset_slave == 12 &&
          strcmp(c.sfx_genset, "12") == 0, "a quoted gensetSlave must apply (got %u/%s)", c.genset_slave, c.sfx_genset);
    CHECK(!gaddr(&c, "{\"gensetSlave\":300}", err, sizeof(err)) && strstr(err, "1..247") && c.genset_slave == 12,
          "gensetSlave 300 must be refused, not wrapped to 44 (got '%s', slave %u)", err, c.genset_slave);
    CHECK(!gaddr(&c, "{\"gensetSlave\":0}", err, sizeof(err)),  "gensetSlave 0 must be refused");
    CHECK(!gaddr(&c, "{\"gensetSlave\":-1}", err, sizeof(err)), "gensetSlave -1 must be refused");
    CHECK(gaddr(&c, "{\"gensetSlave\":120}", err, sizeof(err)) && c.genset_slave == 120 && !c.sfx_genset[0],
          "gensetSlave 120 must apply with no suffix (got %u/'%s')", c.genset_slave, c.sfx_genset);
    stored(&c, 9, "09");
    CHECK(gaddr(&c, "{\"sfxGenset\":\"\"}", err, sizeof(err)) && c.genset_slave == 9 &&
          strcmp(c.sfx_genset, "09") == 0, "sfxGenset \"\" must fall back to the slave (got %s)", c.sfx_genset);
    CHECK(gaddr(&c, "{\"gensetType\":1}", err, sizeof(err)) && c.genset_slave == 9 &&
          strcmp(c.sfx_genset, "09") == 0, "a push without either key must not touch the pair");

    printf("[3] suffix keys: length and type\n");
    char s3[3] = "07";
    CHECK(!setcfg_get_sfx("{\"sfxZtech1\":\"045\"}", "sfxZtech1", s3, sizeof(s3), err, sizeof(err)) &&
          strstr(err, "sfxZtech1") && strcmp(s3, "07") == 0,
          "045 must be refused, field untouched (got '%s', field %s)", err, s3);
    CHECK(!setcfg_get_sfx("{\"du1Sfx\":5}", "du1Sfx", s3, sizeof(s3), err, sizeof(err)) && strcmp(s3, "07") == 0,
          "an unquoted suffix must be refused, not read as absent (got '%s')", err);
    CHECK(setcfg_get_sfx("{\"sfxDI\":\"09\"}", "sfxDI", s3, sizeof(s3), err, sizeof(err)) && strcmp(s3, "09") == 0,
          "09 must be taken (got %s)", s3);
    CHECK(setcfg_get_sfx("{\"sfxDI\":\"\"}", "sfxDI", s3, sizeof(s3), err, sizeof(err)) && !s3[0],
          "an empty suffix must clear the field (back to the default)");
    snprintf(s3, sizeof(s3), "08");
    CHECK(setcfg_get_sfx("{\"duCount\":1}", "sfxDI", s3, sizeof(s3), err, sizeof(err)) && strcmp(s3, "08") == 0,
          "an absent key must leave the field untouched");

    printf("[4] switches and numbers\n");
    int v = 1;
    CHECK(!setcfg_int_in_range("{\"gensetType\":2}", "gensetType", 0, 1, &v, err, sizeof(err)) &&
          v == 1 && strstr(err, "gensetType"), "gensetType 2 must be refused (got '%s')", err);
    CHECK(!setcfg_int_in_range("{\"diEnable\":16}", "diEnable", 0, 15, &v, err, sizeof(err)) && v == 1,
          "diEnable 16 must be refused (got '%s')", err);
    CHECK(setcfg_int_in_range("{\"diEnable\":\"15\"}", "diEnable", 0, 15, &v, err, sizeof(err)) && v == 15,
          "a quoted diEnable must apply (got %d)", v);
    CHECK(setcfg_int_in_range("{\"x\":1}", "diEnable", 0, 15, &v, err, sizeof(err)) && v == 15,
          "an absent switch must stay as it was");

    printf("[5] numbers are refused, never clamped or ignored\n");
    struct { const char *json; int lo, hi; bool ok; int want; } nums[] = {
        { "{\"duCount\":2}",          0, 6, true,  2 },
        { "{\"duCount\": \"3\" }",    0, 6, true,  3 },
        { "{\"duCount\":0}",          0, 6, true,  0 },
        { "{\"duCount\":6,\"x\":1}",  0, 6, true,  6 },
        { "{\"duCount\":9}",          0, 6, false, 0 },   /* used to become 6 */
        { "{\"duCount\":-1}",         0, 6, false, 0 },   /* used to become 0 = no dispensers */
        { "{\"duCount\":\"two\"}",    0, 6, false, 0 },   /* used to be ignored */
        { "{\"duCount\":2.5}",        0, 6, false, 0 },
        { "{\"duCount\":\"2x\"}",     0, 6, false, 0 },
        { "{\"duCount\":\"\"}",       0, 6, false, 0 },
        { "{\"duCount\":true}",       0, 6, false, 0 },
        { "{\"duCount\":2 x}",        0, 6, false, 0 },
        { "{\"mqttPort\":8777}",      1, 65535, true, 8777 },
        { "{\"mqttPort\":65536}",     1, 65535, false, 0 },
        { "{\"mqttPort\":99999999999}", 1, 65535, false, 0 },   /* 11 digits: no overflow */
        { "{\"gensetModel\":12}",     0, 6, false, 0 },   /* used to become model 0 */
    };
    for (size_t i = 0; i < sizeof(nums) / sizeof(nums[0]); i++) {
        const char *key = strstr(nums[i].json, "duCount") ? "duCount" :
                          strstr(nums[i].json, "mqttPort") ? "mqttPort" : "gensetModel";
        int io = -7;
        err[0] = '\0';
        bool r = setcfg_int_in_range(nums[i].json, key, nums[i].lo, nums[i].hi, &io, err, sizeof(err));
        CHECK(r == nums[i].ok && (r ? io == nums[i].want : io == -7 && strstr(err, key)),
              "%s -> %s (io %d, '%s')", nums[i].json, r ? "taken" : "refused", io, err);
    }

    printf("[6] switches\n");
    struct { const char *json; bool ok; bool want; } sw[] = {
        { "{\"gpsEnable\":true}",     true, true  },
        { "{\"gpsEnable\":false}",    true, false },
        { "{\"gpsEnable\":\"true\"}", true, true  },   /* used to be ignored */
        { "{\"gpsEnable\":\"FALSE\"}",true, false },
        { "{\"gpsEnable\":1}",        true, true  },
        { "{\"gpsEnable\":0 }",       true, false },
        { "{\"gpsEnable\":\"yes\"}",  false, false },  /* used to be ignored */
        { "{\"gpsEnable\":tru}",      false, false },
        { "{\"gpsEnable\":2}",        false, false },
        { "{\"gpsEnable\":truex}",    false, false },
        { "{\"gpsEnable\":\"true}",   false, false },
    };
    for (size_t i = 0; i < sizeof(sw) / sizeof(sw[0]); i++) {
        bool io = !sw[i].want;
        bool before = io;
        err[0] = '\0';
        bool r = setcfg_get_bool(sw[i].json, "gpsEnable", &io, err, sizeof(err));
        CHECK(r == sw[i].ok && (r ? io == sw[i].want : io == before && strstr(err, "gpsEnable")),
              "%s -> %s ('%s')", sw[i].json, r ? "taken" : "refused", err);
    }
    bool keep = true;
    CHECK(setcfg_get_bool("{\"duCount\":1}", "gpsEnable", &keep, err, sizeof(err)) && keep,
          "an absent switch must stay as it was");

    printf("[7] strings: not cut, not ignored\n");
    char f8[8] = "old";
    bool present = false;
    CHECK(setcfg_get_text("{\"apn\":\"airtel\"}", "apn", f8, sizeof(f8), &present, err, sizeof(err)) &&
          present && strcmp(f8, "airtel") == 0, "a fitting string must be taken (got %s)", f8);
    CHECK(setcfg_get_text("{\"apn\":\"1234567\"}", "apn", f8, sizeof(f8), &present, err, sizeof(err)) &&
          strcmp(f8, "1234567") == 0, "a string exactly the field's size must be taken (got %s)", f8);
    CHECK(!setcfg_get_text("{\"apn\":\"12345678\"}", "apn", f8, sizeof(f8), &present, err, sizeof(err)) &&
          strcmp(f8, "1234567") == 0 && strstr(err, "longer than 7"),
          "one character too long must be refused, field untouched (got '%s', %s)", err, f8);
    CHECK(!setcfg_get_text("{\"apn\":5}", "apn", f8, sizeof(f8), &present, err, sizeof(err)) &&
          strstr(err, "must be a string"), "a number for a string key must be refused (got '%s')", err);
    CHECK(setcfg_get_text("{\"x\":\"y\"}", "apn", f8, sizeof(f8), &present, err, sizeof(err)) && !present &&
          strcmp(f8, "1234567") == 0, "an absent string key must leave the field untouched");

    printf("[8] the payload as a whole\n");
    static const char *const keys[] = { "duCount", "du1Make", "du1Sfx", "gpsEnable", "apn" };
    const size_t nk = sizeof(keys) / sizeof(keys[0]);
    static const char *const report[] = { "HW_ID", "status", "fw" };
    const size_t nr = sizeof(report) / sizeof(report[0]);
    struct { const char *json; bool ok; const char *why; } pl[] = {
        { "{\"duCount\":1,\"du1Make\":\"yenen\"}",              true,  NULL },
        { " { \"duCount\": \"1\" , \"gpsEnable\":  true } ",    true,  NULL },
        { "{\"duCount\" :1}",                                   false, "bad_payload" },  /* readers would miss it */
        { "{\"cmd\":\"getconfig\"}",                            true,  NULL },
        { "{\"apn\":\"a,b}c\\\"d\"}",                           true,  NULL },  /* , } \" inside a string */
        { "{}",                                                 true,  NULL },  /* nothing to set: the caller refuses */
        { "{\"HW_ID\":\"865510083149163\",\"status\":\"config_report\",\"fw\":\"1.0.0\",\"duCount\":1}",
                                                                true,  NULL },  /* a report sent back */
        { "{\"status\":\"a\",\"status\":\"b\"}",            false, "duplicate_key_status" },
        { "{\"duCount\":1,\"du1make\":\"yenen\"}",              false, "unknown_key_du1make" },
        { "{\"duCount\":1,\"duCount\":2}",                      false, "duplicate_key_duCount" },
        { "{\"duCount\":{\"n\":1}}",                            false, "plain value" },
        { "{\"duCount\":[1]}",                                  false, "plain value" },
        { "{\"cmd\":\"restart\",\"cmd\":\"factory_reset\"}",    false, "duplicate_key_cmd" },
        { "{\"apn\"}",                                          false, "bad_payload" },
        { "{\"apn\":\"x\" \"duCount\":1}",                      false, "bad_payload" },
    };
    for (size_t i = 0; i < sizeof(pl) / sizeof(pl[0]); i++) {
        err[0] = '\0';
        bool r = setcfg_check_members(pl[i].json, keys, nk, report, nr, err, sizeof(err));
        CHECK(r == pl[i].ok && (r || strstr(err, pl[i].why)),
              "%s -> %s ('%s')", pl[i].json, r ? "accepted" : "refused", err);
    }
    CHECK(!payload_is_one_json_object("{\"duCount\":1,\"du1Make\":\"yen"),
          "a push cut short must not count as one complete object");
    CHECK(!payload_is_one_json_object("{\"duCount\":1}{\"duCount\":2}"), "two objects must be refused");
    CHECK(!payload_is_one_json_object("https://example.com/fw.bin"), "a URL is not a setconfig");

    printf("[9] factory_reset: confirm = this gateway's IMEI, unsent readings only with force\n");
    const char *imei = "866738089009581";
    struct { const char *confirm; int unsent; bool force; const char *want; } fr[] = {
        { "866738089009581",    0, false, NULL },                  /* goes ahead */
        { "866738089009581",    0, true,  NULL },
        { "866738089009581",  250, true,  NULL },                  /* forced: discards 250 */
        { "866738089009581",  250, false, "unsent_readings_250" },
        { "866738089009581",    1, false, "unsent_readings_1" },
        { NULL,                 0, false, "confirm_required" },    /* no confirm key */
        { "",                   0, true,  "confirm_required" },    /* empty never matches */
        { "865510083149163",    0, false, "confirm_mismatch" },    /* another unit's IMEI */
        { "86673808900958101",  0, false, "confirm_mismatch" },    /* a nozzle HW_ID, not the IMEI */
        { "866738089009581 ",   0, false, "confirm_mismatch" },    /* trailing space */
        { "865510083149163",  250, true,  "confirm_mismatch" },    /* force never excuses the IMEI */
    };
    for (size_t i = 0; i < sizeof(fr) / sizeof(fr[0]); i++) {
        char why[40] = "";
        const char *r = factory_reset_refusal(fr[i].confirm, imei, fr[i].unsent, fr[i].force,
                                              why, sizeof(why));
        CHECK(fr[i].want ? (r && strcmp(r, fr[i].want) == 0) : r == NULL,
              "confirm '%s' unsent %d force %d -> '%s' (want '%s')", fr[i].confirm ? fr[i].confirm : "(none)",
              fr[i].unsent, fr[i].force, r ? r : "(go ahead)", fr[i].want ? fr[i].want : "(go ahead)");
    }
    char why[40] = "";
    CHECK(factory_reset_refusal("", "", 0, false, why, sizeof(why)) != NULL,
          "an empty IMEI must not let an empty confirm through");
    CHECK(factory_reset_refusal("866738089009581", NULL, 0, false, why, sizeof(why)) != NULL,
          "no IMEI must refuse, not crash");

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
