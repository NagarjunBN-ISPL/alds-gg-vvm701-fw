/* Exercises the price parser, nozzle-label mapping and ack builder that were
 * EXTRACTED from src/main.c (see extract.py) — the production text, compiled
 * natively. */
#include "shim.h"
#include "extracted.inc"

int g_log_quiet = 1;
DeviceConfig g_cfg;
char g_hw_du[DU_MAX_DISPENSERS][24];

static char last_topic[256], last_payload[512];
static int  publishes;

bool rtc_timestamp(char *buf, size_t size) {
    snprintf(buf, size, "2026-09-02 11:04:19");
    return true;
}
bool mqtt_publish(const char *topic, const char *payload) {
    snprintf(last_topic, sizeof(last_topic), "%s", topic);
    snprintf(last_payload, sizeof(last_payload), "%s", payload);
    publishes++;
    return true;
}
int64_t esp_timer_get_time(void) { return 1000000; }

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

/* ── price parser ──────────────────────────────────────────────────────── */
static void t_price(void) {
    printf("[1] price parsing\n");
    struct { const char *json; bool ok; uint16_t want; } v[] = {
        /* the shapes a cloud actually sends */
        {"{\"rsp\":65.32}",            true,  6532},
        {"{\"rsp\":\"65.32\"}",        true,  6532},
        {"{\"rsp\": 65.32 }",          true,  6532},
        {"{\"rsp\":65}",               true,  6500},
        {"{\"rsp\":\"65\"}",           true,  6500},
        {"{\"rsp\":65.3}",             true,  6530},
        {"{\"rsp\":0.01}",             true,     1},
        {"{\"rsp\":99.99}",            true,  9999},
        {"{\"rsp\":9.9}",              true,   990},
        {"{\"nz\":1,\"rsp\":82.50}",   true,  8250},
        {"{\"rsp\":82.50,\"nz\":2}",   true,  8250},
        /* float serialisation noise must not shift the price by a paisa */
        {"{\"rsp\":65.319}",           true,  6532},
        {"{\"rsp\":65.321}",           true,  6532},
        {"{\"rsp\":65.325}",           true,  6533},
        /* refusals */
        {"{\"rsp\":100.00}",           false, 0},
        {"{\"rsp\":0}",                false, 0},
        {"{\"rsp\":0.00}",             false, 0},
        {"{\"rsp\":-5}",               false, 0},
        {"{\"rsp\":\"abc\"}",          false, 0},
        {"{\"rsp\":65.}",              false, 0},
        {"{\"rsp\":65.3211}",          false, 0},
        {"{\"rsp\":65.32x}",           false, 0},
        {"{\"rsp\":\"65.32}",          false, 0},   /* unterminated string    */
        {"{\"price\":65.32}",          false, 0},   /* wrong key              */
        {"{}",                         false, 0},
        {"",                           false, 0},
        {"{\"rsp\":1000}",             false, 0},
        {"{\"rsp\":6532}",             false, 0},   /* paise, not rupees      */
    };
    for (unsigned i = 0; i < sizeof(v)/sizeof(v[0]); i++) {
        uint16_t got = 0xFFFF;
        bool ok = json_get_price_x100(v[i].json, "rsp", &got);
        CHECK(ok == v[i].ok && (!ok || got == v[i].want),
              "%-26s -> ok=%d val=%u (want ok=%d val=%u)",
              v[i].json, ok, ok ? got : 0, v[i].ok, v[i].want);
        if (!v[i].ok)
            CHECK(got == 0xFFFF, "%s: rejected but *out was written", v[i].json);
    }
}

/* ── nozzle labels ─────────────────────────────────────────────────────── */
static void t_labels(void) {
    printf("[2] nozzle labels vs du_task numbering\n");
    /* No suffixes: entry i (0-based) IS dispenser i+1 and owns labels 2i+1 /
     * 2i+2 whatever its make — exactly du_task's `(n-1)*2 + side + 1`. */
    memset(&g_cfg, 0, sizeof(g_cfg));
    struct { int unit; int side; int want; } v[] = {
        {0, 0, 1}, {0, 1, 2}, {1, 0, 3}, {1, 1, 4}, {2, 0, 5}, {4, 1, 10}, {5, 0, 11}, {5, 1, 12},
    };
    for (unsigned i = 0; i < sizeof(v)/sizeof(v[0]); i++) {
        int got = du_nozzle_label(v[i].unit, v[i].side);
        CHECK(got == v[i].want, "unit=%d side=%d -> %d (want %d)",
              v[i].unit, v[i].side, got, v[i].want);
    }
    /* With suffixes the label follows the dispenser NUMBER, not the position:
     * the first entry naming dispenser 03 publishes nozzles 5 and 6. */
    snprintf(g_cfg.du_sfx[0], sizeof(g_cfg.du_sfx[0]), "03");
    snprintf(g_cfg.du_sfx[1], sizeof(g_cfg.du_sfx[1]), "01");
    CHECK(du_nozzle_label(0, 0) == 5 && du_nozzle_label(0, 1) == 6, "entry 1 = DU03 must label 5/6");
    CHECK(du_nozzle_label(1, 0) == 1 && du_nozzle_label(1, 1) == 2, "entry 2 = DU01 must label 1/2");
    memset(&g_cfg, 0, sizeof(g_cfg));
    /* The "nz":1/2 shorthand may never collide with a different nozzle's global
     * label — the property the command parser's mapping rests on. */
    for (int u = 0; u < DU_MAX_DISPENSERS; u++) {
        int la = du_nozzle_label(u, 0), lb = du_nozzle_label(u, 1);
        bool shadowed = (la == 1 || la == 2 || lb == 1 || lb == 2);
        bool own      = (u == 0 && la == 1 && lb == 2);
        CHECK(!shadowed || own, "unit=%d: global label collides with the 1/2 shorthand", u);
    }
}

/* ── ack payloads ──────────────────────────────────────────────────────── */
static bool json_wellformed(const char *s) {
    int depth = 0; bool instr = false, esc = false;
    if (!s || *s != '{') return false;
    for (const char *p = s; *p; p++) {
        if (instr) {
            if (esc)             esc = false;
            else if (*p == '\\') esc = true;
            else if (*p == '"')  instr = false;
            continue;
        }
        if (*p == '"') instr = true;
        else if (*p == '{') depth++;
        else if (*p == '}') { if (--depth == 0 && p[1] != '\0') return false; }
    }
    return depth == 0 && !instr;
}

static void t_acks(void) {
    printf("[3] ack payloads\n");
    snprintf(g_hw_du[0], sizeof(g_hw_du[0]), "86098705151272851");

    /* both nozzles confirmed */
    {
        PendingRsp_t c; memset(&c, 0, sizeof(c));
        c.price_x100 = 6532; c.req = 0x03;
        c.ok[0] = c.ok[1] = true; c.got[0] = c.got[1] = 6532;
        c.result[0] = c.result[1] = "RSP_updated";
        snprintf(c.resp_topic, sizeof(c.resp_topic), "suryalpg/setrsp/imei/x/out");
        publishes = 0;
        setrsp_publish_ack(&c, 0);
        printf("       %s\n", last_payload);
        CHECK(publishes == 1, "published %d times, want exactly 1", publishes);
        CHECK(json_wellformed(last_payload), "ack is not one well-formed object");
        CHECK(strstr(last_payload, "\"status\":\"RSP_updated\"") != NULL, "missing RSP_updated status");
        CHECK(strstr(last_payload, "\"rsp\":\"65.32\"") != NULL, "missing 65.32");
        CHECK(strstr(last_payload, "\"1\":{\"result\":\"RSP_updated\",\"rsp\":\"65.32\"}") != NULL, "nz1 detail wrong");
        CHECK(strstr(last_payload, "\"2\":{\"result\":\"RSP_updated\",\"rsp\":\"65.32\"}") != NULL, "nz2 detail wrong");
        CHECK(strcmp(last_topic, "suryalpg/setrsp/imei/x/out") == 0, "wrong topic %s", last_topic);
    }
    /* mixed outcome: one confirmed, one busy */
    {
        PendingRsp_t c; memset(&c, 0, sizeof(c));
        c.price_x100 = 8250; c.req = 0x03;
        c.ok[0] = true;  c.got[0] = 8250; c.result[0] = "RSP_updated";
        c.ok[1] = false; c.got[1] = 0;    c.result[1] = "nozzle_busy";
        snprintf(c.resp_topic, sizeof(c.resp_topic), "t/out");
        setrsp_publish_ack(&c, 0);
        printf("       %s\n", last_payload);
        CHECK(json_wellformed(last_payload), "mixed ack malformed");
        CHECK(strstr(last_payload, "\"status\":\"RSP_update_failed\"") != NULL,
              "a partly-failed update must not report success");
        CHECK(strstr(last_payload, "\"2\":{\"result\":\"nozzle_busy\"}") != NULL, "nz2 reason missing");
        CHECK(strstr(last_payload, "\"2\":{\"result\":\"nozzle_busy\",\"rsp\"") == NULL,
              "a failed nozzle must not carry a price");
    }
    /* single-nozzle request mentions only that nozzle */
    {
        PendingRsp_t c; memset(&c, 0, sizeof(c));
        c.price_x100 = 100; c.req = 0x02;
        c.ok[1] = false; c.got[1] = 6500; c.result[1] = "verify_failed";
        snprintf(c.resp_topic, sizeof(c.resp_topic), "t/out");
        setrsp_publish_ack(&c, 0);
        printf("       %s\n", last_payload);
        CHECK(json_wellformed(last_payload), "single-nozzle ack malformed");
        CHECK(strstr(last_payload, "\"1\":") == NULL, "reported a nozzle that was not requested");
        CHECK(strstr(last_payload, "\"rsp\":\"1.00\"") != NULL, "1.00 formatted wrong");
        CHECK(strstr(last_payload, "\"result\":\"verify_failed\",\"rsp\":\"65.00\"") != NULL,
              "verify_failed must report the price the pump holds");
    }
    /* never-attempted side still yields valid JSON */
    {
        PendingRsp_t c; memset(&c, 0, sizeof(c));
        c.price_x100 = 9999; c.req = 0x03;
        setrsp_publish_ack(&c, 0);
        CHECK(json_wellformed(last_payload), "ack with no results malformed: %s", last_payload);
        CHECK(strstr(last_payload, "unknown") != NULL, "expected the unknown fallback");
    }
    /* rejection shape */
    {
        publishes = 0;
        setrsp_reject("t/out", g_hw_du[0], "bad_rsp");
        printf("       %s\n", last_payload);
        CHECK(publishes == 1 && json_wellformed(last_payload), "reject ack malformed");
        CHECK(strstr(last_payload, "\"status\":\"RSP_update_rejected\"") != NULL, "missing rejected status");
        CHECK(strstr(last_payload, "\"reason\":\"bad_rsp\"") != NULL, "missing reason");
    }
    /* payload stays far under the 1400-byte modem ceiling */
    {
        PendingRsp_t c; memset(&c, 0, sizeof(c));
        c.price_x100 = 9999; c.req = 0x03;
        c.got[0] = c.got[1] = 9999;
        c.result[0] = c.result[1] = "verify_failed";
        snprintf(g_hw_du[0], sizeof(g_hw_du[0]), "86098705151272899");
        setrsp_publish_ack(&c, 0);
        CHECK(strlen(last_payload) < 300, "ack is %zu bytes", strlen(last_payload));
        CHECK(strlen(last_payload) < MQTT_MODEM_PAYLOAD_SAFE_MAX, "ack over the modem ceiling");
        printf("       worst-case ack %zu bytes\n", strlen(last_payload));
    }
}

int main(void) {
    printf("== main.c setrsp logic — extracted from the live source ==\n");
    t_price();
    t_labels();
    t_acks();
    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
