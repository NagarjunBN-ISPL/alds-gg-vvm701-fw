/* Fuzzes the JSON helpers EXTRACTED from src/main.c under ASan + UBSan.
 * These parse untrusted MQTT payloads, so they are the firmware's widest
 * remote-input surface. Every input buffer is heap-allocated to its exact
 * length so a one-byte overread is a hard failure, not a near miss. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "config.h"
#include "esp_log.h"

int g_log_quiet = 1;
static const char *TAG __attribute__((unused)) = "main";   /* used by ESP_LOG* in the extract */

#include "extracted_json.inc"

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        if (fails < 20) printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static uint32_t rng_s = 0xBEEF0001;
static uint32_t rnd(void) {
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 17; rng_s ^= rng_s << 5;
    return rng_s;
}

/* Run every parser over one payload. The point is not the answers — it is that
 * none of them read out of bounds or trip UB on arbitrary input. */
static void run_all(const char *json) {
    char     sbuf[8];        /* deliberately tiny: forces the truncation path */
    char     sbig[128];
    uint16_t price = 0;
    int      i;
    bool     b;

    static const char *keys[] = {
        "rsp", "nz", "cmd", "duCount", "mqttHost", "mqttPass", "apn",
        "gpsEnable", "mqttTls", "", "\"", "a", "du1Sfx", "sfxDI", "du2Sfx",
        "aVeryLongKeyNameThatExceedsTheInternalSearchBufferUsedByTheseHelpers",
    };
    for (unsigned k = 0; k < sizeof(keys)/sizeof(keys[0]); k++) {
        i = json_get_int(json, keys[k], -1);            (void)i;
        b = json_get_bool(json, keys[k], false);        (void)b;
        b = json_has_key(json, keys[k]);                (void)b;
        if (json_get_str(json, keys[k], sbuf, sizeof(sbuf)))
            CHECK(strlen(sbuf) < sizeof(sbuf), "json_get_str overran the small buffer");
        if (json_get_str(json, keys[k], sbig, sizeof(sbig)))
            CHECK(strlen(sbig) < sizeof(sbig), "json_get_str overran the large buffer");
        if (json_get_price_x100(json, keys[k], &price))
            CHECK(price >= 1 && price <= 9999, "price %u escaped its range", price);
    }
    b = setcfg_has_secret(json);            (void)b;
    b = payload_is_one_json_object(json);   (void)b;
}

/* Exact-length allocation so ASan brackets the buffer. */
static void run_exact(const char *src, size_t n) {
    char *p = malloc(n + 1);
    memcpy(p, src, n);
    p[n] = '\0';
    run_all(p);
    free(p);
}

int main(int argc, char **argv) {
    int iters = (argc > 1) ? atoi(argv[1]) : 200000;
    printf("== JSON helper fuzzing (extracted from main.c), %d cases ==\n", iters);

    /* 1. Adversarial hand-written payloads. */
    printf("[1] adversarial payloads\n");
    {
        static const char *cases[] = {
            "", "{", "}", "{}", "[]", "null", "\"", "\"\"", "\\", "{\\",
            "{\"rsp\":", "{\"rsp\":}", "{\"rsp\":\"", "{\"rsp\":\"unterminated",
            "{\"rsp\":65.32", "{\"rsp\":------}", "{\"rsp\":+65.32}",
            "{\"rsp\":99999999999999999999999999}", "{\"rsp\":-99999999999}",
            "{\"nz\":\"\\\"\"}", "{\"cmd\":\"\\\\\"}", "{\"a\":\"\\u0000\"}",
            "{\"mqttHost\":\"\"}", "{\"mqttPass\":\"" "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" "\"}",
            "{\"rsp\":1e400}", "{\"rsp\":0x41}", "{\"rsp\":.5}", "{\"rsp\":5.}",
            "{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{", "}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}",
            "{\"a\":{\"b\":{\"c\":{\"d\":{\"e\":1}}}}}",
            "{\"rsp\":65.32}{\"rsp\":70.00}",
            "{\"rsp\":65.32}   ", "   {\"rsp\":65.32}",
            "{\"rsp\"  :  65.32}", "{\"rsp\":\t65.32}",
        };
        for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); i++)
            run_exact(cases[i], strlen(cases[i]));

        /* Deep nesting and a very long string, sized past every internal buffer. */
        {
            size_t n = 4096;
            char *big = malloc(n + 1);
            memset(big, '{', n); big[n] = '\0';
            run_exact(big, n);
            memset(big, '"', n);
            run_exact(big, n);
            memset(big, 'a', n);
            memcpy(big, "{\"rsp\":\"", 8);
            run_exact(big, n);
            free(big);
        }
    }

    /* 2. Random bytes, and random bytes biased toward JSON punctuation. */
    printf("[2] %d random payloads\n", iters);
    {
        char buf[512];
        for (int it = 0; it < iters; it++) {
            size_t n = rnd() % sizeof(buf);
            bool structured = (it % 2) == 0;
            for (size_t i = 0; i < n; i++) {
                if (structured) {
                    static const char alphabet[] = "{}[]\":,\\ \t\n0123456789.rspnzcmdtruefalse";
                    buf[i] = alphabet[rnd() % (sizeof(alphabet) - 1)];
                } else {
                    unsigned char c = (unsigned char)(rnd() & 0xFF);
                    buf[i] = (char)(c ? c : 'x');   /* no early NUL: full length */
                }
            }
            run_exact(buf, n);
        }
    }

    /* 3. Truncation sweep: every prefix of a valid payload. */
    printf("[3] truncation sweep\n");
    {
        const char *full = "{\"rsp\":65.32,\"nz\":2,\"cmd\":\"restart\",\"mqttHost\":\"a.b.c\",\"mqttTls\":true}";
        for (size_t n = 0; n <= strlen(full); n++) run_exact(full, n);
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
