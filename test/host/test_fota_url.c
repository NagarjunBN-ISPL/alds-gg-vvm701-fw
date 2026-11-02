/* The FOTA link and HW_ID checks, EXTRACTED from the real sources by
 * extract.py: sanitize_url(), url_normalize() and prepare_url() from
 * src/fota.c, hw_id_is_mine() from src/main.c.
 *
 * What it pins: the link the modem gets is trimmed, has no tab or line break,
 * carries every inner space as %20 and starts https:// in lower case; a link
 * that is not https://, or no longer fits once encoded, is refused with the
 * reason the cloud reads; a push's HW_ID matches this gateway's bare IMEI or
 * one of its equipment ids and nothing else. Expected strings are written out
 * by hand, not computed by the code under test. */
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdbool.h>
#include <ctype.h>
#include "fota.h"

static char g_imei[20] = "865510083149163";

#include "extracted_fota_url.inc"
#include "extracted_hw_id.inc"

/* ── test scaffolding ──────────────────────────────────────────────────── */
static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do {                                        \
        checks++;                                                          \
        if (!(cond)) { fails++;                                            \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__); }                  \
    } while (0)

/* prepare_url() into a buffer of `size`: true when it refuses with
 * `want_reason`, or (want_reason NULL) takes the link as exactly `want_url`. */
static bool prep(const char *in, size_t size, const char *want_reason, const char *want_url) {
    char out[2048];
    const char *r = prepare_url(out, size, in);
    if (want_reason) return r && strcmp(r, want_reason) == 0;
    if (r) { printf("  ...   refused (%s): [%s]\n", r, out); return false; }
    if (strcmp(out, want_url) != 0) { printf("  ...   got [%s]\n", out); return false; }
    return true;
}

int main(void) {
    const size_t SZ = FOTA_URL_MAX + 1;   /* the buffer fota.c runs it in */

    /* [1] The cloud's own presigned link goes through untouched. */
    const char *s3 = "https://devfota-fuelsense.s3.ap-south-1.amazonaws.com/fota/build/"
                     "alds-gg-vvm701-fw%207.bin?X-Amz-Algorithm=AWS4-HMAC-SHA256"
                     "&X-Amz-Expires=1200&X-Amz-SignedHeaders=host&X-Amz-Signature=0f1e";
    CHECK(prep(s3, SZ, NULL, s3), "[1] presigned link changed");

    /* [2] Quotes and whitespace at either end go; the scheme goes to lower case. */
    CHECK(prep("  \"HTTPS://h.example/fw.bin\"\r\n", SZ, NULL, "https://h.example/fw.bin"),
          "[2] trim + scheme");

    /* [3] A space inside the link is sent as %20 — the cloud's file naming. */
    CHECK(prep("https://h/fota/alds-gg-vvm701-fw 7.bin?X-Amz-Date=1", SZ, NULL,
               "https://h/fota/alds-gg-vvm701-fw%207.bin?X-Amz-Date=1"), "[3] one space");
    CHECK(prep("https://h/a  b c.bin", SZ, NULL, "https://h/a%20%20b%20c.bin"),
          "[3] adjacent and several spaces");

    /* [4] Tabs and line breaks inside it are dropped (a link wrapped across lines). */
    CHECK(prep("https://h/fw\r\n-1.0.1\t.bin", SZ, NULL, "https://h/fw-1.0.1.bin"),
          "[4] inner tab / CR / LF");
    CHECK(prep("ht\ntps://h/fw.bin", SZ, NULL, "https://h/fw.bin"), "[4] line break in the scheme");

    /* [5] Not https:// — refused with the reason the cloud reads. */
    static const char *NOT_HTTPS[] = {
        "http://h/fw.bin", "ftp://h/fw.bin", "https:/h/fw.bin", "h/fw.bin", "", "   \r\n",
        "{\"url\":\"https://h/fw.bin\"}",            /* JSON taken for a bare link */
    };
    for (size_t i = 0; i < sizeof(NOT_HTTPS) / sizeof(NOT_HTTPS[0]); i++)
        CHECK(prep(NOT_HTTPS[i], SZ, "non_https_url", NULL), "[5] taken: [%s]", NOT_HTTPS[i]);
    CHECK(prep(NULL, SZ, "non_https_url", NULL), "[5] NULL");

    /* [6] The encoded link must fit: 13 chars + one space = 15 + NUL. */
    CHECK(prep("https://h/a b", 16, NULL, "https://h/a%20b"), "[6] exact fit refused");
    CHECK(prep("https://h/a b", 15, "url_too_long", NULL), "[6] one byte short taken");

    /* [7] At full size: a 1535-char link fits as it is; with a space in it, it
     *     no longer does; two characters shorter, it does again. */
    static char big[FOTA_URL_MAX + 64], want[FOTA_URL_MAX + 64];
    memcpy(big, "https://h/", 10);
    memset(big + 10, 'a', FOTA_URL_MAX - 10);
    big[FOTA_URL_MAX] = '\0';
    CHECK(prep(big, SZ, NULL, big), "[7] full-size link refused");
    big[700] = ' ';
    CHECK(prep(big, SZ, "url_too_long", NULL), "[7] full-size link with a space taken");
    big[FOTA_URL_MAX - 2] = '\0';
    memcpy(want, big, 700);
    memcpy(want + 700, "%20", 3);
    strcpy(want + 703, big + 701);
    CHECK(strlen(want) == FOTA_URL_MAX && prep(big, SZ, NULL, want), "[7] encoded to the last byte");

    /* [8] HW_ID: this gateway's IMEI, or the IMEI and a 2-digit suffix. */
    static const struct { const char *id; bool mine; } IDS[] = {
        { "865510083149163",    true  },
        { "86551008314916301",  true  },
        { "86551008314916399",  true  },
        { "866738089009581",    false },   /* another unit       */
        { "86551008314916",     false },   /* a prefix of ours   */
        { "8655100831491630",   false },   /* one digit more     */
        { "865510083149163001", false },   /* three digits more  */
        { "86551008314916301x", false },
        { "865510083149163ab",  false },
        { "",                   false },
    };
    for (size_t i = 0; i < sizeof(IDS) / sizeof(IDS[0]); i++)
        CHECK(hw_id_is_mine(IDS[i].id) == IDS[i].mine, "[8] HW_ID [%s] read as %s",
              IDS[i].id, IDS[i].mine ? "another unit's" : "this unit's");

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
