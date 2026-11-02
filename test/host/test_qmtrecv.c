/* Where an inbound MQTT message starts and ends in the modem's
 * "+QMTRECV: 0,<id>,"<topic>","<payload>"" line: qmtrecv_payload_start() and
 * qmtrecv_payload_end(), EXTRACTED from src/mqtt.c by extract.py. Every
 * command the gateway takes — FOTA, setconfig, lock, rsp — is cut out of the
 * UART stream by these two.
 *
 * What it pins: a JSON payload is taken whole, quotes and braces inside its
 * strings included, also when a space or line break comes before its '{' (a
 * hand-typed push; it used to be cut at its first inner quote); a pretty-
 * printed one keeps its line breaks; plain text ends at its closing quote;
 * anything still arriving reads as incomplete, never as a shorter message.
 * payload_of() repeats parse_qmtrecv()'s few lines of boundary arithmetic;
 * the expected payloads are written out by hand. */
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "extracted_qmtrecv.inc"

/* ── test scaffolding ──────────────────────────────────────────────────── */
static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do {                                        \
        checks++;                                                          \
        if (!(cond)) { fails++;                                            \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__); }                  \
    } while (0)

/* The payload parse_qmtrecv() would stash for the first +QMTRECV in `urc`,
 * or NULL while it is incomplete. */
static const char *payload_of(const char *urc, char *out, size_t out_size) {
    const char *topic;
    size_t      topic_len;
    const char *p = qmtrecv_payload_start(urc, &topic, &topic_len);
    if (!p) return NULL;
    const bool  quoted = (*p == '"');
    const char *start  = quoted ? p + 1 : p;
    const char *stop   = qmtrecv_payload_end(p);
    if (!stop) return NULL;
    const char *end = quoted ? stop - 1 : stop;
    size_t n = (size_t)(end - start);
    if (n > 0 && start[n - 1] == '\r') n--;
    snprintf(out, out_size, "%.*s", (int)n, start);
    return out;
}

static bool eq(const char *got, const char *want) {
    if (!got) { printf("  ...   got nothing (incomplete)\n"); return false; }
    if (strcmp(got, want) != 0) { printf("  ...   got [%s]\n", got); return false; }
    return true;
}

int main(void) {
    char buf[512];

    /* [1] Compact JSON, as the cloud sends it — and its topic. */
    const char *u1 = "+QMTRECV: 0,1,\"fs/865510083149163/update\",\"{\"url\":\"https://h/a.bin\"}\"\r\n";
    CHECK(eq(payload_of(u1, buf, sizeof(buf)), "{\"url\":\"https://h/a.bin\"}"), "[1] compact JSON");
    const char *topic = NULL;
    size_t      topic_len = 0;
    qmtrecv_payload_start(u1, &topic, &topic_len);
    CHECK(topic && topic_len == 25 && strncmp(topic, "fs/865510083149163/update", 25) == 0,
          "[1] topic");

    /* [2] A space before the '{' — the whole object, not " {". */
    CHECK(eq(payload_of("+QMTRECV: 0,1,\"t\",\" {\"url\":\"https://h/a.bin\",\"HW_ID\":\"865510083149163\"}\"\r\n",
                        buf, sizeof(buf)),
             " {\"url\":\"https://h/a.bin\",\"HW_ID\":\"865510083149163\"}"), "[2] space before JSON");

    /* [3] A line break before it, and a pretty-printed body. */
    CHECK(eq(payload_of("+QMTRECV: 0,1,\"t\",\"\r\n{\r\n  \"url\": \"https://h/a.bin\"\r\n}\r\n\"\r\n",
                        buf, sizeof(buf)),
             "\r\n{\r\n  \"url\": \"https://h/a.bin\"\r\n}\r\n"), "[3] line break before JSON");
    CHECK(eq(payload_of("+QMTRECV: 0,1,\"t\",\"{\r\n  \"cmd\": \"getconfig\"\r\n}\r\n\"\r\n", buf, sizeof(buf)),
             "{\r\n  \"cmd\": \"getconfig\"\r\n}\r\n"), "[3] pretty-printed JSON");

    /* [4] Quotes and braces inside a string do not end the object. */
    CHECK(eq(payload_of("+QMTRECV: 0,1,\"t\",\"{\"note\":\"a}b\\\"c{\",\"url\":\"x\"}\"\r\n", buf, sizeof(buf)),
             "{\"note\":\"a}b\\\"c{\",\"url\":\"x\"}"), "[4] braces and quotes in a string");

    /* [5] Plain text ends at its closing quote, leading spaces kept. */
    CHECK(eq(payload_of("+QMTRECV: 0,1,\"fs/1/lock\",\"unlock\"\r\n", buf, sizeof(buf)), "unlock"),
          "[5] plain text");
    CHECK(eq(payload_of("+QMTRECV: 0,1,\"t\",\"  lock\"\r\n", buf, sizeof(buf)), "  lock"),
          "[5] plain text after spaces");

    /* [6] Still arriving: incomplete, never a shorter message. */
    static const char *PARTIAL[] = {
        "+QMTRECV: 0,1,\"fs/1/upd",                                  /* topic cut  */
        "+QMTRECV: 0,1,\"t\",\"{\"url\":\"https://h/a",              /* JSON open  */
        "+QMTRECV: 0,1,\"t\",\" {\"url\":\"https://h/a.bin\"}",      /* no closing wrapper quote yet */
        "+QMTRECV: 0,1,\"t\",\"   ",                                 /* only spaces so far */
        "+QMTRECV: 0,1,\"t\",\"unlo",                                /* text open  */
    };
    for (size_t i = 0; i < sizeof(PARTIAL) / sizeof(PARTIAL[0]); i++)
        CHECK(payload_of(PARTIAL[i], buf, sizeof(buf)) == NULL, "[6] taken while incomplete: %s", PARTIAL[i]);

    /* [7] Two messages back to back: the second is found after the first. */
    const char *two = "+QMTRECV: 0,1,\"t\",\"{\"n\":1}\"\r\n+QMTRECV: 0,2,\"t\",\" {\"n\":2}\"\r\n";
    const char *p1  = qmtrecv_payload_start(two, NULL, NULL);
    const char *e1  = p1 ? qmtrecv_payload_end(p1) : NULL;
    CHECK(e1 && eq(payload_of(e1, buf, sizeof(buf)), " {\"n\":2}"), "[7] second message after the first");

    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
