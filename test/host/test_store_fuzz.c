/* Runs the REAL src/store.c against a corruptible filesystem, under ASan +
 * UBSan. Question: can anything littlefs hands back — a truncated record, a
 * garbage cursor, a full partition, a missing file, binary junk in the log —
 * crash the queue or make it hand a bad buffer to the publisher? */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>

#include "config.h"
#include "store.h"
#include "esp_littlefs.h"

int g_log_quiet = 1;
int g_wdt_feeds = 0;
int esp_task_wdt_status(void *h) { (void)h; return 0; }
int esp_task_wdt_reset(void) { g_wdt_feeds++; return 0; }

void sim_fs_setroot(const char *root);

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        if (fails < 25) printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static uint32_t rng_s = 0xC0FFEE11;
static uint32_t rnd(void) {
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 17; rng_s ^= rng_s << 5;
    return rng_s;
}

#define ROOT "/tmp/alds_storetest"

/* glibc marks system() warn_unused_result, and a (void) cast does not silence
 * it — assign instead. Failures here are not interesting: the next fopen tells
 * us soon enough. */
static void sh(const char *cmd) { int r = system(cmd); (void)r; }

static void wipe(void) {
    sh("rm -rf " ROOT " && mkdir -p " ROOT);
}

/* Write raw bytes straight into the queue files, behind store.c's back — this
 * is what a power cut mid-append or a bit-rotted block looks like to it. */
static void put_file(const char *name, const void *d, size_t n) {
    char p[512];
    snprintf(p, sizeof(p), "%s/%s", ROOT, name);
    FILE *f = fopen(p, "wb");
    if (!f) return;
    if (n) fwrite(d, 1, n, f);
    fclose(f);
}

/* A peek buffer exactly the size the firmware uses, so a truncating or
 * overlong record shows up as it would on the device. ASan brackets it. */
static void drain_everything(const char *ctx) {
    char *buf = malloc(DU_PAYLOAD_MAX);
    for (int guard = 0; guard < 5000; guard++) {
        uint32_t tok = 0;
        if (!store_peek(buf, DU_PAYLOAD_MAX, &tok)) break;
        size_t len = strlen(buf);
        CHECK(len < DU_PAYLOAD_MAX, "%s: peeked record is not terminated in buffer", ctx);
        store_pop(tok);
    }
    free(buf);
}

int main(int argc, char **argv) {
    int iters = (argc > 1) ? atoi(argv[1]) : 4000;
    sim_fs_setroot(ROOT);
    printf("== littlefs fault injection against the real store.c (%d cases) ==\n", iters);

    char rec[STORE_REC_MAX + 64];

    /* 1. Mount failure must disable the queue, not crash callers. */
    printf("[1] mount failure\n");
    {
        wipe();
        simfs_mount_fails(1);
        CHECK(store_init() == false, "mount failure reported as success");
        CHECK(store_ready() == false, "ready after a failed mount");
        /* every entry point must be a safe no-op now */
        CHECK(store_enqueue("{\"a\":1}") == false, "enqueued into an unmounted store");
        char b[DU_PAYLOAD_MAX]; uint32_t t = 0;
        CHECK(store_peek(b, sizeof(b), &t) == false, "peeked an unmounted store");
        CHECK(store_pop(0) == false, "popped an unmounted store");
        CHECK(store_purge() == false, "purged an unmounted store");
        CHECK(store_has_records() == false, "has_records on an unmounted store");
        (void)store_pending(); (void)store_count(); (void)store_used_bytes();
        simfs_mount_fails(0);
    }

    /* 2. Structural corruption of the on-flash files. */
    printf("[2] corrupt queue files\n");
    {
        struct { const char *name; const char *log; const char *head; } cases[] = {
            {"empty log",          "",                                    "0"},
            {"no trailing newline","{\"a\":1}",                            "0"},
            {"truncated json",     "{\"a\":1}\n{\"b\":",                   "0"},
            {"two objects merged", "{\"a\":1}{\"b\":2}\n",                 "0"},
            {"cursor past EOF",    "{\"a\":1}\n",                          "999999"},
            {"negative cursor",    "{\"a\":1}\n",                          "-5"},
            {"junk cursor",        "{\"a\":1}\n",                          "not-a-number"},
            {"empty cursor file",  "{\"a\":1}\n",                          ""},
            {"cursor mid-record",  "{\"aaaa\":1}\n{\"b\":2}\n",            "3"},
            {"blank lines",        "\n\n\n{\"a\":1}\n\n",                  "0"},
            {"lone newline",       "\n",                                   "0"},
            {"no newline at all",  "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",   "0"},
        };
        for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
            wipe();
            put_file("q.ndjson", cases[i].log, strlen(cases[i].log));
            put_file("q.head",   cases[i].head, strlen(cases[i].head));
            CHECK(store_init(), "%s: init failed", cases[i].name);
            (void)store_count(); (void)store_pending(); (void)store_has_records();
            drain_everything(cases[i].name);
            CHECK(store_enqueue("{\"after\":1}"), "%s: cannot enqueue after recovery", cases[i].name);
            drain_everything(cases[i].name);
        }
    }

    /* 3. A log full of binary junk, including embedded NULs and 0xFF runs. */
    printf("[3] binary junk in the log\n");
    {
        for (int round = 0; round < 200; round++) {
            wipe();
            size_t n = 1 + (rnd() % 4096);
            unsigned char *junk = malloc(n);
            for (size_t i = 0; i < n; i++) {
                uint32_t r = rnd() % 100;
                junk[i] = (r < 10) ? (unsigned char)'\n'
                        : (r < 20) ? 0x00
                        : (r < 30) ? 0xFF
                        : (unsigned char)(rnd() & 0xFF);
            }
            put_file("q.ndjson", junk, n);
            char head[16];
            snprintf(head, sizeof(head), "%d", (int)(rnd() % (n + 64)) - 16);
            put_file("q.head", head, strlen(head));
            free(junk);

            CHECK(store_init(), "binary junk: init failed");
            (void)store_count(); (void)store_used_bytes();
            drain_everything("binary junk");
        }
    }

    /* 4. Oversized records: the enqueue guard and the peek buffer. */
    printf("[4] record size boundaries\n");
    {
        wipe(); CHECK(store_init(), "init");
        memset(rec, 'x', sizeof(rec)); rec[STORE_REC_MAX] = '\0';
        CHECK(store_enqueue(rec) == true, "a record of exactly STORE_REC_MAX was refused");
        rec[STORE_REC_MAX + 1] = '\0';
        memset(rec, 'y', STORE_REC_MAX + 1);
        CHECK(store_enqueue(rec) == false, "an oversized record was accepted");
        CHECK(store_enqueue("") == false, "empty record accepted");
        CHECK(store_enqueue(NULL) == false, "NULL record accepted");
        drain_everything("size boundaries");

        /* A record longer than the buffer, planted directly on flash. */
        wipe();
        {
            size_t n = DU_PAYLOAD_MAX * 2;
            char *big = malloc(n + 2);
            memset(big, 'z', n); big[n] = '\n'; big[n+1] = '\0';
            put_file("q.ndjson", big, n + 1);
            put_file("q.head", "0", 1);
            free(big);
        }
        CHECK(store_init(), "init after planting an overlong record");
        drain_everything("overlong planted record");
    }

    /* 5. A full partition mid-write. */
    printf("[5] partition full\n");
    {
        wipe(); CHECK(store_init(), "init");
        for (int i = 0; i < 50; i++) {
            snprintf(rec, sizeof(rec), "{\"n\":%d}", i);
            store_enqueue(rec);
        }
        simfs_set_capacity(64);                 /* every further write fails */
        for (int i = 0; i < 50; i++) {
            snprintf(rec, sizeof(rec), "{\"full\":%d}", i);
            (void)store_enqueue(rec);            /* must fail, never crash */
        }
        (void)store_count(); (void)store_used_bytes();
        drain_everything("full partition");
        simfs_set_capacity(0);
    }

    /* 6. Files vanishing underneath a live queue. */
    printf("[6] files deleted underneath\n");
    {
        wipe(); CHECK(store_init(), "init");
        for (int i = 0; i < 20; i++) { snprintf(rec, sizeof(rec), "{\"n\":%d}", i); store_enqueue(rec); }
        sh("rm -f " ROOT "/q.ndjson");
        char b[DU_PAYLOAD_MAX]; uint32_t t = 0;
        (void)store_peek(b, sizeof(b), &t);
        (void)store_pop(t);
        (void)store_count();
        CHECK(store_enqueue("{\"recovered\":1}"), "cannot recover after the log vanished");
        sh("rm -f " ROOT "/q.head");
        drain_everything("files deleted");
    }

    /* 7. Randomised operation sequences against a live queue. */
    printf("[7] %d randomised operation sequences\n", iters);
    {
        wipe(); CHECK(store_init(), "init");
        char *buf = malloc(DU_PAYLOAD_MAX);
        uint32_t last_tok = 0;
        for (int it = 0; it < iters; it++) {
            switch (rnd() % 10) {
                case 0: case 1: case 2: case 3: {
                    size_t n = rnd() % (STORE_REC_MAX + 8);
                    for (size_t i = 0; i < n; i++) rec[i] = (char)(0x20 + (rnd() % 0x5f));
                    rec[n] = '\0';
                    (void)store_enqueue(rec);
                    break;
                }
                case 4: case 5:
                    if (store_peek(buf, DU_PAYLOAD_MAX, &last_tok))
                        CHECK(strlen(buf) < DU_PAYLOAD_MAX, "unterminated peek");
                    break;
                case 6: (void)store_pop(last_tok); break;
                case 7: (void)store_pop(rnd()); break;   /* stale/bogus token */
                case 8: (void)store_count(); (void)store_pending();
                        (void)store_has_records(); (void)store_used_bytes(); break;
                default:
                    if ((rnd() % 40) == 0) { (void)store_purge(); }
                    break;
            }
        }
        free(buf);
        drain_everything("random ops");
    }

    sh("rm -rf " ROOT);
    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
