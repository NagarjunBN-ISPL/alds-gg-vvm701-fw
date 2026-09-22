#include "store.h"
#include "config.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "store";

/* store_peek() hands records to a DU_PAYLOAD_MAX buffer via fgets(), which
 * keeps at most size-1 chars. If a record could reach DU_PAYLOAD_MAX it would
 * be silently truncated into invalid JSON. Fail the build instead of shipping
 * a payload the cloud quietly discards. */
_Static_assert(STORE_REC_MAX < DU_PAYLOAD_MAX,
               "STORE_REC_MAX must be < DU_PAYLOAD_MAX or store_peek() truncates records");

#define STORE_BASE   "/store"
#define STORE_PART   "storage"           /* partitions.csv label */
#define STORE_LOG    STORE_BASE "/q.ndjson"
#define STORE_HEAD   STORE_BASE "/q.head"
#define STORE_TMP    STORE_BASE "/q.tmp"
/* Which product's records are on this partition — see claim_the_queue(). */
#define STORE_OWNER  STORE_BASE "/owner"

static bool   s_ready = false;
static long   s_head  = 0;   /* byte offset of the oldest unread record */

/* Two tasks reach this module: du_task spills here when the RAM queue fills,
 * and the main task drains it. Every public entry point takes this lock, so
 * s_head and the log file can never be mutated by both at once — an
 * interleaved append and pop would corrupt the read cursor and either replay
 * or silently discard records.
 *
 * Held only across file operations, never across an MQTT publish: the main
 * task peeks, releases, publishes, then re-takes to pop. */
static SemaphoreHandle_t s_lock = NULL;
static int    s_pending = 0;   /* O(1) record count; see store_pending() */

/* Identifies the record currently at s_head, for the peek/pop token in
 * store.h. Strictly monotonic: bumped once per record that leaves the head
 * (popped or dropped) and once on a queue reset, and NEVER decremented or
 * reused — so a token can only ever match the exact record it was issued for.
 *
 * Deliberately not the byte offset: compact() legitimately moves the head
 * record from offset N to offset 0 without changing WHICH record it is, so an
 * offset-based token would refuse a perfectly valid pop and republish a
 * duplicate every time a compaction landed inside a publish. This counter is
 * untouched by compaction, so it stays exact.
 *
 * It only has to be unique within one session — a peek and its pop are always
 * in the same poll cycle — so it needs no persistence across reboots. */
static uint32_t s_head_seq = 0;

/* Records evicted by drop_oldest_until_fits() since boot.
 *
 * This is permanent sale loss and it was, until this counter existed, entirely
 * invisible to the cloud: du_drop counts records REFUSED at enqueue, not ones
 * that were accepted, buffered, and then discarded to make room. The eviction
 * path only wrote a serial-console warning, so the failure mode a long outage
 * actually produces — the queue filling and shedding its oldest sales — showed
 * up nowhere in telemetry. Cumulative since boot, never reset. */
static uint32_t s_trimmed = 0;

static bool store_lock(void)
{
    if (!s_lock) return true;            /* alloc failed — degrade, don't block */
    return xSemaphoreTake(s_lock, pdMS_TO_TICKS(STORE_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static void store_unlock(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

/* ── head-offset persistence ──────────────────────────────────────────────
 * The read cursor lives in its own tiny file rather than inside the log, so
 * advancing it never rewrites the log itself. Losing it is survivable: the
 * queue replays from 0 and the cloud sees duplicates, which is the safe
 * failure direction (see the peek/pop note in store.h). */
static void head_load(void) {
    s_head = 0;
    FILE *f = fopen(STORE_HEAD, "r");
    if (!f) return;
    long v = 0;
    if (fscanf(f, "%ld", &v) == 1 && v >= 0) s_head = v;
    fclose(f);
}

static void head_save(void) {
    FILE *f = fopen(STORE_HEAD, "w");
    if (!f) { ESP_LOGW(TAG, "head write failed"); return; }
    fprintf(f, "%ld", s_head);
    fclose(f);
}

static long log_size(void) {
    struct stat st;
    if (stat(STORE_LOG, &st) != 0) return 0;
    return (long)st.st_size;
}

/* Empty the queue: both files removed, cursor reset. Called when the last
 * record is consumed, which is the common case after a successful flush —
 * far cheaper than compacting, and it leaves the partition clean. */
static void queue_reset(void) {
    unlink(STORE_LOG);
    unlink(STORE_HEAD);
    s_head = 0;
    s_pending = 0;
    s_head_seq++;   /* whatever lands at the head next is a different record */
}

/* Drop everything before s_head by rewriting the log. Only runs when the file
 * has grown past STORE_COMPACT_AT, so the cost is paid rarely rather than on
 * every pop.
 *
 * ── The s_head == 0 case is a NO-OP, NOT a reset ──────────────────────────
 * This used to read `if (s_head <= 0 || s_head >= size) { queue_reset(); }`,
 * folding two completely different states into one destructive branch:
 *
 *   s_head >= size  every record has been consumed  -> reset is correct
 *   s_head == 0     NOTHING has been consumed yet   -> reset DESTROYS the queue
 *
 * The second case is not hypothetical, it is the normal shape of an outage.
 * While the link is down the main task never pops, so s_head stays 0 while
 * du_task appends; the moment the log passed STORE_COMPACT_AT (256 KB) the
 * next store_enqueue() called compact() and silently deleted every buffered
 * sale — roughly 12 hours of them — at exactly the point the buffer had proven
 * it was needed. Nothing logged an error, and the device kept queueing into a
 * fresh empty file, so from the outside it looked like it was working.
 *
 * With nothing consumed there is by definition nothing to reclaim, so the
 * correct action is to do nothing and let the log grow to STORE_MAX_BYTES,
 * where drop_oldest_until_fits() bounds it. */
static bool compact(void) {
    long size = log_size();
    if (size <= 0)      { queue_reset(); return true; }   /* no log at all       */
    if (s_head >= size) { queue_reset(); return true; }    /* fully drained       */
    if (s_head <= 0)    return true;                       /* nothing to reclaim  */

    FILE *in = fopen(STORE_LOG, "r");
    if (!in) return false;
    if (fseek(in, s_head, SEEK_SET) != 0) { fclose(in); return false; }
    FILE *out = fopen(STORE_TMP, "w");
    if (!out) { fclose(in); return false; }

    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
        /* Feed the watchdog inside the copy loop. Compaction rewrites the whole
         * live log — up to STORE_MAX_BYTES (1 MB) — and littlefs erase/write is
         * slow enough that a full rewrite can run for tens of seconds. It is
         * called from du_task, which IS watchdog-registered, so without this a
         * large compaction could trip the 120 s task WDT and reboot a gateway
         * that was doing exactly what it should. */
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        if (fwrite(buf, 1, n, out) != n) {
            fclose(in); fclose(out); unlink(STORE_TMP);
            ESP_LOGE(TAG, "compact: write failed (partition full?)");
            return false;
        }
    }
    fclose(in);
    fclose(out);

    /* ── Power-fail ordering — the sequence here is load-bearing ───────────
     * Two files change (STORE_LOG, STORE_HEAD) and no reboot can be atomic
     * across both, so the order is chosen so that a power cut at ANY point
     * leaves a state that reads back as a valid SUFFIX of the record stream.
     * Duplicates are acceptable (the cloud dedupes on HW_ID+TS+totalizer);
     * skipping a record is NOT — that is silent sale loss.
     *
     * The old order was `unlink(LOG); rename(TMP, LOG); head_save(0)` and it
     * was unsafe twice over:
     *   - a cut between unlink and rename left NEITHER file — the entire queue
     *     gone, at the end of the exact outage it existed to survive;
     *   - a cut between rename and head_save(0) left the NEW compacted log with
     *     the OLD byte offset still on disk, so reboot resumed reading partway
     *     into the compacted file and SKIPPED every record before that offset.
     *
     * Correct order, and why each step is safe if power is lost right after it:
     *   1. head_save(0)  — LOG is still the full old file. Offset now says 0,
     *                      so reboot replays the old log from the start:
     *                      duplicates of already-sent records. SAFE.
     *   2. rename(TMP,LOG) — littlefs rename is atomic and replaces the
     *                      destination in one metadata commit (no separate
     *                      unlink), so the log is either the old file or the
     *                      compacted one, never absent. Offset is already 0,
     *                      which is correct for the compacted file. SAFE.
     * There is no window in which a record can be skipped. */
    s_head = 0;
    head_save();                       /* step 1 — must precede the rename */
    if (rename(STORE_TMP, STORE_LOG) != 0) {   /* step 2 — atomic replace, no unlink */
        /* Old log still intact; offset 0 just means the next read replays it
         * from the start (harmless duplicates). Leave TMP for the next
         * compaction to overwrite. */
        ESP_LOGE(TAG, "compact: rename failed — old log kept, will replay from head 0");
        return false;
    }
    /* NOTE: s_head_seq is deliberately NOT bumped. Compaction removes only
     * already-CONSUMED bytes (everything before s_head) and rebases the rest to
     * offset 0 — the first UNconsumed record is byte-identical and still at the
     * head. A peek token taken before this runs must still match after it, or
     * the caller would republish a duplicate for no reason. Only records
     * LEAVING the head (pop, drop, reset) change the token. */
    ESP_LOGI(TAG, "compacted to %ld bytes", log_size());
    return true;
}

/* Drop the oldest records until the log fits under STORE_MAX_BYTES. Losing
 * the OLDEST is the deliberate choice: a queue this full means a very long
 * outage, and the most recent sales are the ones still worth having.
 *
 * Trimmed with HYSTERESIS — down to (STORE_MAX_BYTES - STORE_TRIM_BYTES), not
 * just back under the ceiling. Trimming to exactly the limit means the very
 * next record puts the log over again, so every single enqueue from then on
 * dropped one record and ran a full compact() — a multi-hundred-KB littlefs
 * rewrite per payload, several times a minute, forever. That is slow enough to
 * back up the whole acquisition task and it burns flash for nothing. Freeing a
 * slack block instead makes the trim+compact happen once per STORE_TRIM_BYTES
 * of new data (~1.8 h at two nozzles) rather than once per record. */
static void drop_oldest_until_fits(void) {
    FILE *f = fopen(STORE_LOG, "r");
    if (!f) return;
    if (fseek(f, s_head, SEEK_SET) != 0) { fclose(f); return; }

    static char line[STORE_REC_MAX + 2];
    long dropped = 0;
    const long target = (long)STORE_MAX_BYTES - (long)STORE_TRIM_BYTES;
    while (log_size() - s_head > target) {
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        if (!fgets(line, sizeof(line), f)) break;
        s_head = ftell(f);
        dropped++;
    }
    fclose(f);
    if (dropped) {
        s_pending -= (int)dropped;
        if (s_pending < 0) s_pending = 0;
        /* Every dropped record leaves the head, so the token of anything a
         * concurrent peek is holding is now stale — which is the whole point:
         * this is the path that used to make a peek/pop pair discard a record
         * it never published. */
        s_head_seq += (uint32_t)dropped;
        s_trimmed  += (uint32_t)dropped;
        ESP_LOGW(TAG, "queue full — dropped %ld oldest record(s)", dropped);
        head_save();
        compact();
    }
}

/* ── public API ───────────────────────────────────────────────────────── */

/* ── Whose records are these? ────────────────────────────────────────────
 * The offline queue survives a firmware change: an OTA writes an app slot and
 * flips otadata, it does not touch `storage`. The CNG sibling
 * (cng-cpil-vvm701-fw) mounts this same partition at the same path and reads
 * the same q.ndjson, and both products store records as verbatim JSON that
 * replay republishes byte-for-byte — so whichever firmware boots next inherits,
 * and publishes, whatever the previous one left behind, under the other
 * station's topic tree.
 *
 * The config cannot answer "whose queue is this": on a round trip
 * (this product -> CNG -> back) the config found on return is genuinely ours
 * and the product guard correctly stays quiet, while the queue is full of the
 * other product's readings. So the queue carries its OWN owner, checked before
 * a single record is read back. This is the same protocol cng-cpil runs
 * (store_fs.c, claim_the_queue) on the same file, so the two firmwares agree
 * on the partition's state whichever direction a gateway travels:
 *
 *   owner == PRODUCT_ID   normal boot / same-product update: keep the backlog
 *   owner  = other value  the other product last owned it: purge, then claim
 *   owner absent          unclaimed (fresh board, or the other product handed
 *                         it over via store_disown()): purge, then claim
 *
 * fota.c calls store_disown() on a cross-product handover so the marker is
 * gone for exactly as long as the other product owns the flash. A fresh board
 * has no marker and no records, so the purge is a no-op and only the marker is
 * written. Runs unlocked: called from store_init() before s_ready is set, so
 * nothing else can touch the queue yet. */
static void claim_the_queue(void) {
    char owner[64] = {0};
    FILE *f = fopen(STORE_OWNER, "r");
    if (f) {
        if (!fgets(owner, sizeof(owner), f)) owner[0] = '\0';
        fclose(f);
        owner[strcspn(owner, "\r\n")] = '\0';
    }
    if (strcmp(owner, PRODUCT_ID) == 0) return;          /* ours — the normal path */

    long bytes = log_size();
    if (bytes > 0) {
        queue_reset();
        ESP_LOGW(TAG, "queue belongs to [%s], not [%s] — discarded %ld bytes of records "
                      "rather than republish one product's readings as the other's",
                 owner[0] ? owner : "(no owner recorded)", PRODUCT_ID, bytes);
    }
    f = fopen(STORE_OWNER, "w");
    if (!f) {
        ESP_LOGE(TAG, "could not record queue ownership — the queue will be re-examined "
                      "(and discarded again) on every boot");
        return;
    }
    fputs(PRODUCT_ID, f);
    fclose(f);
    ESP_LOGI(TAG, "queue claimed for [%s]", PRODUCT_ID);
}

void store_disown(void) {
    if (!s_ready) return;
    if (unlink(STORE_OWNER) == 0)
        ESP_LOGW(TAG, "queue ownership released — the next firmware to boot will "
                      "discard whatever it finds here");
}

bool store_init(void) {
    esp_vfs_littlefs_conf_t conf = {
        .base_path              = STORE_BASE,
        .partition_label        = STORE_PART,
        .format_if_mount_failed = true,   /* first boot: blank partition */
        .dont_mount             = false,
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        /* Not fatal by design: without the queue the gateway behaves exactly
         * as it did before this feature existed (records dropped on a failed
         * publish). A storage fault must never stop it polling or publishing. */
        ESP_LOGE(TAG, "littlefs mount failed (%s) — queue disabled", esp_err_to_name(err));
        return false;
    }

    size_t total = 0, used = 0;
    if (esp_littlefs_info(STORE_PART, &total, &used) == ESP_OK)
        ESP_LOGI(TAG, "mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock)
        ESP_LOGE(TAG, "queue mutex alloc failed — store will be unarbitrated");

    /* Delete any orphaned compaction temp. compact() renames STORE_TMP over
     * STORE_LOG atomically, so a STORE_TMP still present at boot means a previous
     * compaction was cut short (brownout/genset switchover) — the OLD log is
     * intact and this file is stale. It is a FIXED name and the next compaction
     * would overwrite it anyway (so it never accumulates), but clearing it now
     * frees its space immediately rather than reserving it until then. Safe: at
     * boot an existing STORE_TMP is always the discardable side of the rename. */
    unlink(STORE_TMP);

    head_load();

    long size = log_size();
    /* `s_head > 0 && s_head >= size`, NOT `size > 0 && s_head >= size`. The
     * second form misses a power cut INSIDE queue_reset() between its two
     * unlink()s: the log is gone but q.head survives, so a stale cursor points
     * past the end of the fresh empty log that du_task starts next. Every peek
     * then fseeks past EOF and nothing publishes until the log outgrows the
     * stale offset — and the first record read after that is a partial line.
     * `head == 0 && size == 0` is the valid never-used state and stays as is.
     * (Found and fixed in cng-cpil's port of this file, 2026-09-01.) */
    if (s_head > 0 && s_head >= size) queue_reset();   /* fully drained last run, or torn reset */

    claim_the_queue();                  /* before s_ready: nothing else can race it */
    s_ready = true;
    s_pending = store_count();          /* seed the O(1) counter from the real scan */
    ESP_LOGI(TAG, "queue holds %d record(s)", s_pending);
    return true;
}

bool store_ready(void) { return s_ready; }

bool store_enqueue(const char *payload) {
    if (!s_ready || !payload || !*payload) return false;

    size_t len = strlen(payload);
    if (len > STORE_REC_MAX) {
        ESP_LOGW(TAG, "record too large (%u) — dropped", (unsigned)len);
        return false;
    }
    /* Records are newline-delimited, so an embedded newline would split one
     * record into two unparseable halves. Every payload built in this
     * firmware is single-line, so this is a guard against a future change
     * rather than an observed case. */
    if (memchr(payload, '\n', len)) {
        ESP_LOGW(TAG, "record contains newline — dropped");
        return false;
    }

    if (!store_lock()) { ESP_LOGW(TAG, "enqueue: lock timeout — record dropped"); return false; }

    /* s_head > 0 is part of the condition, not just an optimisation: with
     * nothing consumed there is nothing to reclaim, and calling compact() to
     * find that out is how the queue-wipe bug documented above was reached. */
    if (s_head > 0 && log_size() > STORE_COMPACT_AT) compact();
    if (log_size() - s_head > STORE_MAX_BYTES) drop_oldest_until_fits();

    FILE *f = fopen(STORE_LOG, "a");
    if (!f) { store_unlock(); ESP_LOGE(TAG, "append failed to open"); return false; }
    int w  = fprintf(f, "%s\n", payload);
    /* fclose flushes stdio's buffer to littlefs — the ACTUAL flash write can
     * fail HERE (bad block / partition edge) even though fprintf() already
     * succeeded into the in-memory buffer. Check it: otherwise a flush failure
     * silently loses the record while s_pending is bumped as if it were stored,
     * and du_emit believes the spill succeeded. Only count the record once it
     * is truly on flash. */
    int ce = fclose(f);
    if (w < 0 || ce != 0) {
        store_unlock();
        ESP_LOGE(TAG, "append write/flush failed (w=%d close=%d) — record dropped", w, ce);
        return false;
    }
    s_pending++;
    store_unlock();
    return true;
}

bool store_has_records(void) {
    if (!s_ready) return false;
    if (!store_lock()) return false;
    bool any = (log_size() > s_head);
    store_unlock();
    return any;
}

int store_count(void) {
    if (!s_ready) return 0;
    if (!store_lock()) return 0;
    FILE *f = fopen(STORE_LOG, "r");
    if (!f) { store_unlock(); return 0; }
    if (fseek(f, s_head, SEEK_SET) != 0) { fclose(f); store_unlock(); return 0; }
    static char line[STORE_REC_MAX + 2];
    int n = 0;
    while (fgets(line, sizeof(line), f)) {
        /* This walks the entire live log. It is only called at boot and never
         * on a hot path (diagnostics uses the O(1) counter below), but a 1 MB
         * backlog is thousands of lines and the WDT does not care why a task
         * is busy. */
        if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
        n++;
    }
    fclose(f);
    store_unlock();
    return n;
}

/* O(1) approximate record count, maintained by enqueue/pop rather than by
 * scanning. diagnostics reports this every cycle, and calling the real
 * store_count() there would walk up to 1 MB while holding the store lock —
 * blocking du_task from spilling for the duration, on a purely informational
 * read. Seeded from the real count at mount, so it is exact unless a record is
 * dropped by the overflow path, which logs separately anyway. */
int store_pending(void)
{
    return s_pending > 0 ? s_pending : 0;
}

bool store_purge(void)
{
    if (!s_ready) return false;
    if (!store_lock()) {
        ESP_LOGE(TAG, "purge: lock timeout — queue NOT cleared");
        return false;
    }
    int had = s_pending;
    queue_reset();
    store_unlock();
    ESP_LOGW(TAG, "queue purged: %d record(s) discarded", had);
    return true;
}

bool store_peek(char *buf, size_t buf_size, uint32_t *token) {
    if (!s_ready || !buf || buf_size == 0) return false;
    if (!store_lock()) return false;
    FILE *f = fopen(STORE_LOG, "r");
    if (!f) { store_unlock(); return false; }
    if (fseek(f, s_head, SEEK_SET) != 0) { fclose(f); store_unlock(); return false; }
    bool ok = (fgets(buf, (int)buf_size, f) != NULL);
    if (ok && token) *token = s_head_seq;   /* read under the lock, with the record */
    fclose(f);
    store_unlock();
    if (!ok) return false;
    buf[strcspn(buf, "\r\n")] = '\0';
    return buf[0] != '\0';
}

bool store_pop(uint32_t token) {
    if (!s_ready) return false;
    if (!store_lock()) return false;

    /* The head moved while the caller was publishing — a trim dropped records,
     * or the queue was reset. Discarding now would throw away a record nobody
     * has published. Refuse; the new head gets published next cycle. */
    if (token != s_head_seq) {
        ESP_LOGW(TAG, "pop refused: head moved during publish (token %lu, now %lu) — "
                      "nothing discarded",
                 (unsigned long)token, (unsigned long)s_head_seq);
        store_unlock();
        return false;
    }

    FILE *f = fopen(STORE_LOG, "r");
    if (!f) { store_unlock(); return false; }
    if (fseek(f, s_head, SEEK_SET) != 0) { fclose(f); store_unlock(); return false; }
    static char line[STORE_REC_MAX + 2];
    if (!fgets(line, sizeof(line), f)) { fclose(f); queue_reset(); store_unlock(); return false; }
    s_head = ftell(f);
    fclose(f);

    if (s_pending > 0) s_pending--;
    s_head_seq++;                              /* this record has left the head */
    if (s_head >= log_size()) queue_reset();   /* drained — reclaim the space */
    else                      head_save();
    store_unlock();
    return true;
}

size_t store_used_bytes(void) {
    if (!s_ready) return 0;
    size_t total = 0, used = 0;
    if (esp_littlefs_info(STORE_PART, &total, &used) != ESP_OK) return 0;
    return used;
}

uint32_t store_trimmed(void) { return s_trimmed; }

/* Live queue bytes — exactly the quantity drop_oldest_until_fits() compares
 * against its target, so a percentage derived from this is a true distance to
 * the point where the oldest sales start being discarded.
 *
 * store_used_bytes() cannot answer that: it reports whole-partition usage
 * (1920 KB, including littlefs metadata and any compaction temp file), not the
 * 768 KB STORE_MAX_BYTES ceiling the trim is measured against. */
size_t store_queue_bytes(void) {
    if (!s_ready) return 0;
    long size = log_size();
    if (size <= s_head) return 0;
    return (size_t)(size - s_head);
}
