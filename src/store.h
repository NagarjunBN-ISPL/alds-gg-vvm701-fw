#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * store.h — offline record queue (store-and-forward).
 *
 * Holds records that could not be published while the link was down, and
 * replays them oldest-first once it recovers. Backed by littlefs on the
 * `storage` partition (data/littlefs, 0xe20000, 0x1e0000 = 1920K —
 * partitions.csv).
 *
 * ── What goes in here, and what deliberately does not ─────────────────────
 *
 *   queued      every dispenser payload du_task produces
 *   not queued  radar, genset, GPS, DI, diagnostics
 *
 * The exclusions are not an oversight: radar/genset/GPS/DI are sampled state,
 * so the first reading after recovery is current and replaying stale ones adds
 * nothing.
 *
 * NOTE — this is deliberately broader than the original design, and the
 * sizing follows from it. The queue was first written to hold only payloads
 * whose publish had actually FAILED (~300 records/day, hence the old "weeks of
 * buffer" note). Since acquisition moved to du_task, the whole poll stream is
 * buffered during an outage instead: every nozzle, every 12 s, roughly 14,400
 * records/day at two nozzles. That is intentional — the outage window is
 * exactly where per-sale granularity is destroyed, and a totalizer series with
 * holes in it cannot be reconstructed afterwards — but it means the partition
 * holds hours, not weeks. See STORE_MAX_BYTES in config.h for the arithmetic.
 *
 * ── Storage model ─────────────────────────────────────────────────────────
 * Records are the exact JSON payload that failed to publish, stored verbatim,
 * one per line (NDJSON). Storing the finished payload rather than a packed
 * binary record means replay is a byte-for-byte republish: no re-serialisation
 * at flush time, and no second copy of the payload schema that could drift
 * away from the live one in main.c.
 *
 * NOT NVS. NVS is 20K, holds the connectivity config that keeps the gateway
 * reachable, and Espressif's own guidance says it is unsuitable for logging or
 * frequent writes. A full queue must never be able to disturb the config.
 */

/* Mount the storage partition, formatting it on first use. Safe to call once
 * at boot; every other entry point is a no-op until this succeeds, so a
 * mount failure degrades to today's behaviour (records dropped) rather than
 * blocking the poll loop. Returns false if the partition is missing or
 * unmountable. */
bool store_init(void);

/* True once store_init() has succeeded. */
bool store_ready(void);

/* Append one payload to the queue. Call this ONLY when a publish has actually
 * failed. Returns false if the queue is unavailable or the write failed —
 * callers should treat that as "dropped", the same as today. */
bool store_enqueue(const char *payload);

/* Number of records currently queued (0 if unavailable).
 * O(n) — it scans the log. Use store_has_records() on hot paths. */
int store_count(void);

/* True if at least one record is queued. O(1): compares the read cursor
 * against the file size, no line scan. Exists because the enqueue path has to
 * ask this question on every single payload to preserve FIFO order. */
bool store_has_records(void);

/* Queued record count, O(1) — maintained by enqueue/pop rather than scanned.
 * Use this anywhere the number is only informational (diagnostics); calling
 * store_count() there would walk the whole log while holding the store lock,
 * blocking du_task from spilling for the duration of a purely cosmetic read. */
int store_pending(void);

/* ── Peek / pop, and why they carry a token ───────────────────────────────
 * Peek-then-pop is deliberate: the record is only discarded after the publish
 * is confirmed, so a reboot mid-flush re-sends rather than loses. Duplicates
 * are the safe failure direction — the cloud can dedupe on HW_ID + TS +
 * totalizer, but it cannot invent a lost sale.
 *
 * The store lock is RELEASED between the two, because a publish is a modem
 * round-trip that can block for many seconds and must not hold du_task off the
 * queue. That gap is real: du_task can call store_enqueue() in it, and
 * store_enqueue() may run drop_oldest_until_fits(), which advances the read
 * cursor past whole records. A bare store_pop() would then discard a record
 * that was never published — silent loss, at the end of a long outage, which
 * is exactly when the queue is full enough for that trim to fire.
 *
 * So store_peek() hands back a token identifying the record it returned, and
 * store_pop() only discards if that record is still at the head. If it is not,
 * the pop is refused (returns false) and the caller simply moves on — whatever
 * is at the head now will be published next cycle. */
bool store_peek(char *buf, size_t buf_size, uint32_t *token);

/* Discard the record identified by `token`, i.e. the one the matching
 * store_peek() returned. Call only after its publish is confirmed. Returns
 * false without discarding anything if the head has moved since the peek. */
bool store_pop(uint32_t token);

/* Bytes currently used by the queue, for the diagnostics report. */
size_t store_used_bytes(void);

/* Records EVICTED because the queue hit STORE_MAX_BYTES, cumulative since
 * boot. Distinct from du_task_dropped(), which counts records refused at
 * enqueue: these were accepted and buffered, then discarded to make room for
 * newer ones. Both are permanent sale loss; this one is what a long outage
 * actually produces, and it was invisible to the cloud before this counter. */
uint32_t store_trimmed(void);

/* Bytes of live (un-popped) queue, measured the same way the trim path
 * measures it. Use this — not store_used_bytes() — for "how close is this
 * device to discarding its oldest sales", since the trim ceiling is
 * STORE_MAX_BYTES, not the partition size. */
size_t store_queue_bytes(void);

/* Discard the entire queue. The ONLY caller is the boot path that detects the
 * firmware's compiled credentials pointing at a different broker than the one
 * NVS was configured for (see nvs_config_enforce_firmware_creds): whatever is
 * buffered belongs to the previous deployment, and publishing it to the new one
 * would leak one client's sales into another's account. That is the one case
 * where destroying buffered records is the correct action — everywhere else in
 * this module the rule is the opposite. Returns false if the queue was not
 * mounted or the lock could not be taken. */
bool store_purge(void);

/* Release the partition's ownership marker before a CROSS-PRODUCT FOTA hands
 * the flash to the other firmware (see claim_the_queue in store.c). Pair it
 * with store_purge(); the marker is what makes THIS product discard what it
 * finds when the gateway comes back, whichever firmware performed that trip. */
void store_disown(void);
