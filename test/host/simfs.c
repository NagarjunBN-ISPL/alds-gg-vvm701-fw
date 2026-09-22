/* Filesystem shim: redirects store.c's compiled-in "/store/..." paths into a
 * temp directory, and can simulate a full partition. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "esp_littlefs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static char   s_root[256] = "/tmp/storetest";
static int    s_mount_fail;
static size_t s_capacity;      /* 0 = unlimited */

void sim_fs_setroot(const char *root) { snprintf(s_root, sizeof(s_root), "%s", root); }
void simfs_mount_fails(int f)         { s_mount_fail = f; }
void simfs_set_capacity(size_t b)     { s_capacity = b; }

static const char *mapped(const char *path) {
    static char buf[512];
    if (strncmp(path, "/store", 6) == 0)
        snprintf(buf, sizeof(buf), "%s%s", s_root, path + 6);
    else
        snprintf(buf, sizeof(buf), "%s", path);
    return buf;
}

/* Total bytes currently stored, for the capacity simulation. */
static size_t dir_bytes(void) {
    size_t total = 0;
    const char *names[] = { "/q.ndjson", "/q.head", "/q.tmp" };
    for (unsigned i = 0; i < 3; i++) {
        char p[512];
        struct stat st;
        snprintf(p, sizeof(p), "%s%s", s_root, names[i]);
        if (stat(p, &st) == 0) total += (size_t)st.st_size;
    }
    return total;
}

FILE *sim_fopen(const char *path, const char *mode) {
    if (s_capacity && (mode[0] == 'w' || mode[0] == 'a') && dir_bytes() >= s_capacity)
        return NULL;                              /* partition full */
    return fopen(mapped(path), mode);
}
int sim_stat(const char *path, struct stat *st) { return stat(mapped(path), st); }
int sim_unlink(const char *path)                { return unlink(mapped(path)); }
int sim_rename(const char *a, const char *b) {
    char first[512];
    snprintf(first, sizeof(first), "%s", mapped(a));
    return rename(first, mapped(b));
}

esp_err_t esp_vfs_littlefs_register(const esp_vfs_littlefs_conf_t *conf) {
    (void)conf;
    if (s_mount_fail) return 1;
    char cmd[600];
    snprintf(cmd, sizeof(cmd), "mkdir -p '%s'", s_root);
    if (system(cmd) != 0) return 1;
    return ESP_OK;
}
esp_err_t esp_littlefs_info(const char *label, size_t *total, size_t *used) {
    (void)label;
    if (total) *total = s_capacity ? s_capacity : (1920u * 1024u);
    if (used)  *used  = dir_bytes();
    return ESP_OK;
}
const char *esp_err_to_name(esp_err_t e) { (void)e; return "ESP_FAIL"; }

/* ── FreeRTOS mutex (single-threaded host: counters only) ────────────────── */
static int s_mutex_obj;
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &s_mutex_obj; }
int  xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) { (void)s; (void)t; return 1; }
void xSemaphoreGive(SemaphoreHandle_t s) { (void)s; }
