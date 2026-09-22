/* In-memory NVS with fault injection. Deliberately allocates the blob to the
 * EXACT stored length, so ASan traps any read past what NVS actually holds —
 * the failure mode a real short/corrupt blob would produce on the device. */
#include "nvs.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char   s_tag[64];
static int    s_tag_present;
static void  *s_blob;
static size_t s_blob_len;
static int    s_have_blob;
static int    s_erases;
static int    s_open_fails;
static int    s_write_fails;

void simnvs_reset(void) {
    free(s_blob);
    s_blob = NULL; s_blob_len = 0; s_have_blob = 0;
    s_tag[0] = '\0'; s_tag_present = 0;
    s_erases = s_open_fails = s_write_fails = 0;
}
void simnvs_set_tag(const char *tag) {
    if (!tag) { s_tag_present = 0; s_tag[0] = '\0'; return; }
    snprintf(s_tag, sizeof(s_tag), "%s", tag);
    s_tag_present = 1;
}
void simnvs_set_blob_raw(const void *d, size_t n) {
    free(s_blob);
    s_blob = malloc(n ? n : 1);
    memcpy(s_blob, d, n);
    s_blob_len = n;
    s_have_blob = 1;
}
void simnvs_no_blob(void)          { free(s_blob); s_blob = NULL; s_blob_len = 0; s_have_blob = 0; }
void simnvs_open_fails(int f)      { s_open_fails = f; }
void simnvs_writes_fail(int f)     { s_write_fails = f; }
int  simnvs_erases(void)           { return s_erases; }
const char *simnvs_tag(void)       { return s_tag_present ? s_tag : NULL; }
size_t simnvs_blob_len(void)       { return s_have_blob ? s_blob_len : 0; }
const void *simnvs_blob(void)      { return s_have_blob ? s_blob : NULL; }

esp_err_t nvs_open(const char *ns, int mode, nvs_handle_t *out) {
    (void)ns; (void)mode;
    if (s_open_fails) return 1;
    *out = 1;
    return ESP_OK;
}
esp_err_t nvs_get_str(nvs_handle_t h, const char *key, char *out, size_t *len) {
    (void)h;
    if (strcmp(key, "product") != 0 || !s_tag_present) return ESP_ERR_NVS_NOT_FOUND;
    size_t need = strlen(s_tag) + 1;
    if (!out) { *len = need; return ESP_OK; }
    if (*len < need) return ESP_ERR_NVS_INVALID_LENGTH;
    memcpy(out, s_tag, need);
    *len = need;
    return ESP_OK;
}
esp_err_t nvs_set_str(nvs_handle_t h, const char *key, const char *val) {
    (void)h;
    if (s_write_fails) return 1;
    if (strcmp(key, "product") == 0) simnvs_set_tag(val);
    return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t h, const char *key, void *out, size_t *len) {
    (void)h; (void)key;
    if (!s_have_blob) return ESP_ERR_NVS_NOT_FOUND;
    if (!out) { *len = s_blob_len; return ESP_OK; }
    if (*len < s_blob_len) return ESP_ERR_NVS_INVALID_LENGTH;
    memcpy(out, s_blob, s_blob_len);
    *len = s_blob_len;
    return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h, const char *key, const void *val, size_t len) {
    (void)h; (void)key;
    if (s_write_fails) return 1;
    simnvs_set_blob_raw(val, len);
    return ESP_OK;
}
esp_err_t nvs_get_u8 (nvs_handle_t h, const char *k, uint8_t  *o) { (void)h;(void)k;(void)o; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_get_u16(nvs_handle_t h, const char *k, uint16_t *o) { (void)h;(void)k;(void)o; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_get_u32(nvs_handle_t h, const char *k, uint32_t *o) { (void)h;(void)k;(void)o; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_set_u32(nvs_handle_t h, const char *k, uint32_t v)  { (void)h;(void)k;(void)v; return s_write_fails ? 1 : ESP_OK; }
esp_err_t nvs_set_u8 (nvs_handle_t h, const char *k, uint8_t v)   { (void)h;(void)k;(void)v; return s_write_fails ? 1 : ESP_OK; }
esp_err_t nvs_erase_key(nvs_handle_t h, const char *k)            { (void)h;(void)k; return ESP_OK; }
esp_err_t nvs_erase_all(nvs_handle_t h) {
    (void)h;
    s_erases++;
    simnvs_no_blob();
    s_tag_present = 0; s_tag[0] = '\0';
    return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }
void      nvs_close(nvs_handle_t h)  { (void)h; }
