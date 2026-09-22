/* Host stub for nvs.h — an in-memory key store with fault injection, enough to
 * run the real nvs_config.c against corrupt / foreign / missing NVS. */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_task_wdt.h"   /* esp_err_t / ESP_OK */

#define ESP_ERR_NVS_NOT_FOUND         0x1102
#define ESP_ERR_NVS_NO_FREE_PAGES     0x1110
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1111
#define ESP_ERR_NVS_INVALID_LENGTH    0x1105
#define NVS_READONLY  0
#define NVS_READWRITE 1

typedef int nvs_handle_t;

esp_err_t nvs_open(const char *ns, int mode, nvs_handle_t *out);
esp_err_t nvs_get_str(nvs_handle_t h, const char *key, char *out, size_t *len);
esp_err_t nvs_set_str(nvs_handle_t h, const char *key, const char *val);
esp_err_t nvs_get_blob(nvs_handle_t h, const char *key, void *out, size_t *len);
esp_err_t nvs_set_blob(nvs_handle_t h, const char *key, const void *val, size_t len);
esp_err_t nvs_get_u8(nvs_handle_t h, const char *key, uint8_t *out);
esp_err_t nvs_get_u16(nvs_handle_t h, const char *key, uint16_t *out);
esp_err_t nvs_get_u32(nvs_handle_t h, const char *key, uint32_t *out);
esp_err_t nvs_set_u32(nvs_handle_t h, const char *key, uint32_t val);
esp_err_t nvs_set_u8(nvs_handle_t h, const char *key, uint8_t val);
esp_err_t nvs_erase_key(nvs_handle_t h, const char *key);
esp_err_t nvs_erase_all(nvs_handle_t h);
esp_err_t nvs_commit(nvs_handle_t h);
void      nvs_close(nvs_handle_t h);

/* ── fault injection, driven by the test ─────────────────────────────────── */
void  simnvs_reset(void);
void  simnvs_set_tag(const char *tag);          /* NULL = no product tag      */
void  simnvs_set_blob_raw(const void *d, size_t n);
void  simnvs_no_blob(void);
void  simnvs_open_fails(int fails);
void  simnvs_writes_fail(int fails);
int   simnvs_erases(void);
const char *simnvs_tag(void);
size_t simnvs_blob_len(void);
const void *simnvs_blob(void);
