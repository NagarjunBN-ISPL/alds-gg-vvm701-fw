/* Drives the product guard and the string-termination hardening extracted from
 * src/nvs_config.c against a stubbed NVS. */
#include "shim.h"
#include "nvs.h"

int g_log_quiet = 1;

/* ── stubbed NVS ───────────────────────────────────────────────────────── */
static char sim_tag[64];
static bool sim_tag_present;
static int  sim_erases, sim_writes;
static int  sim_sibling_erases;      /* nvs_erase_all() on the CNG namespace */
static bool sim_open_fails;

/* Handle 1 = our namespace, handle 2 = the CNG sibling's. The guard must wipe
 * BOTH on a product mismatch and NEITHER otherwise. */
esp_err_t nvs_open(const char *ns, int mode, nvs_handle_t *out) {
    (void)mode;
    if (sim_open_fails) return 1;
    *out = (strcmp(ns, "cng") == 0) ? 2 : 1;
    return ESP_OK;
}
esp_err_t nvs_get_str(nvs_handle_t h, const char *key, char *out, size_t *len) {
    (void)h; (void)key;
    if (!sim_tag_present) return ESP_ERR_NVS_NOT_FOUND;
    snprintf(out, *len, "%s", sim_tag);
    return ESP_OK;
}
esp_err_t nvs_set_str(nvs_handle_t h, const char *key, const char *val) {
    (void)h; (void)key;
    snprintf(sim_tag, sizeof(sim_tag), "%s", val);
    sim_tag_present = true;
    sim_writes++;
    return ESP_OK;
}
esp_err_t nvs_erase_all(nvs_handle_t h) {
    if (h == 2) { sim_sibling_erases++; return ESP_OK; }
    sim_erases++; sim_tag_present = false; return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h)    { (void)h; return ESP_OK; }
void      nvs_close(nvs_handle_t h)     { (void)h; }

/* State the extracted functions expect to find. */
#define PRODUCT_KEY "product"
#define SIBLING_NS_CNG "cng"
#define NS          "iot_cfg"
static bool s_product_wiped = false;
static char s_reverted_foreign[40] = "";

#include "extracted_product.inc"

static int fails = 0, checks = 0;
#define CHECK(cond, fmt, ...) do { checks++; if (!(cond)) { fails++; \
        printf("  FAIL  " fmt "\n", ##__VA_ARGS__); } } while (0)

static void sim_reset(const char *tag) {
    sim_erases = sim_writes = sim_sibling_erases = 0;
    sim_open_fails = false;
    s_product_wiped = false;
    s_reverted_foreign[0] = '\0';
    if (tag) { snprintf(sim_tag, sizeof(sim_tag), "%s", tag); sim_tag_present = true; }
    else     { sim_tag[0] = '\0'; sim_tag_present = false; }
}

int main(void) {
    printf("== product guard (extracted from nvs_config.c) ==\n");

    /* 1. THE CASE ASKED ABOUT: an LPG gateway reflashed as a CNG unit. */
    printf("[1] NVS from a different product\n");
    {
        sim_reset("cng-cpil-vvm701/1");
        product_check();
        CHECK(sim_erases == 1, "config was NOT erased — the gateway would come up "
                               "reporting 'ready' with the other product's equipment");
        CHECK(sim_sibling_erases == 1, "the CNG sibling's namespace was not erased — a later "
                                       "reflash to CNG would boot on a stale station plan");
        CHECK(nvs_config_product_wiped(), "wipe not reported to the caller — "
                                          "the offline queue would not be purged");
        CHECK(strcmp(sim_tag, PRODUCT_ID) == 0, "tag not restamped (got [%s])", sim_tag);
        CHECK(sim_writes == 1, "expected exactly one stamp, got %d", sim_writes);
    }

    /* 1b. The other product's image was tried and ROLLED BACK by the
     *     bootloader: its stamp is a lie, the config underneath is ours. */
    printf("[1b] NVS stamped by a foreign image the bootloader just reverted\n");
    {
        sim_reset("cng-cpil-vvm701/1");
        nvs_config_note_reverted_foreign_image("cng-cpil-vvm701-fw");
        product_check();
        CHECK(sim_erases == 0, "erased OUR configuration because a failed swap attempt stamped it");
        CHECK(sim_sibling_erases == 0, "touched the sibling namespace after a reverted swap");
        CHECK(!nvs_config_product_wiped(), "reported a wipe — the offline queue would be purged for nothing");
        CHECK(strcmp(sim_tag, PRODUCT_ID) == 0, "tag not restamped after the reverted image (got [%s])", sim_tag);
        CHECK(sim_writes == 1, "expected exactly one restamp, got %d", sim_writes);
        /* The hint must not blind the guard to a REAL foreign product later. */
        sim_reset("cng-cpil-vvm701/1");
        product_check();
        CHECK(sim_erases == 1 && sim_sibling_erases == 1,
              "after the hint is cleared a real foreign stamp must still wipe");
    }

    /* 2. Same product — the normal path must touch nothing. */
    printf("[2] NVS from this product\n");
    {
        sim_reset(PRODUCT_ID);
        product_check();
        CHECK(sim_erases == 0, "erased a matching product's configuration");
        CHECK(sim_sibling_erases == 0, "touched the sibling namespace on a normal boot");
        CHECK(sim_writes == 0, "rewrote the tag every boot — needless flash wear");
        CHECK(!nvs_config_product_wiped(), "reported a wipe that did not happen");
    }

    /* 3. Legacy unit with no tag: adopt, never wipe. */
    printf("[3] gateway provisioned before the tag existed\n");
    {
        sim_reset(NULL);
        product_check();
        CHECK(sim_erases == 0, "wiped a legacy unit — this would deconfigure the "
                               "entire deployed fleet on its first FOTA");
        CHECK(!nvs_config_product_wiped(), "reported a wipe on a legacy unit");
        CHECK(strcmp(sim_tag, PRODUCT_ID) == 0, "did not stamp the legacy unit");
        /* and it is then a no-op forever after */
        sim_reset(PRODUCT_ID);
        product_check();
        CHECK(sim_erases == 0 && sim_writes == 0, "not idempotent after stamping");
    }

    /* 4. A schema bump within the same product still wipes. */
    printf("[4] same product, incompatible schema\n");
    {
        sim_reset("alds-sr-vvm701/0");
        product_check();
        CHECK(sim_erases == 1, "a schema bump must erase — the blob layout moved");
        CHECK(nvs_config_product_wiped(), "schema bump not reported");
    }

    /* 5. NVS unavailable: degrade, never wipe on a guess. */
    printf("[5] NVS open fails\n");
    {
        sim_reset("something-else");
        sim_open_fails = true;
        product_check();
        CHECK(sim_erases == 0, "erased without being able to read the tag");
        CHECK(!nvs_config_product_wiped(), "claimed a wipe it could not perform");
    }

    /* 6. String hardening: a blob with no terminators anywhere. */
    printf("[6] unterminated strings from a foreign blob\n");
    {
        DeviceConfig c;
        memset(&c, 'X', sizeof(c));          /* every byte non-NUL */
        terminate_strings(&c);
        CHECK(strlen(c.mqtt_host) == sizeof(c.mqtt_host) - 1,
              "mqtt_host runs past its array (%zu)", strlen(c.mqtt_host));
        CHECK(strlen(c.apn) == sizeof(c.apn) - 1, "apn unterminated");
        CHECK(strlen(c.mqtt_pass) == sizeof(c.mqtt_pass) - 1, "mqtt_pass unterminated");
        CHECK(strlen(c.mqtt_base_topic) == sizeof(c.mqtt_base_topic) - 1, "base topic unterminated");
        CHECK(strlen(c.mqtt_host_bak) == sizeof(c.mqtt_host_bak) - 1, "host _bak unterminated");
        CHECK(strlen(c.sfx_genset) == 2, "sfx_genset unterminated");
        for (int i = 0; i < DU_MAX_DISPENSERS; i++)
            CHECK(strlen(c.du_sfx[i]) == 2, "du_sfx[%d] unterminated", i);
        CHECK(strlen(c.sfx_di) == 2, "sfx_di unterminated");
        for (int i = 0; i < RADAR_MAX; i++) {
            CHECK(strlen(c.sfx_ztech[i]) == 2, "sfx_ztech[%d] unterminated", i);
            CHECK(strlen(c.sfx_pantech[i]) == 2, "sfx_pantech[%d] unterminated", i);
        }
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
