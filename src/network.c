#include "network.h"
#include "modem.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "network";

/* Live APN — defaults to the compiled-in config.h value so network_init()
 * still works if network_set_apn() is never called. app_main() overrides
 * these from DeviceConfig right after loading NVS. */
static char s_apn[48]      = MODEM_APN;
static char s_apn_user[32] = MODEM_APN_USER;
static char s_apn_pass[32] = MODEM_APN_PASS;

void network_set_apn(const char *apn, const char *user, const char *pass) {
    strncpy(s_apn, apn, sizeof(s_apn) - 1);
    s_apn[sizeof(s_apn) - 1] = '\0';
    strncpy(s_apn_user, user ? user : "", sizeof(s_apn_user) - 1);
    s_apn_user[sizeof(s_apn_user) - 1] = '\0';
    strncpy(s_apn_pass, pass ? pass : "", sizeof(s_apn_pass) - 1);
    s_apn_pass[sizeof(s_apn_pass) - 1] = '\0';
}

void network_build_qicsgp(char *out, size_t out_size) {
    /* auth type: 0=none, 3=PAP or CHAP (modem negotiates). Only send auth
     * when a username is actually set — an empty-user context with auth=3
     * is rejected by some networks. */
    int auth = (s_apn_user[0] != '\0') ? 3 : 0;
    snprintf(out, out_size, "AT+QICSGP=1,1,\"%s\",\"%s\",\"%s\",%d",
             s_apn, s_apn_user, s_apn_pass, auth);
}

static int parse_reg_stat(const char *resp, const char *prefix) {
    const char *p = strstr(resp, prefix);
    if (!p) return -1;
    p += strlen(prefix);
    while (*p == ' ') p++;
    const char *comma = strchr(p, ',');
    if (comma) return atoi(comma + 1);
    return atoi(p);
}

static bool wait_registered(int timeout_ms) {
    /* Enable verbose CEREG URCs so we can see stat,tac,ci later */
    modem_at("AT+CEREG=2", "OK", 2000, NULL, 0);

    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    int tick = 0;
    while (esp_timer_get_time() < deadline) {
        char r1[64] = {0}, r2[64] = {0};
        modem_at("AT+CEREG?", "+CEREG:", 3000, r1, sizeof(r1));
        modem_at("AT+CGREG?", "+CGREG:", 3000, r2, sizeof(r2));
        int cereg = parse_reg_stat(r1, "+CEREG:");
        int cgreg = parse_reg_stat(r2, "+CGREG:");
        ESP_LOGI(TAG, "CEREG=%d CGREG=%d (%ds)", cereg, cgreg, tick * 3);
        /* Registered on EITHER access tech is registered — check the positive
         * case FIRST. An LTE-only SIM (Jio 4G, private enterprise APN with no 2G
         * roaming) commonly comes up CEREG=1 (LTE) while the tower returns
         * CGREG=3 (2G/GPRS denied); returning false on CGREG==3 alone stranded
         * exactly those SIMs at tick 0. stat 1 = home, 5 = roaming. */
        if (cereg == 1 || cereg == 5) return true;
        if (cgreg == 1 || cgreg == 5) return true;
        /* Give up early ONLY when BOTH techs are explicitly DENIED (stat 3).
         * A lone 3 with the other tech still 0/2/4 (not registered / searching /
         * unknown) is not terminal — keep polling until the timeout, during
         * which the searching tech may still attach. stat 0/2 must never be read
         * as "denied" (that was the flaw in one proposed fix): 0 is the normal
         * state at tick 0 before the scan has even started. */
        if (cereg == 3 && cgreg == 3) {
            ESP_LOGE(TAG, "Registration denied on BOTH LTE and 2G (CEREG=3, CGREG=3)");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
        tick++;
    }
    return false;
}

bool network_init(void) {
    /* cmd must hold the QICSGP command, whose APN+user+pass can total ~110
     * chars — keep it comfortably above 128. */
    char cmd[200], resp[256];
    bool ok;

    modem_at("ATE0",      "OK", 2000, NULL, 0);
    modem_at("AT+CMEE=2", "OK", 2000, NULL, 0);

    /* SIM ready check. The modem stays powered across esp_restart() (never
     * re-pulsed), so a wedged SIM subsystem — e.g. left in a bad state by a
     * FOTA attempt's QIDEACT/reactivate — will keep answering CPIN? with
     * +CME ERROR forever, and an ESP reboot alone won't clear it. So: poll
     * for readiness, and if it stays stuck, PWRKEY power-cycle the modem
     * (which DOES reset the baseband/SIM) and try once more before failing. */
    bool sim_ready = false;
    for (int cycle = 0; cycle < 2 && !sim_ready; cycle++) {
        if (cycle > 0) {
            ESP_LOGW(TAG, "SIM stuck — power-cycling modem to reset SIM subsystem");
            modem_hard_reset();
            modem_at("ATE0",      "OK", 2000, NULL, 0);
            modem_at("AT+CMEE=2", "OK", 2000, NULL, 0);
        }
        for (int i = 0; i < 10; i++) {
            memset(resp, 0, sizeof(resp));
            modem_at("AT+CPIN?", "READY", 5000, resp, sizeof(resp));
            if (strstr(resp, "READY")) { sim_ready = true; break; }
            ESP_LOGW(TAG, "SIM not ready (cycle %d try %d/10)", cycle + 1, i + 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!sim_ready) {
        ESP_LOGE(TAG, "SIM not ready after power-cycle — rebooting");
        return false;
    }
    ESP_LOGI(TAG, "SIM: READY");

    /* Signal strength */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+CSQ", "+CSQ:", 3000, resp, sizeof(resp));
    const char *p = strstr(resp, "+CSQ:");
    if (p) {
        int rssi = atoi(p + 5);
        if (rssi == 99) ESP_LOGW(TAG, "Signal: unknown — check antenna");
        else            ESP_LOGI(TAG, "Signal: %d dBm (rssi=%d)", -113 + 2 * rssi, rssi);
    }

    /* Disable RF, set scan mode, re-enable — ensures clean attach */
    modem_at("AT+CFUN=0", "OK", 15000, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Automatic network scan mode (GSM + LTE), take effect immediately */
    ok = modem_at("AT+QCFG=\"nwscanmode\",0,1", "OK", 3000, NULL, 0);
    ESP_LOGI(TAG, "nwscanmode=auto: %s", ok ? "OK" : "FAIL");

    /* PDP context */
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", s_apn);
    ok = modem_at(cmd, "OK", 5000, NULL, 0);
    ESP_LOGI(TAG, "CGDCONT: %s", ok ? "OK" : "FAIL");

    ok = modem_at("AT+CFUN=1", "OK", 15000, NULL, 0);
    ESP_LOGI(TAG, "CFUN=1: %s", ok ? "OK" : "FAIL");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Waiting for network registration...");
    if (!wait_registered(120000)) {
        ESP_LOGE(TAG, "Network registration timeout");
        return false;
    }
    ESP_LOGI(TAG, "Registered");

    /* Log network info */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+QNWINFO", "OK", 3000, resp, sizeof(resp));
    ESP_LOGI(TAG, "Network: %s", resp);

    /* Activate Quectel TCP/IP context.
     * When no context is active, Quectel replies with bare OK — no "+QIACT:"
     * line — so wait on "OK" itself rather than stalling the full timeout. */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+QIACT?", "OK", 3000, resp, sizeof(resp));
    if (strstr(resp, "+QIACT: 1,1")) {
        ESP_LOGI(TAG, "PDP already active");
    } else {
        network_build_qicsgp(cmd, sizeof(cmd));
        modem_at(cmd, "OK", 5000, NULL, 0);
        ok = modem_at("AT+QIACT=1", "OK", 60000, NULL, 0);
        ESP_LOGI(TAG, "QIACT=1: %s", ok ? "OK" : "FAIL");
    }

    /* Log assigned IP */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+CGPADDR=1", "+CGPADDR:", 5000, resp, sizeof(resp));
    ESP_LOGI(TAG, "IP: %s", resp);

    /* Confirm the data bearer is genuinely up before declaring success. If
     * QIACT=1 above failed, the modem is registered but cannot open a socket;
     * returning true anyway lets boot bring-up skip the modem-hard-reset
     * escalation and march straight into an mqtt_connect() that cannot work.
     * Re-query with the same "+QIACT: 1,1" marker the runtime bearer check uses
     * (a bare OK is printed even with NO context active, so test for the
     * marker, never its absence). */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+QIACT?", "OK", 3000, resp, sizeof(resp));
    if (!strstr(resp, "+QIACT: 1,1")) {
        ESP_LOGE(TAG, "PDP context 1 not active after bring-up — network_init failed");
        return false;
    }
    return true;
}

bool network_check(void) {
    char resp[64] = {0};
    modem_at("AT+CEREG?", "+CEREG:", 3000, resp, sizeof(resp));
    int s = parse_reg_stat(resp, "+CEREG:");
    if (s == 1 || s == 5) return true;
    memset(resp, 0, sizeof(resp));
    modem_at("AT+CGREG?", "+CGREG:", 3000, resp, sizeof(resp));
    s = parse_reg_stat(resp, "+CGREG:");
    return (s == 1 || s == 5);
}

bool network_data_bearer_up(void) {
    /* Registration first: it is the cheaper question, and a modem that has
     * stopped answering AT at all fails it — which is what lets the ladder tell
     * a wedged modem from a broker outage. */
    if (!network_check()) return false;

    /* Then the bearer itself. Same parse as network_init() above: with no
     * context active the modem answers a bare OK and prints no "+QIACT:" line
     * at all, so the ABSENCE of the marker is the negative case — never test
     * for a "0" that is not printed. */
    char resp[128] = {0};
    modem_at("AT+QIACT?", "OK", 3000, resp, sizeof(resp));
    if (strstr(resp, "+QIACT: 1,1")) return true;

    ESP_LOGW(TAG, "registered, but PDP context 1 is DOWN — no socket can be "
                  "opened until it is reactivated");
    return false;
}
