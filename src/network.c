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
        if (cereg == 3) { ESP_LOGE(TAG, "Registration denied (CEREG=3)"); return false; }
        if (cgreg == 3) { ESP_LOGE(TAG, "Registration denied (CGREG=3)"); return false; }
        if (cereg == 1 || cereg == 5) return true;
        if (cgreg == 1 || cgreg == 5) return true;
        vTaskDelay(pdMS_TO_TICKS(3000));
        tick++;
    }
    return false;
}

bool network_init(void) {
    char cmd[128], resp[256];
    bool ok;

    modem_at("ATE0",      "OK", 2000, NULL, 0);
    modem_at("AT+CMEE=2", "OK", 2000, NULL, 0);

    /* SIM ready check */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+CPIN?", "READY", 5000, resp, sizeof(resp));
    if (!strstr(resp, "READY")) {
        ESP_LOGE(TAG, "SIM not ready: %s", resp);
        return false;
    }
    ESP_LOGI(TAG, "SIM: READY");

    /* Signal strength */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+CSQ", "+CSQ:", 3000, resp, sizeof(resp));
    char *p = strstr(resp, "+CSQ:");
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
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", MODEM_APN);
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
        snprintf(cmd, sizeof(cmd), "AT+QICSGP=1,1,\"%s\",\"\",\"\",0", MODEM_APN);
        modem_at(cmd, "OK", 5000, NULL, 0);
        ok = modem_at("AT+QIACT=1", "OK", 60000, NULL, 0);
        ESP_LOGI(TAG, "QIACT=1: %s", ok ? "OK" : "FAIL");
    }

    /* Log assigned IP */
    memset(resp, 0, sizeof(resp));
    modem_at("AT+CGPADDR=1", "+CGPADDR:", 5000, resp, sizeof(resp));
    ESP_LOGI(TAG, "IP: %s", resp);

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
