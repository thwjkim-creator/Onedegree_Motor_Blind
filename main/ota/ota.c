#include <stdio.h>
#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "ota.h"

#define OTA_VERSION_URL  "https://raw.githubusercontent.com/thwjkim-creator/Onedegree_Motor_Blind/main/version/version.txt"
#define OTA_FIRMWARE_URL "https://raw.githubusercontent.com/thwjkim-creator/Onedegree_Motor_Blind/main/version/firmware.bin"

static const char *TAG = "ota";

/* "0.0.1" → 10001, "1.2.3" → 10203 형태로 변환해 크기 비교 */
static int parse_version(const char *ver)
{
    int major = 0, minor = 0, patch = 0;
    sscanf(ver, "%d.%d.%d", &major, &minor, &patch);
    return major * 10000 + minor * 100 + patch;
}

void ota_check_and_update(void)
{
    ESP_LOGI(TAG, "버전 확인 중...");

    esp_http_client_config_t http_cfg = {
        .url               = OTA_VERSION_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);

    char version_buf[16] = {0};

    if (esp_http_client_open(client, 0) == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int http_status = esp_http_client_get_status_code(client);
        if (http_status == 200) {
            int len = esp_http_client_read(client, version_buf, sizeof(version_buf) - 1);
            if (len > 0) version_buf[len] = '\0';
        } else {
            ESP_LOGW(TAG, "version.txt HTTP %d — OTA 생략", http_status);
        }
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "현재: %s  최신: %s",
             CURRENT_FIRMWARE_VERSION, version_buf[0] ? version_buf : "unknown");

    if (version_buf[0] && parse_version(version_buf) > parse_version(CURRENT_FIRMWARE_VERSION)) {
        ESP_LOGI(TAG, "신버전 감지 — HTTPS OTA 시작");
        esp_http_client_config_t ota_cfg = {
            .url               = OTA_FIRMWARE_URL,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .keep_alive_enable = true,
        };
        esp_https_ota_config_t config = { .http_config = &ota_cfg };
        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "OTA 완료 — 재부팅");
            esp_restart();
        } else {
            ESP_LOGW(TAG, "OTA 실패 (0x%x), 계속 실행", ret);
        }
    }
}
