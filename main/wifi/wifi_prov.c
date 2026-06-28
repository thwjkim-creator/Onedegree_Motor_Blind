#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
#include "wifi_prov.h"

#define PROV_NAME_PREFIX    "PROV_"
#define WIFI_CONNECTED_BIT  BIT0

static const char *TAG = "wifi_prov";
static EventGroupHandle_t s_wifi_eg;
static bool s_prov_done = false;  /* 프로비저닝 완료 후에만 WiFi 자동 재연결 허용 */

static void prov_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (id == WIFI_PROV_CRED_FAIL) {
        wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)data;
        ESP_LOGW(TAG, "WiFi 연결 실패 (%s) — 재입력 대기",
                 (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "비밀번호 오류" : "AP 없음");
        wifi_prov_mgr_reset_sm_state_on_failure();
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_prov_done) {
            ESP_LOGW(TAG, "끊김 – 재연결 시도");
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

void wifi_prov_init(void)
{
    // 1. 네트워크 스택 초기화
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 2. Wi-Fi 드라이버 초기화
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 3. 이벤트 핸들러 등록
    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                wifi_event_handler, NULL));

    // 4. 프로비저닝 이벤트 핸들러 등록 (비밀번호 오류 시 재시도 지원)
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID,
                                                prov_event_handler, NULL));

    // 5. BLE 프로비저닝 매니저 초기화
    wifi_prov_mgr_config_t prov_cfg = {
        .scheme               = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    // 6. 프로비저닝 여부 확인
    wifi_prov_mgr_reset_provisioning();  /* WiFi 변경 시: 주석 해제 → 플래시 → 앱 재프로비저닝 → 다시 주석 */
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        char prov_name[32];
        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        snprintf(prov_name, sizeof(prov_name), "%s%02X%02X%02X",
                 PROV_NAME_PREFIX, mac[3], mac[4], mac[5]);
        ESP_LOGI(TAG, "BLE 프로비저닝 시작: %s  (ESP BLE Prov 앱 사용)", prov_name);
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1,
                                                          NULL, prov_name, NULL));
    } else {
        wifi_prov_mgr_deinit();
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "저장된 자격증명으로 WiFi 연결 중...");
    }

    // 7. IP 획득까지 대기
    xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    s_prov_done = true;  /* 이후 WiFi 끊김 시 자동 재연결 활성화 */
    // BLE 경로에서만 deinit 필요 (이미 프로비저닝된 경로는 else에서 이미 호출함)
    if (!provisioned) {
        wifi_prov_mgr_deinit();
    }
}
