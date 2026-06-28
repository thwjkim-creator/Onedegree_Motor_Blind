idf_component_register(
    SRCS "main.c"
         "wifi/wifi_prov.c"
         "ota/ota.c"
         "motor/motor.c"
         "motor/motor_task.c"
         "encoder/encoder.c"
    INCLUDE_DIRS "." "wifi" "ota" "motor" "encoder"
    REQUIRES nvs_flash esp_wifi esp_event esp_netif mqtt json driver
             wifi_provisioning protocomm bt
             app_update esp_https_ota esp_http_client
    EMBED_TXTFILES "certs/github_root_cert.pem"
)
