#pragma once

#include "mqtt_client.h"

/* MQTT 명령 페이로드 구조체 */
typedef struct {
    float rot;  /* 회전 수 */
    int   dir;  /* 1=정방향  -1=역방향  0=정지 */
    int   spd;  /* 목표 속도 1~100% */
} motor_cmd_t;

/* 모터 태스크 및 MQTT 연결 초기화.
 * mqtt_client는 이미 esp_mqtt_client_init()된 핸들이어야 합니다.
 * 이 함수는 이벤트 핸들러를 등록하고 FreeRTOS 태스크를 생성합니다.
 * 호출 후 app_main에서 esp_mqtt_client_start()를 호출하세요. */
void motor_task_start(const char *sub_topic, const char *pub_topic,
                      esp_mqtt_client_handle_t mqtt_client);
