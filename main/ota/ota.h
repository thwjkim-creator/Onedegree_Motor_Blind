#pragma once

/* 현재 펌웨어 버전 — OTA 업데이트 시 patch 번호를 증가시킵니다. */
#define CURRENT_FIRMWARE_VERSION  "0.0.9"

/* GitHub version.txt와 비교해 신버전이면 HTTPS OTA를 수행하고 재부팅합니다. */
void ota_check_and_update(void);
