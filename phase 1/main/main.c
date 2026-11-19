/*
Embedded project - Phase 1 (ESP32, ESP-IDF + FreeRTOS)
Malak Mekky
Malak Taha
Jannah Hella

Tasks:
  robot   (core 1, prio 5) : state machine, sensors, motors        -> robot.c
  ble_tx  (core 0, prio 4) : sends status lines to the PC         -> ble_link.c
  nimble  (core 0)         : BLE stack, receives PC commands      -> ble_link.c
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "ble_link.h"
#include "robot.h"

void app_main(void) {

  // flash storage, needed by BLE and for keeping the learned path
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ble_link_init();

  xTaskCreatePinnedToCore(robot_task, "robot", 6144, NULL, 5, NULL, 1);
};
