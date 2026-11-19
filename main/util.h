/*
 * util.h - small Arduino-style helpers so the logic reads like phase 0.
 */
#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static inline uint32_t millis(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// needs CONFIG_FREERTOS_HZ=1000 (sdkconfig.defaults) for 1 ms resolution
static inline void delay_ms(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline int constrain(int x, int lo, int hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
