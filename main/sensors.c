/*
 * sensors.c - reads the 5 sensor outputs (LOW = black).
 */
#include "driver/gpio.h"

#include "config.h"
#include "sensors.h"

static const gpio_num_t sensorPins[5] = {
  PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5
};

void sensors_init(void) {

  uint64_t mask = 0;
  for (int i = 0; i < 5; i++)
    mask |= (1ULL << sensorPins[i]);

  // the sensor board drives the pins, and 34/35 have no internal pulls anyway
  gpio_config_t in = {
    .pin_bit_mask = mask,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&in);
}

uint8_t readLine(void) {

  uint8_t s = 0;

  for (int i = 0; i < 5; i++) {
    if (gpio_get_level(sensorPins[i]) == 0)
      s |= (1 << i);
  }

  return s;
}

bool leftBranch(uint8_t s) {
  return (s & 0b00011) == 0b00011;
}

bool rightBranch(uint8_t s) {
  return (s & 0b11000) == 0b11000;
}

bool normalLine(uint8_t s) {
  return s == 0b00100 || s == 0b00110 || s == 0b01100 || s == 0b01110;
}

bool lineError(uint8_t s, float *error) {

  int total = 0;
  int active = 0;

  for (int i = 0; i < 5; i++) {
    if (s & (1 << i)) {
      total += i - 2;
      active++;
    }
  }

  if (active == 0)
    return false;

  *error = (float)total / active;
  return true;
}
