/*
 * motors.c - L298N driver using LEDC PWM (8 bit, so speeds stay 0-255 like analogWrite).
 */
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "config.h"
#include "util.h"
#include "motors.h"

#define PWM_MODE  LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_CH_A  LEDC_CHANNEL_0
#define PWM_CH_B  LEDC_CHANNEL_1

static void pwmWrite(ledc_channel_t ch, int duty) {
  ledc_set_duty(PWM_MODE, ch, duty);
  ledc_update_duty(PWM_MODE, ch);
}

void motors_init(void) {

  gpio_config_t out = {
    .pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2) |
                    (1ULL << PIN_IN3) | (1ULL << PIN_IN4),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&out);

  ledc_timer_config_t timer = {
    .speed_mode = PWM_MODE,
    .timer_num = PWM_TIMER,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .freq_hz = PWM_FREQ_HZ,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer);

  ledc_channel_config_t chA = {
    .gpio_num = PIN_ENA,
    .speed_mode = PWM_MODE,
    .channel = PWM_CH_A,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&chA);

  ledc_channel_config_t chB = chA;
  chB.gpio_num = PIN_ENB;
  chB.channel = PWM_CH_B;
  ledc_channel_config(&chB);

  stopCar();
}

void drive(int left, int right) {

  gpio_set_level(PIN_IN1, left >= 0 ? 1 : 0);
  gpio_set_level(PIN_IN2, left >= 0 ? 0 : 1);

  gpio_set_level(PIN_IN3, right >= 0 ? 1 : 0);
  gpio_set_level(PIN_IN4, right < 0 ? 1 : 0);

  pwmWrite(PWM_CH_A, constrain(abs(left), 0, 255));
  pwmWrite(PWM_CH_B, constrain(abs(right), 0, 255));
}

void stopCar(void) {
  pwmWrite(PWM_CH_A, 0);
  pwmWrite(PWM_CH_B, 0);
}

void brakeCar(void) {
  gpio_set_level(PIN_IN1, 1);
  gpio_set_level(PIN_IN2, 1);
  gpio_set_level(PIN_IN3, 1);
  gpio_set_level(PIN_IN4, 1);
  pwmWrite(PWM_CH_A, 255);
  pwmWrite(PWM_CH_B, 255);
  delay_ms(BRAKE_TIME);
  stopCar();
}

void spin(int dir, int speed) {
  drive(dir * speed, -dir * speed);
}
