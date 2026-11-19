/*
Embedded project - Phase 1 (ESP32 / ESP-IDF port of phase 0 v29.ino)
Malak Mekky
Malak Taha
Jannah Hella

Same logic as the Arduino version. Only the hardware layer changed:
  digitalWrite/digitalRead -> gpio_set_level/gpio_get_level
  analogWrite              -> LEDC PWM (8 bit, 1 kHz, so the 0-255 speeds stay the same)
  millis()/delay()         -> esp_timer / vTaskDelay (1 ms tick, see sdkconfig.defaults)
  Serial.print             -> printf (idf.py monitor, 115200 baud)
*/

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

// motor driver pins (L298N)
#define ENA GPIO_NUM_21
#define IN1 GPIO_NUM_19
#define IN2 GPIO_NUM_18
#define IN3 GPIO_NUM_22
#define IN4 GPIO_NUM_23
#define ENB GPIO_NUM_4

#define HIGH 1
#define LOW  0

// PWM (LEDC) setup for ENA / ENB
#define PWM_MODE  LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_FREQ  1000
#define PWM_RES   LEDC_TIMER_8_BIT
#define PWM_CH_A  LEDC_CHANNEL_0
#define PWM_CH_B  LEDC_CHANNEL_1

// sensor pins (leftmost to rightmost = OUT1..OUT5), sensor board powered from 3.3V
// wired in board order: OUT5=34, OUT4=35, OUT3=32, OUT2=33, OUT1=25
static const gpio_num_t sensors[5] = {
  GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_35, GPIO_NUM_34
};

// speeds
static const int normalSpeed = 120;
static const int turnSpeed = 110;
static const int scanSpeed = 90;
static const int kickSpeed = 150;
static const unsigned long kickTime = 40;
static const int followMaxSpeed = 160;
static const int searchSpeed = 100;

// spin settings
static const float spinTrimLeft = 1.00;
static const float spinTrimRight = 1.00;
static const unsigned long pivotCreepTime = 50;
static const unsigned long brakeTime = 30;

// lost line settings
static const float driftErr = 0.9;
static const unsigned long lostTime = 120;
static const unsigned long maxSearchTime = 6000;
static const unsigned long alignTime = 30;
static const unsigned long turnProtectionTime = 350;
static const unsigned long retryAfter = 1500;

// turn settings
static const unsigned long nodeTime = 130;
static const unsigned long minTurnTime = 180;
static const unsigned long maxTurnTime = 2200;
static const unsigned long cooldownTime = 300;
static const unsigned long nodeConfirmTime = 15;

// line following
static const float kp = 15.0;

// state
static bool readyForNode = false;
static bool isSearching = false;
static bool searchFailed = false;
static bool turnCompleted = false;
static bool driftLoss = false;

static unsigned long lastNodeTime = 0;
static unsigned long lastTurnTime = 0;
static unsigned long lineLostTime = 0;
static unsigned long searchStartTime = 0;
static unsigned long centerStartTime = 0;
static unsigned long failedAt = 0;

static int lastTurnDirection = 1;
static float lastError = 0;

// ---------- Arduino-style helpers ----------

static unsigned long millis(void) {
  return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

static void delay(unsigned long ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static int constrain(int x, int lo, int hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static void pwmWrite(ledc_channel_t ch, int duty) {
  ledc_set_duty(PWM_MODE, ch, duty);
  ledc_update_duty(PWM_MODE, ch);
}

// ---------- motor control ----------

static void drive(int left, int right) {

  gpio_set_level(IN1, left >= 0 ? HIGH : LOW);
  gpio_set_level(IN2, left >= 0 ? LOW : HIGH);

  gpio_set_level(IN3, right >= 0 ? HIGH : LOW);
  gpio_set_level(IN4, right < 0 ? HIGH : LOW);

  pwmWrite(PWM_CH_A, constrain(abs(left), 0, 255));
  pwmWrite(PWM_CH_B, constrain(abs(right), 0, 255));
}

static void stopCar(void) {
  pwmWrite(PWM_CH_A, 0);
  pwmWrite(PWM_CH_B, 0);
}

// stops the motors fast so the car doesn't keep rolling
static void brakeCar(void) {
  gpio_set_level(IN1, HIGH);
  gpio_set_level(IN2, HIGH);
  gpio_set_level(IN3, HIGH);
  gpio_set_level(IN4, HIGH);
  pwmWrite(PWM_CH_A, 255);
  pwmWrite(PWM_CH_B, 255);
  delay(brakeTime);
  stopCar();
}

// spin on the spot (1 = right, -1 = left)
static void spin(int dir, int speed) {
  drive(dir * (int)(speed * spinTrimLeft),
       -dir * (int)(speed * spinTrimRight));
}

// ---------- sensors ----------

// sensor reading (black = 1)
static uint8_t readLine(void) {

  uint8_t sensorValues = 0;

  for (int i = 0; i < 5; i++) {
    if (gpio_get_level(sensors[i]) == LOW)
      sensorValues |= (1 << i);
  }

  return sensorValues;
}

static bool leftBranch(uint8_t sensorValues) {
  return (sensorValues & 0b00011) == 0b00011;
}

static bool rightBranch(uint8_t sensorValues) {
  return (sensorValues & 0b11000) == 0b11000;
}

// ---------- 90 degree turn ----------

static bool turn90(int direction) {

  stopCar();
  delay(60);

  // small push to start the spin
  spin(direction, kickSpeed);
  delay(kickTime);

  spin(direction, turnSpeed);

  unsigned long turnStart = millis();
  bool leftOldLine = false;
  unsigned long stableTime = 0;

  while (millis() - turnStart < maxTurnTime) {

    uint8_t sensorValues = readLine();
    bool centerDetected = sensorValues & 0b00100;

    if (!centerDetected) {
      leftOldLine = true;
      stableTime = 0;
    }

    if (leftOldLine &&
        millis() - turnStart > minTurnTime &&
        centerDetected) {

      if (stableTime == 0)
        stableTime = millis();

      if (millis() - stableTime > 30) {

        brakeCar();
        delay(70);

        lastTurnTime = millis();
        turnCompleted = true;

        return true;
      }
    }
    else if (!centerDetected) {
      stableTime = 0;
    }
  }

  stopCar();
  return false;
}

// ---------- search for the line when it is lost ----------

static void recoverLine(void) {

  if (!isSearching) {

    isSearching = true;
    printf("line lost -> searching\n");
    searchStartTime = millis();
    centerStartTime = 0;
    readyForNode = false;

    // big error before losing it = drifted off, otherwise the line ended
    driftLoss = fabsf(lastError) >= driftErr;
    printf(driftLoss ? "drifted off -> small sweeps\n" : "line ended -> full spin\n");

    stopCar();
  }

  uint8_t sensorValues = readLine();

  bool centerDetected = (sensorValues & 0b00100) != 0;

  // center sensor found the line
  if (centerDetected) {

    if (centerStartTime == 0) {
      brakeCar();
      centerStartTime = millis();
    }
    stopCar();

    if (millis() - centerStartTime >= alignTime) {

      isSearching = false;
      centerStartTime = 0;
      lineLostTime = 0;
      lastNodeTime = millis();
      printf("line found\n");

      return;
    }

  } else {

    centerStartTime = 0;

    int dir = lastTurnDirection;

    // if we only drifted, sweep one way then the other
    if (driftLoss) {
      unsigned long t = millis() - searchStartTime;
      if (t >= 300 && t < 900)
        dir = -lastTurnDirection;
    }

    spin(dir, searchSpeed);
  }

  // give up after max time, retry later
  if (millis() - searchStartTime >= maxSearchTime) {

    stopCar();
    isSearching = false;
    searchFailed = true;
    failedAt = millis();
    printf("search failed -> will retry\n");
  }
}

// ---------- setup ----------

static void setup(void) {

  // direction pins
  gpio_config_t out = {
    .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&out);

  // sensor pins (the sensor board drives them, no internal pulls needed)
  uint64_t inMask = 0;
  for (int i = 0; i < 5; i++)
    inMask |= (1ULL << sensors[i]);

  gpio_config_t in = {
    .pin_bit_mask = inMask,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&in);

  // PWM for ENA / ENB
  ledc_timer_config_t timer = {
    .speed_mode = PWM_MODE,
    .timer_num = PWM_TIMER,
    .duty_resolution = PWM_RES,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer);

  ledc_channel_config_t chA = {
    .gpio_num = ENA,
    .speed_mode = PWM_MODE,
    .channel = PWM_CH_A,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&chA);

  ledc_channel_config_t chB = chA;
  chB.gpio_num = ENB;
  chB.channel = PWM_CH_B;
  ledc_channel_config(&chB);

  stopCar();

  delay(3000);

  lastNodeTime = millis();
}

// ---------- loop ----------

static void loop(void) {

  // wait after a failed search then try again
  if (searchFailed) {
    stopCar();
    if (readLine() != 0 || millis() - failedAt > retryAfter) {
      searchFailed = false;
      lineLostTime = 0;
      readyForNode = false;
      lastNodeTime = millis();
      printf("resuming\n");
    }
    return;
  }

  uint8_t sensorValues = readLine();

  if (isSearching) {
    recoverLine();
    return;
  }

  // no black under the car
  if (sensorValues == 0) {

    stopCar();

    // ignore it right after a turn
    if (turnCompleted &&
        millis() - lastTurnTime < turnProtectionTime) {

      lineLostTime = 0;
      return;
    }

    if (lineLostTime == 0)
      lineLostTime = millis();

    if (millis() - lineLostTime >= lostTime) {
      recoverLine();
    }

    return;
  }

  lineLostTime = 0;

  // normal line seen, so we are ready for the next intersection
  if (sensorValues == 0b00100 ||
      sensorValues == 0b00110 ||
      sensorValues == 0b01100 ||
      sensorValues == 0b01110) {

    readyForNode = true;
  }

  // intersection
  if (readyForNode &&
      millis() - lastNodeTime > cooldownTime &&
      (leftBranch(sensorValues) || rightBranch(sensorValues))) {

    bool leftDetected = leftBranch(sensorValues);
    bool rightDetected = rightBranch(sensorValues);

    // read again to make sure it is real
    delay(nodeConfirmTime);

    uint8_t newReading = readLine();

    bool turnLeft = leftDetected && leftBranch(newReading);
    bool turnRight = rightDetected && rightBranch(newReading);

    if (turnLeft || turnRight) {

      // move forward slowly and check for branches
      drive(scanSpeed, scanSpeed);

      unsigned long scanStart = millis();

      while (millis() - scanStart < nodeTime) {

        uint8_t currentReading = readLine();

        if (leftBranch(currentReading)) turnLeft = true;
        if (rightBranch(currentReading)) turnRight = true;
      }

      // move a bit more so the wheels are in the middle of the intersection
      delay(pivotCreepTime);
      brakeCar();

      int direction = 0;

      // right has priority
      if (turnRight)
        direction = 1;
      else if (turnLeft)
        direction = -1;

      printf("node L=%d R=%d -> %s\n", turnLeft, turnRight,
             direction == 1 ? "RIGHT" : direction == -1 ? "LEFT" : "NONE");

      if (direction != 0) {

        lastTurnDirection = direction;

        if (!turn90(direction)) {
          printf("turn failed -> recovering\n");
          stopCar();
        }
      }

      readyForNode = false;
      lastNodeTime = millis();
      lastError = 0;
      return;
    }
  }

  // follow the line
  int totalPosition = 0;
  int activeSensors = 0;

  for (int i = 0; i < 5; i++) {

    if (sensorValues & (1 << i)) {
      totalPosition += i - 2;
      activeSensors++;
    }
  }

  if (activeSensors > 0) {

    float error = (float)totalPosition / activeSensors;
    lastError = error;

    if (error < -0.3)
      lastTurnDirection = -1;

    else if (error > 0.3)
      lastTurnDirection = 1;

    int correction = (int)(kp * error);

    int leftSpeed = normalSpeed + correction;
    int rightSpeed = normalSpeed - correction;

    drive(
      constrain(leftSpeed, 0, followMaxSpeed),
      constrain(rightSpeed, 0, followMaxSpeed)
    );
  }
}

// ---------- entry point ----------

// wiring check, set TEST_MODE then flash + monitor (wheels off the ground)
//   0 = normal run
//   1 = print sensors, leftmost first (1 = black), motors off
//   2 = left wheels forward 2s, then right wheels forward 2s, repeat
#define TEST_MODE 0

static void sensorTest(void) {
  while (1) {
    uint8_t s = readLine();
    printf("L ");
    for (int i = 0; i < 5; i++)
      printf("%d ", (s >> i) & 1);
    printf("R\n");
    delay(200);
  }
}

static void motorTest(void) {
  while (1) {
    printf("LEFT wheels forward\n");
    drive(130, 0);
    delay(2000);
    stopCar();
    delay(1000);

    printf("RIGHT wheels forward\n");
    drive(0, 130);
    delay(2000);
    stopCar();
    delay(1000);
  }
}

// runs setup() once then loop() forever, like the Arduino core does
static void robotTask(void *arg) {
  setup();

  if (TEST_MODE == 1) sensorTest();
  if (TEST_MODE == 2) motorTest();

  while (1) {
    loop();
  }
}

void app_main(void) {
  // pinned to core 1 so it never starves core 0 (WiFi/BLE stack lives there in phase 1)
  xTaskCreatePinnedToCore(robotTask, "robot", 4096, NULL, 5, NULL, 1);
}
