/*
Embedded project - Phase 1 (ESP32, ESP-IDF + FreeRTOS)
Malak Mekky
Malak Taha
Jannah Hella

Tasks:
  robot   (core 1, prio 5) : state machine, sensors, motors        -> robot.c
  ble_tx  (core 0, prio 4) : sends status lines to the PC         -> ble_link.c
  nimble  (core 0)         : BLE stack, receives PC commands      -> ble_link.c

VERSION HISTORY (newest first, one entry per change)
  v1.15 2026-11-19  Line search back to the zip version: SEARCH_SPEED 130 -> 100 (130 swept
                    past the line) and removed the v1.2 RECOVER_PROTECTION_TIME rule, so a
                    lost line is searched / turned round exactly as in v1.0.
  v1.14 2026-11-19  Training no longer stops after failed turns (removed the v1.11
                    TRAIN_MAX_FAILED_TURNS check): it prints "WARN turn failed", searches
                    for the line and keeps exploring.
  v1.13 2026-11-19  Removed the v1.12 3 s wait for someone to fix the car by hand: when the
                    line is lost the car searches / turns round right away again (LOST_TIME).
  v1.12 2026-11-19  3 s to fix the car by hand: when the line is lost the car stops and waits
                    HAND_FIX_TIME (3 s, "WAIT" message). Put back on the line in time, it just
                    continues ("BACK on the line") with nothing stored; otherwise it searches /
                    turns round as before.
  v1.11 2026-11-19  Training stops after more than 2 failed turns (TRAIN_MAX_FAILED_TURNS)
                    instead of building a wrong map. STATUS also shows why the ESP32 last
                    restarted (BOOT line: BROWNOUT = power dip). Tuning for denser tracks:
                    MATCH_TOL_FRAC 0.30 -> 0.35, COOLDOWN_TIME 300 -> 200, NODE_DRIFT_ERR
                    1.0 -> 1.5, END_CONFIRM_TIME 80 -> 120.
  v1.10 2026-11-19  Turns fixed: TURN_SPEED 145 -> 120 and KICK_SPEED 180 -> 160. At 145/180
                    the car spun past the line (~20 failed turns in one training, so the map
                    and the solve path were wrong) and the ESP32 restarted during solve runs.
  v1.9  2026-11-19  PC app only (no reflash needed): prints a banner "END CIRCLE FOUND"
                    when training reaches the end circle for the first time.
  v1.8  2026-11-19  Fewer drift-offs: KP 15 -> 25 (stronger steering). Faster line search:
                    SEARCH_SPEED 100 -> 130. More time to map every point: TRAIN_MAX_TIME
                    90 s -> 180 s.
  v1.7  2026-11-19  Faster turns: TURN_SPEED 130 -> 145.
  v1.6  2026-11-19  Faster again: NORMAL_SPEED 95 -> 110, FOLLOW_MAX_SPEED 135 -> 150.
                    Stronger turn kick: KICK_SPEED 150 -> 180.
  v1.5  2026-11-19  Removed driving back to the start after training: the car stops
                    where training ends and waits.
  v1.4  2026-11-19  FORGET command back (car + PC app): erases the saved path and map.
  v1.3  2026-11-19  Drift is not a node: after creeping over a "branch", if the line is
                    still off to that side it was the car drifting, so nothing is saved
                    (NODE_DRIFT_ERR).
  v1.2  2026-11-19  Losing the line within 1 s of finding it again = search again, not a
                    dead end (RECOVER_PROTECTION_TIME). Training stops 90 s after the end
                    circle is found and uses the map so far (TRAIN_MAX_TIME).
  v1.1  2026-11-19  Start circle and end circle told apart (a circle near the start is the
                    start, never a third circle). Slower line following: NORMAL_SPEED
                    120 -> 95, FOLLOW_MAX_SPEED 160 -> 135.
  v1.0  2026-11-16  Handover zip version: graph map, depth-first exploration, Dijkstra
                    shortest path, BLE control.
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
}
