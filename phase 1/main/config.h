/*
 * config.h - every pin and tuning value in one place.
 * Values marked (phase 0) are unchanged from the tuned Arduino code.
 */
#pragma once

#include "driver/gpio.h"

// ---------------- pins ----------------

// L298N (left motors on ENA/IN1/IN2, right motors on ENB/IN3/IN4)
#define PIN_ENA GPIO_NUM_21
#define PIN_IN1 GPIO_NUM_19
#define PIN_IN2 GPIO_NUM_18
#define PIN_IN3 GPIO_NUM_22
#define PIN_IN4 GPIO_NUM_23
#define PIN_ENB GPIO_NUM_4

// HW-871 sensor board, leftmost to rightmost = OUT1..OUT5 (board powered from 3.3V)
#define PIN_SENSOR_1 GPIO_NUM_25   // leftmost
#define PIN_SENSOR_2 GPIO_NUM_33
#define PIN_SENSOR_3 GPIO_NUM_32   // center
#define PIN_SENSOR_4 GPIO_NUM_35
#define PIN_SENSOR_5 GPIO_NUM_34   // rightmost

// ---------------- PWM ----------------
#define PWM_FREQ_HZ 1000           // close to the Mega's 980 Hz, so speeds behave the same

// ---------------- speeds (0-255) ----------------
#define NORMAL_SPEED       95     // (phase 0: 120) slower = fewer drift-offs and missed intersections
#define TURN_SPEED         125     // <-- CHECK: keep the value you are using now
#define SCAN_SPEED          90     // (phase 0)
#define KICK_SPEED         190     // <-- CHECK: keep the value you are using now
#define FOLLOW_MAX_SPEED   135     // (phase 0: 160) keeps the same steering headroom above NORMAL_SPEED
#define SEARCH_SPEED       120     // <-- CHECK: keep the value you are using now
#define SOLVE_SPEED_BOOST    0     // extra speed during the solve run, raise once it's reliable

// ---------------- timings (ms) ----------------
#define KICK_TIME           80     // (phase 0)
#define PIVOT_CREEP_TIME    50     // (phase 0)
#define BRAKE_TIME          30     // (phase 0)
#define NODE_TIME          130     // (phase 0) forward scan over an intersection
#define NODE_CONFIRM_TIME   15     // (phase 0)
#define MIN_TURN_TIME      180     // (phase 0)
#define MAX_TURN_TIME     2200     // (phase 0)
#define TURN_STABLE_TIME    30     // (phase 0) center must stay on the line this long
#define COOLDOWN_TIME      300     // (phase 0) min time between two intersections
#define LOST_TIME          120     // (phase 0)
#define TURN_PROTECTION_TIME 350   // (phase 0)
#define ALIGN_TIME          30     // (phase 0)
#define MAX_SEARCH_TIME   6000     // (phase 0)
#define RETRY_AFTER       1500     // (phase 0)
#define MAX_SEARCH_TRIES     3     // give up (stop + report) after this many failed searches

#define END_CONFIRM_TIME    80     // extra creep to confirm the end circle (all 5 black)
#define START_TIMEOUT     3000     // max time to drive out of the start circle
#define START_DELAY       1000     // pause after TRAIN/SOLVE is received
#define TRAIN_MAX_TIME  150000     // once the end circle is known, stop exploring after this long and use the map so far
#define UTURN_MAX_TIME    4000     // max time for a 180 degree spin
#define TURN90_GUESS_MS    500     // first guess of a 90 degree spin time, learned while running
#define CORNER_RATIO       1.5f    // spin shorter than 1.5 x (90 deg time) = it was a corner, not a dead end
#define LINE_GAP_MS         50     // while spinning, center must be white this long between two lines (phase 0: 15)
#define CIRCLE_REVERSE_EXTRA 250   // after backing out of a circle onto the line, keep reversing this long
#define CIRCLE_REVERSE_TIMEOUT 2500

// ---------------- maze map ----------------
#define MATCH_TOL_FRAC     0.25f   // a new edge ends at a known node if within 25% of the distance driven...
#define MATCH_TOL_MIN       250    // ...or 250 ms, whichever is bigger
#define MATCH_EXIT_DIFF       1    // a known node may differ by at most 1 exit from what the sensors see now
#define CIRCLE_TOL_FRAC    0.30f   // before the end is found, a circle this close to the start is the start again...
#define CIRCLE_TOL_MIN      400    // ...but at least 400 ms
#define CIRCLE_TOL_MAX     1000    // ...and at most 1000 ms (so the real end is never mistaken for the start)

// ---------------- line following ----------------
#define DRIFT_ERR          0.9f    // (phase 0) |error| above this before losing the line = drifted off
#define KP                15.0f    // (phase 0)
#define KI                 0.0f    // tune after KD
#define KD                 0.0f    // keep 0 (the D term is divided by dt = 5 ms, so even 0.5 is huge)
#define PID_PERIOD_MS        5     // I and D terms are updated at this fixed rate
#define INTEGRAL_LIMIT     2.0f

// ---------------- BLE ----------------
#define BLE_DEVICE_NAME "LineBot"

// ---------------- wiring check ----------------
//   0 = normal operation
//   1 = print sensors every 200 ms (leftmost first, 1 = black), motors off
//   2 = left wheels forward 2 s, then right wheels forward 2 s, repeat
#define TEST_MODE 0
