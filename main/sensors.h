/*
 * sensors.h - 5 channel digital line sensor (HW-871 / TCRT5000).
 * A reading is a 5 bit value, bit 0 = leftmost sensor, 1 = black.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ALL_BLACK   0b11111
#define CENTER_MASK 0b00100
#define MIDDLE_MASK 0b01110

void sensors_init(void);

uint8_t readLine(void);

// both left sensors black
bool leftBranch(uint8_t s);

// both right sensors black
bool rightBranch(uint8_t s);

// line centered under the car (one of the "normal" patterns)
bool normalLine(uint8_t s);

// position of the line, -2 (far left) .. +2 (far right)
// returns false if no sensor sees black
bool lineError(uint8_t s, float *error);
