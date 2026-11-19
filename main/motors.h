/*
 * motors.h - L298N driver (two PWM enables + four direction pins).
 */
#pragma once

void motors_init(void);

// signed speeds, -255..255 (negative = backwards)
void drive(int left, int right);

// cut PWM, wheels coast
void stopCar(void);

// short both motors for BRAKE_TIME so the car stops fast
void brakeCar(void);

// spin on the spot, dir 1 = right (clockwise), -1 = left
void spin(int dir, int speed);
