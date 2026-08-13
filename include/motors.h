#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

// Initialize motor driver pins
void initMotors();

// Set left and right motor speeds.
// Range: -255 to +255
// Positive = forward, negative = reverse.
void setMotors(int leftSpeed, int rightSpeed);

// Stop both motors and disable the driver.
void stopMotors();

#endif // MOTORS_H
