#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// IR SENSOR SUBSYSTEM
// ============================================================================

// Initialize IR sensor pins
void initIRSensors();

// Read raw IR sensor values and determine wall presence
void readIRSensors();

// Return true if a wall is detected on the left
bool isWallLeft();

// Return true if a wall is detected in front
bool isWallFront();

// Return true if a wall is detected on the right
bool isWallRight();

#endif
