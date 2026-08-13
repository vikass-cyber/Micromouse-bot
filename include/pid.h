#ifndef PID_H
#define PID_H

#include <Arduino.h>

// Initialize PID controller state
void initPID();

// Reset PID internal errors
void resetPID();

// Drive straight using wall/IMU correction
float calculateDriveCorrection(
    float error,
    float dt
);

// Calculate turning PID output
float calculateTurnOutput(
    float error,
    float dt
);

// Limit PID output to safe range
float constrainPIDOutput(
    float output,
    float minOutput,
    float maxOutput
);

#endif
