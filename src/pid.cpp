#include "pid.h"

// ============================================================================
// PID CONTROLLER
// ============================================================================

static float driveIntegral = 0.0f;
static float turnIntegral  = 0.0f;

void initPID() {
    driveIntegral = 0.0f;
    turnIntegral = 0.0f;
}

void resetPID() {
    driveIntegral = 0.0f;
    turnIntegral = 0.0f;
}

float calculateDriveCorrection(float error, float dt) {
    // The current Micromouse controller primarily uses
    // proportional + derivative correction.
    // Integral is kept at zero to avoid wind-up.

    static float previousError = 0.0f;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    float derivative = (error - previousError) / dt;

    previousError = error;

    // These are the default drive PID values from config.h.
    const float KP = 2.5f;
    const float KD = 1.2f;

    float output = (KP * error) + (KD * derivative);

    return constrainPIDOutput(output, -255.0f, 255.0f);
}

float calculateTurnOutput(float error, float dt) {
    static float previousError = 0.0f;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    float derivative = (error - previousError) / dt;

    previousError = error;

    // These are the default turning PID values from config.h.
    const float KP = 2.8f;
    const float KD = 1.5f;

    float output = (KP * error) + (KD * derivative);

    return constrainPIDOutput(output, -200.0f, 200.0f);
}

float constrainPIDOutput(
    float output,
    float minOutput,
    float maxOutput
) {
    if (output < minOutput) {
        return minOutput;
    }

    if (output > maxOutput) {
        return maxOutput;
    }

    return output;
}
