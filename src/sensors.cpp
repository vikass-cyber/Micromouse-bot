#include "sensors.h"

// ============================================================================
// IR SENSOR INITIALIZATION
// ============================================================================

void initIRSensors() {
    pinMode(PIN_IR_LEFT, INPUT);
    pinMode(PIN_IR_FRONT, INPUT);
    pinMode(PIN_IR_RIGHT, INPUT);
}

// ============================================================================
// IR SENSOR READING
// ============================================================================

void readIRSensors() {
    const int samples = 4;

    long sumL = 0;
    long sumF = 0;
    long sumR = 0;

    for (int i = 0; i < samples; i++) {
        sumL += analogRead(PIN_IR_LEFT);
        sumF += analogRead(PIN_IR_FRONT);
        sumR += analogRead(PIN_IR_RIGHT);
    }

    ir.rawL = sumL / samples;
    ir.rawF = sumF / samples;
    ir.rawR = sumR / samples;

    ir.wallL = (ir.rawL > WALL_THRESH_LEFT);
    ir.wallF = (ir.rawF > WALL_THRESH_FRONT);
    ir.wallR = (ir.rawR > WALL_THRESH_RIGHT);
}

// ============================================================================
// WALL STATUS HELPERS
// ============================================================================

bool isWallLeft() {
    return ir.wallL;
}

bool isWallFront() {
    return ir.wallF;
}

bool isWallRight() {
    return ir.wallR;
}
