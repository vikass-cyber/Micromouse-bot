#include "motors.h"

// ============================================================================
// MOTOR DRIVER INITIALIZATION
// ============================================================================

void initMotors()
{
    pinMode(PIN_MOTOR_L_PWM, OUTPUT);
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);

    pinMode(PIN_MOTOR_R_PWM, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);

    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Start in a safe stopped state
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);

    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);

    digitalWrite(PIN_MOTOR_STBY, LOW);
}

// ============================================================================
// SET MOTOR SPEEDS
// ============================================================================

void setMotors(int leftSpeed, int rightSpeed)
{
    // Limit PWM values
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Enable motor driver
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    // ------------------------------------------------------------------------
    // LEFT MOTOR
    // ------------------------------------------------------------------------

    if (leftSpeed > 0)
    {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite(PIN_MOTOR_L_PWM, leftSpeed);
    }
    else if (leftSpeed < 0)
    {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        analogWrite(PIN_MOTOR_L_PWM, -leftSpeed);
    }
    else
    {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite(PIN_MOTOR_L_PWM, 0);
    }

    // ------------------------------------------------------------------------
    // RIGHT MOTOR
    // ------------------------------------------------------------------------

    if (rightSpeed > 0)
    {
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        analogWrite(PIN_MOTOR_R_PWM, rightSpeed);
    }
    else if (rightSpeed < 0)
    {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
        analogWrite(PIN_MOTOR_R_PWM, -rightSpeed);
    }
    else
    {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        analogWrite(PIN_MOTOR_R_PWM, 0);
    }
}

// ============================================================================
// STOP MOTORS
// ============================================================================

void stopMotors()
{
    // Stop both motors
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);

    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);

    // Disable TB6612FNG
    digitalWrite(PIN_MOTOR_STBY, LOW);
}
