#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// 1. MOTOR DRIVER PINS - TB6612FNG
// ============================================================================

#define PIN_MOTOR_L_PWM   25
#define PIN_MOTOR_L_IN1   26
#define PIN_MOTOR_L_IN2   27

#define PIN_MOTOR_R_PWM   14
#define PIN_MOTOR_R_IN1   12
#define PIN_MOTOR_R_IN2   13

#define PIN_MOTOR_STBY    5

// ============================================================================
// 2. ENCODERS
// ============================================================================

#define PIN_ENC_L_A       34
#define PIN_ENC_L_B       35
#define PIN_ENC_R_A       32
#define PIN_ENC_R_B       39

// ============================================================================
// 3. IR DISTANCE SENSORS
// ============================================================================

#define PIN_IR_LEFT       36
#define PIN_IR_FRONT      33
#define PIN_IR_RIGHT      15

// ============================================================================
// 4. BATTERY & USER CONTROLS
// ============================================================================

#define PIN_BATTERY_ADC   4

#define PIN_BUTTON_NEXT   0
#define PIN_BUTTON_SELECT 16
#define PIN_BUZZER        18

// ============================================================================
// 5. DISPLAY / MPU6050
// ============================================================================

#define MPU6050_ADDR      0x68

#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64

// ============================================================================
// 6. MAZE
// ============================================================================

#define MAZE_SIZE         16
#define MAX_QUEUE_SIZE    256

// ============================================================================
// 7. PHYSICAL PARAMETERS
// ============================================================================

#define WHEEL_DIAMETER_MM 32.0f
#define TICKS_PER_REV     360.0f

#define MM_PER_TICK \
    ((3.14159265359f * WHEEL_DIAMETER_MM) / TICKS_PER_REV)

#define CELL_DISTANCE_MM 180.0f

// ============================================================================
// 8. SENSOR THRESHOLDS
// ============================================================================

#define WALL_THRESH_LEFT   800
#define WALL_THRESH_FRONT  1200
#define WALL_THRESH_RIGHT  800

// ============================================================================
// 9. MAZE WALL BITMASK
// ============================================================================

#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08
#define VISITED 0x10

// ============================================================================
// 10. ENUMERATIONS
// ============================================================================

enum Direction
{
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

enum MotionCmd
{
    CMD_FORWARD_1,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_TURN_180,
    CMD_STOP
};

enum SystemState
{
    MENU_MAIN,
    STATE_SEARCH_RUN,
    STATE_FAST_RUN,
    STATE_CALIBRATION,
    STATE_DIAGNOSTICS,
    STATE_ERROR
};

// ============================================================================
// 11. DATA STRUCTURES
// ============================================================================

struct Point
{
    uint8_t x;
    uint8_t y;
};

struct Configuration
{
    float kp_drive   = 2.5f;
    float kd_drive   = 1.2f;

    float kp_turn    = 2.8f;
    float kd_turn    = 1.5f;

    int search_speed = 150;
    int fast_speed   = 230;

    float accel_rate = 5.0f;
};

struct IRSensors
{
    int rawL;
    int rawF;
    int rawR;

    bool wallL;
    bool wallF;
    bool wallR;
};

struct MPUData
{
    float gyroZ_offset = 0.0f;
    float gz           = 0.0f;
    float yaw          = 0.0f;

    unsigned long lastTime = 0;

    bool initialized = false;
};

// ============================================================================
// 12. BATTERY
// ============================================================================

const float BATTERY_ADC_FACTOR =
    (3.3f / 4095.0f) * ((10.0f + 3.3f) / 3.3f);

const float MIN_BATTERY_VOLTAGE = 6.8f;

// ============================================================================
// 13. GLOBAL OBJECTS
// ============================================================================

extern Adafruit_SSD1306 display;
extern Preferences nvram;

extern volatile long leftTicks;
extern volatile long rightTicks;

extern Configuration config;
extern IRSensors ir;
extern MPUData mpu;

extern SystemState currentState;

extern uint8_t posX;
extern uint8_t posY;

extern Direction heading;

#endif
