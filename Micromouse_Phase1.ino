#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// 1. PIN CONFIGURATIONS & HARDWARE CONSTANTS
// ============================================================================

// TB6612FNG Motor Driver Pins
#define PIN_MOTOR_L_PWM  25
#define PIN_MOTOR_L_IN1  26
#define PIN_MOTOR_L_IN2  27

#define PIN_MOTOR_R_PWM  14
#define PIN_MOTOR_R_IN1  12
#define PIN_MOTOR_R_IN2  13

#define PIN_MOTOR_STBY   33

// Quadrature Encoder Pins
#define PIN_ENC_L_A      34
#define PIN_ENC_L_B      35
#define PIN_ENC_R_A      32
#define PIN_ENC_R_B      39

// User Interface & Peripherals
#define PIN_BUTTON       0
#define PIN_BUZZER       18

// OLED Screen Setup
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================================
// 2. ENCODER DRIVER & INTERRUPT SERVICE ROUTINES
// ============================================================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;

void IRAM_ATTR isrLeftEncoder() {
    if (digitalRead(PIN_ENC_L_B) == HIGH) {
        leftTicks++;
    } else {
        leftTicks--;
    }
}

void IRAM_ATTR isrRightEncoder() {
    if (digitalRead(PIN_ENC_R_B) == HIGH) {
        rightTicks++;
    } else {
        rightTicks--;
    }
}

void initEncoders() {
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightEncoder, RISING);
}

void resetEncoders() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
}

// ============================================================================
// 3. MOTOR DRIVER PRIMITIVES (UNIVERSAL ANALOGWRITE)
// ============================================================================

void initMotorDriver() {
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    pinMode(PIN_MOTOR_L_PWM, OUTPUT);
    pinMode(PIN_MOTOR_R_PWM, OUTPUT);

    // Disable motors on boot
    digitalWrite(PIN_MOTOR_STBY, LOW);
}

void enableMotors() {
    digitalWrite(PIN_MOTOR_STBY, HIGH);
}

void disableMotors() {
    digitalWrite(PIN_MOTOR_STBY, LOW);
}

void setLeftMotor(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite(PIN_MOTOR_L_PWM, speed);
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        analogWrite(PIN_MOTOR_L_PWM, -speed);
    } else {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite(PIN_MOTOR_L_PWM, 0);
    }
}

void setRightMotor(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        analogWrite(PIN_MOTOR_R_PWM, speed);
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
        analogWrite(PIN_MOTOR_R_PWM, -speed);
    } else {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        analogWrite(PIN_MOTOR_R_PWM, 0);
    }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    enableMotors();
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}

void moveForward(uint8_t speed) { setMotorSpeeds(speed, speed); }
void moveReverse(uint8_t speed) { setMotorSpeeds(-speed, -speed); }
void turnLeft(uint8_t speed)    { setMotorSpeeds(-speed, speed); }
void turnRight(uint8_t speed)   { setMotorSpeeds(speed, -speed); }

void stopMotors() {
    setLeftMotor(0);
    setRightMotor(0);
    disableMotors();
}

// ============================================================================
// 4. OLED & DIAGNOSTIC UTILITIES
// ============================================================================

void updateOLED(const char* statusMessage) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    display.println("--- MICROMOUSE P1 ---");
    display.setCursor(0, 16);
    display.printf("State: %s\n\n", statusMessage);
    
    noInterrupts();
    long currentLeft = leftTicks;
    long currentRight = rightTicks;
    interrupts();
    
    display.printf("Enc L: %ld\n", currentLeft);
    display.printf("Enc R: %ld\n", currentRight);
    display.display();
}

// ============================================================================
// 5. MAIN SETUP & EXECUTION ROUTINE
// ============================================================================

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);

    // Initialize OLED Display (I2C address 0x3C)
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("Error: SSD1306 OLED initialization failed. Check wiring."));
        for (;;);
    }

    initMotorDriver();
    initEncoders();

    updateOLED("Standby");

    // Audio cue: Ready for start trigger
    tone(PIN_BUZZER, 2000, 100);

    // Block execution until button (GPIO 0) is pressed
    while (digitalRead(PIN_BUTTON) == HIGH) {
        delay(10);
    }

    // Audio cue: Execution starting
    tone(PIN_BUZZER, 3000, 200);
    delay(500);
    
    resetEncoders();
}

void loop() {
    // --- Step 1: Forward Motion Test ---
    moveForward(150);
    updateOLED("FORWARD");
    delay(1500);

    // --- Step 2: Stop Test ---
    stopMotors();
    updateOLED("STOPPED");
    delay(1000);

    // --- Step 3: Pivot Right Test ---
    turnRight(130);
    updateOLED("TURN RIGHT");
    delay(600);

    // --- Step 4: Stop Test ---
    stopMotors();
    updateOLED("STOPPED");
    delay(1000);

    // --- Step 5: Pivot Left Test ---
    turnLeft(130);
    updateOLED("TURN LEFT");
    delay(600);

    // --- Step 6: Reverse Motion Test ---
    moveReverse(120);
    updateOLED("REVERSE");
    delay(1000);

    // --- Step 7: Sequence Complete & Continuous Telemetry ---
    stopMotors();
    updateOLED("COMPLETE");

    // Continuous monitoring loop for debugging quadrature tick counts
    while (true) {
        noInterrupts();
        long snapshotL = leftTicks;
        long snapshotR = rightTicks;
        interrupts();

        Serial.printf("[TELEMETRY] Left Encoders: %ld | Right Encoders: %ld\n", snapshotL, snapshotR);
        updateOLED("COMPLETED");
        delay(100);
    }
}