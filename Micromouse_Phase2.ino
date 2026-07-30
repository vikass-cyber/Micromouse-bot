#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// 1. PIN DEFINITIONS & CONSTANTS
// ============================================================================

// TB6612FNG Motors
#define PIN_MOTOR_L_PWM  25
#define PIN_MOTOR_L_IN1  26
#define PIN_MOTOR_L_IN2  27
#define PIN_MOTOR_R_PWM  14
#define PIN_MOTOR_R_IN1  12
#define PIN_MOTOR_R_IN2  13
#define PIN_MOTOR_STBY   33

// Quadrature Encoders
#define PIN_ENC_L_A      34
#define PIN_ENC_L_B      35
#define PIN_ENC_R_A      32
#define PIN_ENC_R_B      39

// IR Analog Sensors
#define PIN_IR_LEFT      36 // VP
#define PIN_IR_FRONT     39 // VN (Shared with ENC_R_B if needed, remapped to 39/34)
#define PIN_IR_RIGHT     35 // Shared / ADC1 pins
#define PIN_IR_FRONT_L   15
#define PIN_IR_FRONT_R   4

// Peripherals
#define PIN_BUTTON       0
#define PIN_BUZZER       18
#define MPU6050_ADDR     0x68

// Kinematic Constants
#define WHEEL_DIAMETER_MM 32.0f
#define TICKS_PER_REV     360.0f
#define MM_PER_TICK       ((3.14159265f * WHEEL_DIAMETER_MM) / TICKS_PER_REV)
#define CELL_DISTANCE_MM  180.0f

// OLED Display
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================================
// 2. GLOBAL STATE & SENSOR STRUCTURES
// ============================================================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;

struct IRSensors {
    int rawL, rawF, rawR;
    int calibL, calibF, calibR;
    bool wallL, wallF, wallR;
} ir;

struct MPUData {
    float gyroZ_offset = 0.0f;
    float gz = 0.0f;
    float yaw = 0.0f;
    unsigned long lastTime = 0;
} mpu;

// PID Tuning Constants
struct PIDGains {
    float Kp = 2.5f;
    float Ki = 0.0f;
    float Kd = 1.2f;
} pidDrive, pidTurn;

// Thresholds for Wall Detection
const int WALL_THRESHOLD_L = 800;
const int WALL_THRESHOLD_F = 1200;
const int WALL_THRESHOLD_R = 800;
const int TARGET_CENTER_DIFF = 0; // Ideal IR_L - IR_R when perfectly centered

// ============================================================================
// 3. INTERRUPT SERVICE ROUTINES
// ============================================================================

void IRAM_ATTR isrLeftEncoder() {
    if (digitalRead(PIN_ENC_L_B) == HIGH) leftTicks++;
    else leftTicks--;
}

void IRAM_ATTR isrRightEncoder() {
    if (digitalRead(PIN_ENC_R_B) == HIGH) rightTicks++;
    else rightTicks--;
}

// ============================================================================
// 4. LOW-LEVEL HARDWARE DRIVERS
// ============================================================================

void initHardware() {
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    pinMode(PIN_MOTOR_L_PWM, OUTPUT);
    pinMode(PIN_MOTOR_R_PWM, OUTPUT);
    digitalWrite(PIN_MOTOR_STBY, LOW);

    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightEncoder, RISING);

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);

    // I2C & Display
    Wire.begin(21, 22);
    Wire.setClock(400000); // 400kHz Fast I2C
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for (;;);
    }
}

void setMotors(int leftSpeed, int rightSpeed) {
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed > 0) {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite(PIN_MOTOR_L_PWM, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        analogWrite(PIN_MOTOR_L_PWM, -leftSpeed);
    } else {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite(PIN_MOTOR_L_PWM, 0);
    }

    if (rightSpeed > 0) {
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        analogWrite(PIN_MOTOR_R_PWM, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
        analogWrite(PIN_MOTOR_R_PWM, -rightSpeed);
    } else {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        analogWrite(PIN_MOTOR_R_PWM, 0);
    }
}

void stopMotors() {
    setMotors(0, 0);
    digitalWrite(PIN_MOTOR_STBY, LOW);
}

// ============================================================================
// 5. MPU6050 & IR SENSOR PROCESSING
// ============================================================================

void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0);    // Wake up
    Wire.endTransmission(true);

    // Calibrate Gyro Z offset
    long gyroSum = 0;
    const int samples = 500;
    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 2, true);
        int16_t rawGZ = (Wire.read() << 8) | Wire.read();
        gyroSum += rawGZ;
        delay(2);
    }
    mpu.gyroZ_offset = (float)gyroSum / (float)samples;
    mpu.lastTime = micros();
}

void updateYaw() {
    unsigned long now = micros();
    float dt = (now - mpu.lastTime) / 1000000.0f;
    mpu.lastTime = now;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    int16_t rawGZ = (Wire.read() << 8) | Wire.read();

    float gz_dps = ((float)rawGZ - mpu.gyroZ_offset) / 131.0f; // Scale factor for 250dps
    if (abs(gz_dps) > 0.5f) { // Noise deadband filter
        mpu.yaw += gz_dps * dt;
    }
}

void readIRSensors() {
    // Basic moving average noise reduction
    int samples = 4;
    long sumL = 0, sumF = 0, sumR = 0;

    for (int i = 0; i < samples; i++) {
        sumL += analogRead(PIN_IR_LEFT);
        sumF += analogRead(PIN_IR_FRONT);
        sumR += analogRead(PIN_IR_RIGHT);
    }

    ir.rawL = sumL / samples;
    ir.rawF = sumF / samples;
    ir.rawR = sumR / samples;

    ir.wallL = (ir.rawL > WALL_THRESHOLD_L);
    ir.wallF = (ir.rawF > WALL_THRESHOLD_F);
    ir.wallR = (ir.rawR > WALL_THRESHOLD_R);
}

// ============================================================================
// 6. CLOSED-LOOP MOTION CONTROLLER
// ============================================================================

void resetOdometry() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
    mpu.yaw = 0.0f;
    mpu.lastTime = micros();
}

void driveDistance(float distanceMM, int maxSpeed) {
    resetOdometry();
    long targetTicks = distanceMM / MM_PER_TICK;
    
    float lastError = 0;
    float errorSum = 0;

    while (true) {
        updateYaw();
        readIRSensors();

        noInterrupts();
        long currentTicks = (leftTicks + rightTicks) / 2;
        interrupts();

        long remainingTicks = targetTicks - currentTicks;
        if (remainingTicks <= 0) break;

        // --- Trapezoidal / Deceleration Profile ---
        int currentSpeed = maxSpeed;
        if (remainingTicks < 300) { // Deceleration zone
            currentSpeed = map(remainingTicks, 0, 300, 60, maxSpeed);
        }

        // --- Angular Correction (Wall Following vs Gyro) ---
        float steeringCorrection = 0.0f;

        if (ir.wallL && ir.wallR) {
            // Dual Wall PID Alignment
            float wallError = (ir.rawL - ir.rawR) - TARGET_CENTER_DIFF;
            steeringCorrection = wallError * 0.08f; 
        } else {
            // Gyro Heading Lock
            float headingError = 0.0f - mpu.yaw;
            steeringCorrection = headingError * pidDrive.Kp;
        }

        setMotors(currentSpeed - steeringCorrection, currentSpeed + steeringCorrection);
        delay(5);
    }

    stopMotors();
}

void turnAngle(float targetAngleDeg) {
    resetOdometry();
    float lastError = 0;
    
    while (true) {
        updateYaw();
        float error = targetAngleDeg - mpu.yaw;

        if (abs(error) < 1.0f) break; // Precision tolerance trigger

        float pTerm = error * pidTurn.Kp;
        float dTerm = (error - lastError) * pidTurn.Kd;
        lastError = error;

        int turnSpeed = constrain((int)(pTerm + dTerm), -180, 180);
        
        // Anti-stall lower bound PWM
        if (turnSpeed > 0 && turnSpeed < 50) turnSpeed = 50;
        if (turnSpeed < 0 && turnSpeed > -50) turnSpeed = -50;

        setMotors(-turnSpeed, turnSpeed);
        delay(5);
    }

    stopMotors();
}

// Dynamic Helpers
void turnLeft90()  { turnAngle(-90.0f); }
void turnRight90() { turnAngle(90.0f); }
void turn180()     { turnAngle(180.0f); }

// ============================================================================
// 7. TELEMETRY & DEBUG DASHBOARD
// ============================================================================

void renderDashboard(const char* actionState) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.printf("ACT: %s\n", actionState);
    display.printf("Yaw: %.1f deg\n", mpu.yaw);
    display.printf("IR  : L:%d F:%d R:%d\n", ir.rawL, ir.rawF, ir.rawR);
    display.printf("Walls: [%c] [%c] [%c]\n", 
                    ir.wallL ? 'L' : ' ', 
                    ir.wallF ? 'F' : ' ', 
                    ir.wallR ? 'R' : ' ');
    display.display();

    // Serial Plotter Compatibility Format
    Serial.printf(">Yaw:%.2f,IR_Left:%d,IR_Front:%d,IR_Right:%d\n", 
                  mpu.yaw, ir.rawL, ir.rawF, ir.rawR);
}

// ============================================================================
// 8. INITIALIZATION & MAIN SEQUENCE
// ============================================================================

void setup() {
    Serial.begin(115200);
    initHardware();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println("Calibrating IMU...");
    display.println("DO NOT MOVE MOUSE");
    display.display();

    initMPU6050();
    
    tone(PIN_BUZZER, 2000, 100);
    
    // Wait for Button Press to Begin Execution
    while (digitalRead(PIN_BUTTON) == HIGH) {
        updateYaw();
        readIRSensors();
        renderDashboard("STANDBY");
        delay(50);
    }

    tone(PIN_BUZZER, 3000, 200);
    delay(500);
}

void loop() {
    // Executing standard Micromouse Phase 2 motion sequence demo:
    
    // 1. Traverse 1 Cell (180mm) Forward with Active Wall PID
    renderDashboard("1 CELL FWD");
    driveDistance(CELL_DISTANCE_MM, 160);
    delay(300);

    // 2. Perform 90-degree Precision Pivot Right
    renderDashboard("TURN RIGHT 90");
    turnRight90();
    delay(300);

    // 3. Traverse 1 Cell Forward
    renderDashboard("1 CELL FWD");
    driveDistance(CELL_DISTANCE_MM, 160);
    delay(300);

    // 4. Perform 180-degree U-Turn
    renderDashboard("TURN 180");
    turn180();
    delay(300);

    // Sequence execution completed - Enter telemetry loop
    while (true) {
        updateYaw();
        readIRSensors();
        renderDashboard("FINISHED");
        delay(100);
    }
}