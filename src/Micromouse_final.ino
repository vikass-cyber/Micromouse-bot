#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// 1. PIN DEFINITIONS & HARDWARE CONSTANTS
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

// Analog IR Sensors & Diagnostics
#define PIN_IR_LEFT      36
#define PIN_IR_FRONT     39
#define PIN_IR_RIGHT     35
#define PIN_BATTERY_ADC  34 // Voltage divider pin (10k / 3.3k)

// User Interface & Peripherals
#define PIN_BUTTON_NEXT  0
#define PIN_BUTTON_SELECT 4
#define PIN_BUZZER       18
#define MPU6050_ADDR     0x68

// OLED Display Configuration
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Preferences nvram;

// Kinematic Constants
#define WHEEL_DIAMETER_MM 32.0f
#define TICKS_PER_REV     360.0f
#define MM_PER_TICK       ((3.14159265f * WHEEL_DIAMETER_MM) / TICKS_PER_REV)
#define CELL_DISTANCE_MM  180.0f

// Sensor Thresholds
const int WALL_THRESHOLD_L = 800;
const int WALL_THRESHOLD_F = 1200;
const int WALL_THRESHOLD_R = 800;

// Battery Safety Constants
const float BATTERY_ADC_FACTOR = 3.3f / 4095.0f * ((10.0f + 3.3f) / 3.3f);
const float MIN_BATTERY_VOLTAGE = 6.8f; // 2S LiPo low voltage limit

// ============================================================================
// 2. DATA STRUCTURES & GLOBAL STATES
// ============================================================================

struct Configuration {
    float kp_drive = 2.5f;
    float kd_drive = 1.2f;
    float kp_turn  = 2.8f;
    float kd_turn  = 1.5f;
    int search_speed = 150;
    int fast_speed   = 230;
    float accel_rate = 5.0f; 
} config;

enum SystemState { MENU_MAIN, STATE_SEARCH_RUN, STATE_FAST_RUN, STATE_CALIBRATION, STATE_DIAGNOSTICS, STATE_ERROR };
SystemState currentState = MENU_MAIN;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

struct IRSensors {
    int rawL, rawF, rawR;
    bool wallL, wallF, wallR;
} ir;

struct MPUData {
    float gyroZ_offset = 0.0f;
    float gz = 0.0f;
    float yaw = 0.0f;
    unsigned long lastTime = 0;
} mpu;

// ============================================================================
// 3. MAZE SOLVER & ALGORITHMIC CLASS (PHASE 3 INTEGRATION)
// ============================================================================

#define MAZE_SIZE 16
#define MAX_QUEUE_SIZE (MAZE_SIZE * MAZE_SIZE)

#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08
#define VISITED 0x10

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum MotionCmd { CMD_FORWARD_1, CMD_TURN_LEFT, CMD_TURN_RIGHT, CMD_TURN_180, CMD_STOP };

struct Point {
    uint8_t x, y;
};

class MazeSolver {
public:
    MazeSolver();
    void init();
    void setWall(uint8_t x, uint8_t y, Direction dir);
    bool hasWall(uint8_t x, uint8_t y, Direction dir);
    void setVisited(uint8_t x, uint8_t y);
    bool isVisited(uint8_t x, uint8_t y);
    void updateFloodFill(uint8_t targetX1, uint8_t targetY1, uint8_t targetX2 = 255, uint8_t targetY2 = 255);
    uint8_t getDistance(uint8_t x, uint8_t y);
    Direction getBestNeighborDir(uint8_t x, uint8_t y, Direction currentHeading);
    MotionCmd getNextMotionCommand(uint8_t x, uint8_t y, Direction currentHeading);

private:
    uint8_t cells[MAZE_SIZE][MAZE_SIZE];
    uint8_t dist[MAZE_SIZE][MAZE_SIZE];
    Point queue[MAX_QUEUE_SIZE];
    int qHead, qTail;
    
    void pushQueue(uint8_t x, uint8_t y);
    Point popQueue();
    bool isQueueEmpty();
    void clearQueue();
};

MazeSolver::MazeSolver() { init(); }

void MazeSolver::init() {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            cells[x][y] = 0;
            dist[x][y] = 255;
            if (y == MAZE_SIZE - 1) cells[x][y] |= WALL_N;
            if (x == MAZE_SIZE - 1) cells[x][y] |= WALL_E;
            if (y == 0)             cells[x][y] |= WALL_S;
            if (x == 0)             cells[x][y] |= WALL_W;
        }
    }
}

void MazeSolver::setWall(uint8_t x, uint8_t y, Direction dir) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return;
    switch (dir) {
        case NORTH:
            cells[x][y] |= WALL_N;
            if (y < MAZE_SIZE - 1) cells[x][y + 1] |= WALL_S;
            break;
        case EAST:
            cells[x][y] |= WALL_E;
            if (x < MAZE_SIZE - 1) cells[x + 1][y] |= WALL_W;
            break;
        case SOUTH:
            cells[x][y] |= WALL_S;
            if (y > 0) cells[x][y - 1] |= WALL_N;
            break;
        case WEST:
            cells[x][y] |= WALL_W;
            if (x > 0) cells[x - 1][y] |= WALL_E;
            break;
    }
}

bool MazeSolver::hasWall(uint8_t x, uint8_t y, Direction dir) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return true;
    switch (dir) {
        case NORTH: return (cells[x][y] & WALL_N) != 0;
        case EAST:  return (cells[x][y] & WALL_E) != 0;
        case SOUTH: return (cells[x][y] & WALL_S) != 0;
        case WEST:  return (cells[x][y] & WALL_W) != 0;
    }
    return true;
}

void MazeSolver::setVisited(uint8_t x, uint8_t y) {
    if (x < MAZE_SIZE && y < MAZE_SIZE) cells[x][y] |= VISITED;
}

bool MazeSolver::isVisited(uint8_t x, uint8_t y) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return false;
    return (cells[x][y] & VISITED) != 0;
}

uint8_t MazeSolver::getDistance(uint8_t x, uint8_t y) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE) return 255;
    return dist[x][y];
}

void MazeSolver::pushQueue(uint8_t x, uint8_t y) {
    if (qTail < MAX_QUEUE_SIZE) queue[qTail++] = {x, y};
}

Point MazeSolver::popQueue() { return queue[qHead++]; }
bool MazeSolver::isQueueEmpty() { return qHead >= qTail; }
void MazeSolver::clearQueue() { qHead = 0; qTail = 0; }

void MazeSolver::updateFloodFill(uint8_t targetX1, uint8_t targetY1, uint8_t targetX2, uint8_t targetY2) {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            dist[x][y] = 255;
        }
    }
    clearQueue();

    dist[targetX1][targetY1] = 0;
    pushQueue(targetX1, targetY1);
    
    if (targetX2 < MAZE_SIZE && targetY2 < MAZE_SIZE) {
        dist[targetX2][targetY1] = 0; pushQueue(targetX2, targetY1);
        dist[targetX1][targetY2] = 0; pushQueue(targetX1, targetY2);
        dist[targetX2][targetY2] = 0; pushQueue(targetX2, targetY2);
    }

    while (!isQueueEmpty()) {
        Point curr = popQueue();
        uint8_t currentDist = dist[curr.x][curr.y];

        if (!hasWall(curr.x, curr.y, NORTH) && curr.y < MAZE_SIZE - 1) {
            if (dist[curr.x][curr.y + 1] == 255) {
                dist[curr.x][curr.y + 1] = currentDist + 1;
                pushQueue(curr.x, curr.y + 1);
            }
        }
        if (!hasWall(curr.x, curr.y, EAST) && curr.x < MAZE_SIZE - 1) {
            if (dist[curr.x + 1][curr.y] == 255) {
                dist[curr.x + 1][curr.y] = currentDist + 1;
                pushQueue(curr.x + 1, curr.y);
            }
        }
        if (!hasWall(curr.x, curr.y, SOUTH) && curr.y > 0) {
            if (dist[curr.x][curr.y - 1] == 255) {
                dist[curr.x][curr.y - 1] = currentDist + 1;
                pushQueue(curr.x, curr.y - 1);
            }
        }
        if (!hasWall(curr.x, curr.y, WEST) && curr.x > 0) {
            if (dist[curr.x - 1][curr.y] == 255) {
                dist[curr.x - 1][curr.y] = currentDist + 1;
                pushQueue(curr.x - 1, curr.y);
            }
        }
    }
}

Direction MazeSolver::getBestNeighborDir(uint8_t x, uint8_t y, Direction currentHeading) {
    uint8_t minDistance = 255;
    Direction bestDir = currentHeading;

    Direction dirs[4] = { currentHeading, 
                          (Direction)((currentHeading + 1) % 4), 
                          (Direction)((currentHeading + 3) % 4), 
                          (Direction)((currentHeading + 2) % 4) };

    for (int i = 0; i < 4; i++) {
        Direction d = dirs[i];
        if (!hasWall(x, y, d)) {
            uint8_t nx = x, ny = y;
            if (d == NORTH) ny++;
            else if (d == EAST) nx++;
            else if (d == SOUTH) ny--;
            else if (d == WEST) nx--;

            if (dist[nx][ny] < minDistance) {
                minDistance = dist[nx][ny];
                bestDir = d;
            }
        }
    }
    return bestDir;
}

MotionCmd MazeSolver::getNextMotionCommand(uint8_t x, uint8_t y, Direction currentHeading) {
    Direction targetDir = getBestNeighborDir(x, y, currentHeading);
    if (targetDir == currentHeading) {
        return CMD_FORWARD_1;
    } else if (targetDir == (currentHeading + 1) % 4) {
        return CMD_TURN_RIGHT;
    } else if (targetDir == (currentHeading + 3) % 4) {
        return CMD_TURN_LEFT;
    } else {
        return CMD_TURN_180;
    }
}

MazeSolver solver;
uint8_t posX = 0, posY = 0;
Direction heading = NORTH;

// ============================================================================
// 4. LOW-LEVEL DRIVERS & HARDWARE INTERRUPTS
// ============================================================================

void IRAM_ATTR isrLeftEncoder() {
    if (digitalRead(PIN_ENC_L_B) == HIGH) leftTicks++;
    else leftTicks--;
}

void IRAM_ATTR isrRightEncoder() {
    if (digitalRead(PIN_ENC_R_B) == HIGH) rightTicks++;
    else rightTicks--;
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

float readBatteryVoltage() {
    int raw = analogRead(PIN_BATTERY_ADC);
    return raw * BATTERY_ADC_FACTOR;
}

void checkBatteryHealth() {
    float voltage = readBatteryVoltage();
    if (voltage < MIN_BATTERY_VOLTAGE && voltage > 2.0f) {
        stopMotors();
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.println("LOW BATT!");
        display.setTextSize(1);
        display.printf("Voltage: %.2fV\n", voltage);
        display.display();
        
        while (true) {
            tone(PIN_BUZZER, 1000, 100);
            delay(200);
            tone(PIN_BUZZER, 500, 100);
            delay(800);
        }
    }
}

void readIRSensors() {
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

void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    long gyroSum = 0;
    const int samples = 500;
    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47);
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
    float gz_dps = ((float)rawGZ - mpu.gyroZ_offset) / 131.0f;
    if (abs(gz_dps) > 0.5f) {
        mpu.yaw += gz_dps * dt;
    }
}

// ============================================================================
// 5. CLOSED-LOOP NAVIGATION & PID CONTROL
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

    while (true) {
        updateYaw();
        readIRSensors();
        
        noInterrupts();
        long currentTicks = (leftTicks + rightTicks) / 2;
        interrupts();

        long remainingTicks = targetTicks - currentTicks;
        if (remainingTicks <= 0) break;

        int currentSpeed = maxSpeed;
        if (remainingTicks < 300) {
            currentSpeed = map(remainingTicks, 0, 300, 60, maxSpeed);
        }

        float steeringCorrection = 0.0f;
        if (ir.wallL && ir.wallR) {
            float wallError = (ir.rawL - ir.rawR);
            steeringCorrection = wallError * 0.05f;
        } else {
            steeringCorrection = mpu.yaw * config.kp_drive;
        }

        int leftPwm  = constrain(currentSpeed - steeringCorrection, 0, 255);
        int rightPwm = constrain(currentSpeed + steeringCorrection, 0, 255);
        setMotors(leftPwm, rightPwm);
        delay(5);
    }
    stopMotors();
}

void turnAngle(float targetAngle) {
    resetOdometry();
    while (true) {
        updateYaw();
        float error = targetAngle - mpu.yaw;
        if (abs(error) < 1.0f) break;

        float turnSpeed = constrain(error * config.kp_turn, -180.0f, 180.0f);
        if (turnSpeed > 0 && turnSpeed < 60) turnSpeed = 60;
        if (turnSpeed < 0 && turnSpeed > -60) turnSpeed = -60;

        setMotors(-turnSpeed, turnSpeed);
        delay(5);
    }
    stopMotors();
}

void turnRight90() { turnAngle(-90.0f); }
void turnLeft90()  { turnAngle(90.0f); }
void turn180()     { turnAngle(180.0f); }

// ============================================================================
// 6. NVRAM PREFERENCES & MENU SYSTEM (PHASE 4)
// ============================================================================

void loadConfiguration() {
    nvram.begin("micromouse", true);
    config.kp_drive    = nvram.getFloat("kp_d", 2.5f);
    config.kd_drive    = nvram.getFloat("kd_d", 1.2f);
    config.kp_turn     = nvram.getFloat("kp_t", 2.8f);
    config.kd_turn     = nvram.getFloat("kd_t", 1.5f);
    config.fast_speed  = nvram.getInt("f_spd", 230);
    nvram.end();
}

void saveConfiguration() {
    nvram.begin("micromouse", false);
    nvram.putFloat("kp_d", config.kp_drive);
    nvram.putFloat("kd_d", config.kd_drive);
    nvram.putFloat("kp_t", config.kp_turn);
    nvram.putFloat("kd_t", config.kd_turn);
    nvram.putInt("f_spd", config.fast_speed);
    nvram.end();
}

const char* menuItems[] = {
    "1. Search Run",
    "2. Fast Run",
    "3. Diagnostics",
    "4. Save Config",
    "5. Calibrate Sensors"
};
const int TOTAL_MENU_ITEMS = 5;
int currentMenuIdx = 0;

void drawMenu() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.printf("MICROMOUSE OS v4.0\nBATT: %.2fV\n---\n", readBatteryVoltage());

    for (int i = 0; i < TOTAL_MENU_ITEMS; i++) {
        if (i == currentMenuIdx) {
            display.printf("> %s\n", menuItems[i]);
        } else {
            display.printf("  %s\n", menuItems[i]);
        }
    }
    display.display();
}

void scanWallsAndUpdateMap() {
    readIRSensors();
    if (ir.wallF) solver.setWall(posX, posY, heading);
    if (ir.wallR) solver.setWall(posX, posY, (Direction)((heading + 1) % 4));
    if (ir.wallL) solver.setWall(posX, posY, (Direction)((heading + 3) % 4));
    solver.setVisited(posX, posY);
}

void updateCoordinates() {
    switch (heading) {
        case NORTH: posY++; break;
        case EAST:  posX++; break;
        case SOUTH: posY--; break;
        case WEST:  posX--; break;
    }
}

void executeSearchRun() {
    posX = 0; posY = 0; heading = NORTH;
    solver.init();

    while (true) {
        checkBatteryHealth();

        if (solver.getDistance(posX, posY) == 0) {
            tone(PIN_BUZZER, 3000, 1000);
            stopMotors();
            display.clearDisplay();
            display.setCursor(10, 25);
            display.println("GOAL REACHED!");
            display.display();
            delay(3000);
            break;
        }

        scanWallsAndUpdateMap();
        solver.updateFloodFill(7, 7, 8, 8);
        MotionCmd cmd = solver.getNextMotionCommand(posX, posY, heading);

        switch (cmd) {
            case CMD_FORWARD_1:
                driveDistance(CELL_DISTANCE_MM, config.search_speed);
                updateCoordinates();
                break;
            case CMD_TURN_RIGHT:
                turnRight90();
                heading = (Direction)((heading + 1) % 4);
                driveDistance(CELL_DISTANCE_MM, config.search_speed);
                updateCoordinates();
                break;
            case CMD_TURN_LEFT:
                turnLeft90();
                heading = (Direction)((heading + 3) % 4);
                driveDistance(CELL_DISTANCE_MM, config.search_speed);
                updateCoordinates();
                break;
            case CMD_TURN_180:
                turn180();
                heading = (Direction)((heading + 2) % 4);
                driveDistance(CELL_DISTANCE_MM, config.search_speed);
                updateCoordinates();
                break;
            case CMD_STOP:
                stopMotors();
                return;
        }
    }
}

void runDiagnostics() {
    display.clearDisplay();
    while (digitalRead(PIN_BUTTON_NEXT) == HIGH) {
        readIRSensors();
        updateYaw();

        display.clearDisplay();
        display.setCursor(0, 0);
        display.printf("L:%d F:%d R:%d\n", ir.rawL, ir.rawF, ir.rawR);
        display.printf("Yaw: %.1f deg\n", mpu.yaw);
        display.printf("EncL: %ld R: %ld\n", leftTicks, rightTicks);
        display.printf("Batt: %.2f V\n", readBatteryVoltage());
        display.display();
        delay(100);
    }
}

void handleMenuNavigation() {
    drawMenu();
    while (currentState == MENU_MAIN) {
        checkBatteryHealth();
        if (digitalRead(PIN_BUTTON_NEXT) == LOW) {
            tone(PIN_BUZZER, 2500, 50);
            currentMenuIdx = (currentMenuIdx + 1) % TOTAL_MENU_ITEMS;
            drawMenu();
            delay(200);
        }
        if (digitalRead(PIN_BUTTON_SELECT) == LOW) {
            tone(PIN_BUZZER, 3500, 100);
            delay(200);
            switch (currentMenuIdx) {
                case 0: currentState = STATE_SEARCH_RUN; break;
                case 1: currentState = STATE_FAST_RUN; break;
                case 2: currentState = STATE_DIAGNOSTICS; break;
                case 3: 
                    saveConfiguration();
                    display.clearDisplay();
                    display.setCursor(10, 25);
                    display.println("CONFIG SAVED!");
                    display.display();
                    delay(1000);
                    drawMenu();
                    break;
                case 4: currentState = STATE_CALIBRATION; break;
            }
        }
        delay(20);
    }
}

// ============================================================================
// 7. SETUP & MAIN EXECUTION LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);

    // Pin Modes Initialization
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    pinMode(PIN_MOTOR_L_PWM, OUTPUT);
    pinMode(PIN_MOTOR_R_PWM, OUTPUT);
    stopMotors();

    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightEncoder, RISING);

    pinMode(PIN_BUTTON_NEXT, INPUT_PULLUP);
    pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);

    Wire.begin(21, 22);
    Wire.setClock(400000);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println("Calibrating Gyro...");
    display.display();

    initMPU6050();
    loadConfiguration();
    checkBatteryHealth();

    tone(PIN_BUZZER, 2000, 100);
}

void loop() {
    switch (currentState) {
        case MENU_MAIN:
            handleMenuNavigation();
            break;

        case STATE_SEARCH_RUN:
            executeSearchRun();
            currentState = MENU_MAIN;
            break;

        case STATE_FAST_RUN:
            display.clearDisplay();
            display.setCursor(0, 20);
            display.println("FAST RUN MODE");
            display.display();
            delay(2000);
            currentState = MENU_MAIN;
            break;

        case STATE_DIAGNOSTICS:
            runDiagnostics();
            currentState = MENU_MAIN;
            break;

        case STATE_CALIBRATION:
            display.clearDisplay();
            display.setCursor(0, 20);
            display.println("CALIBRATING...");
            display.display();
            delay(1500);
            currentState = MENU_MAIN;
            break;

        default:
            currentState = MENU_MAIN;
            break;
    }
}