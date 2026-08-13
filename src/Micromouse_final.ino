#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "pid.h"

// ============================================================================
// 1. GLOBAL VARIABLES & OBJECT INSTANTIATIONS
// ============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Preferences nvram;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

Configuration config;
IRSensors ir;
MPUData mpu;

SystemState currentState = MENU_MAIN;

uint8_t posX = 0;
uint8_t posY = 0;

Direction heading = NORTH;

bool mazeSearchCompleted = false;


// ============================================================================
// 2. MAZE SOLVER
// ============================================================================

class MazeSolver {
public:

    MazeSolver();
    void init();

    void setWall(uint8_t x, uint8_t y, Direction dir);
    bool hasWall(uint8_t x, uint8_t y, Direction dir);

    void setVisited(uint8_t x, uint8_t y);
    bool isVisited(uint8_t x, uint8_t y);

    void updateFloodFill(
        uint8_t targetX1,
        uint8_t targetY1,
        uint8_t targetX2 = 255,
        uint8_t targetY2 = 255
    );

    uint8_t getDistance(uint8_t x, uint8_t y);

    Direction getBestNeighborDir(
        uint8_t currentX,
        uint8_t currentY,
        Direction currentHeading
    );

    MotionCmd getNextMotionCommand(
        uint8_t currentX,
        uint8_t currentY,
        Direction currentHeading
    );

private:

    uint8_t cells[MAZE_SIZE][MAZE_SIZE];
    uint8_t dist[MAZE_SIZE][MAZE_SIZE];

    Point queue[MAX_QUEUE_SIZE];

    int qHead;
    int qTail;

    void pushQueue(uint8_t x, uint8_t y);
    Point popQueue();

    bool isQueueEmpty();
    void clearQueue();
};


// ============================================================================
// MAZE SOLVER IMPLEMENTATION
// ============================================================================

MazeSolver::MazeSolver()
{
    init();
}


void MazeSolver::init()
{
    qHead = 0;
    qTail = 0;

    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            cells[x][y] = 0;
            dist[x][y] = 255;

            if (y == MAZE_SIZE - 1)
                cells[x][y] |= WALL_N;

            if (x == MAZE_SIZE - 1)
                cells[x][y] |= WALL_E;

            if (y == 0)
                cells[x][y] |= WALL_S;

            if (x == 0)
                cells[x][y] |= WALL_W;
        }
    }
}


void MazeSolver::setWall(
    uint8_t x,
    uint8_t y,
    Direction dir
)
{
    if (x >= MAZE_SIZE || y >= MAZE_SIZE)
        return;

    switch (dir)
    {
        case NORTH:

            cells[x][y] |= WALL_N;

            if (y < MAZE_SIZE - 1)
                cells[x][y + 1] |= WALL_S;

            break;


        case EAST:

            cells[x][y] |= WALL_E;

            if (x < MAZE_SIZE - 1)
                cells[x + 1][y] |= WALL_W;

            break;


        case SOUTH:

            cells[x][y] |= WALL_S;

            if (y > 0)
                cells[x][y - 1] |= WALL_N;

            break;


        case WEST:

            cells[x][y] |= WALL_W;

            if (x > 0)
                cells[x - 1][y] |= WALL_E;

            break;
    }
}


bool MazeSolver::hasWall(
    uint8_t x,
    uint8_t y,
    Direction dir
)
{
    if (x >= MAZE_SIZE || y >= MAZE_SIZE)
        return true;

    switch (dir)
    {
        case NORTH:
            return (cells[x][y] & WALL_N) != 0;

        case EAST:
            return (cells[x][y] & WALL_E) != 0;

        case SOUTH:
            return (cells[x][y] & WALL_S) != 0;

        case WEST:
            return (cells[x][y] & WALL_W) != 0;
    }

    return true;
}


void MazeSolver::setVisited(
    uint8_t x,
    uint8_t y
)
{
    if (x < MAZE_SIZE && y < MAZE_SIZE)
        cells[x][y] |= VISITED;
}


bool MazeSolver::isVisited(
    uint8_t x,
    uint8_t y
)
{
    if (x >= MAZE_SIZE || y >= MAZE_SIZE)
        return false;

    return (cells[x][y] & VISITED) != 0;
}


uint8_t MazeSolver::getDistance(
    uint8_t x,
    uint8_t y
)
{
    if (x >= MAZE_SIZE || y >= MAZE_SIZE)
        return 255;

    return dist[x][y];
}


void MazeSolver::pushQueue(
    uint8_t x,
    uint8_t y
)
{
    if (qTail < MAX_QUEUE_SIZE)
    {
        queue[qTail].x = x;
        queue[qTail].y = y;

        qTail++;
    }
}


Point MazeSolver::popQueue()
{
    Point p = queue[qHead];

    qHead++;

    return p;
}


bool MazeSolver::isQueueEmpty()
{
    return qHead >= qTail;
}


void MazeSolver::clearQueue()
{
    qHead = 0;
    qTail = 0;
}


void MazeSolver::updateFloodFill(
    uint8_t targetX1,
    uint8_t targetY1,
    uint8_t targetX2,
    uint8_t targetY2
)
{
    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            dist[x][y] = 255;
        }
    }

    clearQueue();


    // First goal cell
    if (targetX1 < MAZE_SIZE &&
        targetY1 < MAZE_SIZE)
    {
        dist[targetX1][targetY1] = 0;

        pushQueue(
            targetX1,
            targetY1
        );
    }


    // Remaining three center cells
    if (targetX2 < MAZE_SIZE &&
        targetY2 < MAZE_SIZE)
    {
        dist[targetX2][targetY1] = 0;

        pushQueue(
            targetX2,
            targetY1
        );


        dist[targetX1][targetY2] = 0;

        pushQueue(
            targetX1,
            targetY2
        );


        dist[targetX2][targetY2] = 0;

        pushQueue(
            targetX2,
            targetY2
        );
    }


    // BFS FLOOD FILL
    while (!isQueueEmpty())
    {
        Point curr = popQueue();

        uint8_t currentDist =
            dist[curr.x][curr.y];


        // NORTH
        if (!hasWall(
                curr.x,
                curr.y,
                NORTH
            ) &&
            curr.y < MAZE_SIZE - 1)
        {
            if (dist[curr.x][curr.y + 1] == 255)
            {
                dist[curr.x][curr.y + 1] =
                    currentDist + 1;

                pushQueue(
                    curr.x,
                    curr.y + 1
                );
            }
        }


        // EAST
        if (!hasWall(
                curr.x,
                curr.y,
                EAST
            ) &&
            curr.x < MAZE_SIZE - 1)
        {
            if (dist[curr.x + 1][curr.y] == 255)
            {
                dist[curr.x + 1][curr.y] =
                    currentDist + 1;

                pushQueue(
                    curr.x + 1,
                    curr.y
                );
            }
        }


        // SOUTH
        if (!hasWall(
                curr.x,
                curr.y,
                SOUTH
            ) &&
            curr.y > 0)
        {
            if (dist[curr.x][curr.y - 1] == 255)
            {
                dist[curr.x][curr.y - 1] =
                    currentDist + 1;

                pushQueue(
                    curr.x,
                    curr.y - 1
                );
            }
        }


        // WEST
        if (!hasWall(
                curr.x,
                curr.y,
                WEST
            ) &&
            curr.x > 0)
        {
            if (dist[curr.x - 1][curr.y] == 255)
            {
                dist[curr.x - 1][curr.y] =
                    currentDist + 1;

                pushQueue(
                    curr.x - 1,
                    curr.y
                );
            }
        }
    }
}


Direction MazeSolver::getBestNeighborDir(
    uint8_t x,
    uint8_t y,
    Direction currentHeading
)
{
    uint8_t minDistance = 255;

    Direction bestDir = currentHeading;


    Direction dirs[4] =
    {
        currentHeading,

        (Direction)(
            (currentHeading + 1) % 4
        ),

        (Direction)(
            (currentHeading + 3) % 4
        ),

        (Direction)(
            (currentHeading + 2) % 4
        )
    };


    for (int i = 0; i < 4; i++)
    {
        Direction d = dirs[i];

        if (!hasWall(x, y, d))
        {
            uint8_t nx = x;
            uint8_t ny = y;


            if (d == NORTH)
                ny++;

            else if (d == EAST)
                nx++;

            else if (d == SOUTH)
                ny--;

            else if (d == WEST)
                nx--;


            if (nx < MAZE_SIZE &&
                ny < MAZE_SIZE)
            {
                if (dist[nx][ny] < minDistance)
                {
                    minDistance =
                        dist[nx][ny];

                    bestDir = d;
                }
            }
        }
    }

    return bestDir;
}


MotionCmd MazeSolver::getNextMotionCommand(
    uint8_t x,
    uint8_t y,
    Direction currentHeading
)
{
    Direction targetDir =
        getBestNeighborDir(
            x,
            y,
            currentHeading
        );


    if (targetDir == currentHeading)
    {
        return CMD_FORWARD_1;
    }


    if (targetDir ==
        (currentHeading + 1) % 4)
    {
        return CMD_TURN_RIGHT;
    }


    if (targetDir ==
        (currentHeading + 3) % 4)
    {
        return CMD_TURN_LEFT;
    }


    return CMD_TURN_180;
}


MazeSolver solver;


// ============================================================================
// 3. ENCODER INTERRUPTS
// ============================================================================

void IRAM_ATTR isrLeftEncoder()
{
    if (digitalRead(PIN_ENC_L_B) == HIGH)
        leftTicks++;
    else
        leftTicks--;
}


void IRAM_ATTR isrRightEncoder()
{
    if (digitalRead(PIN_ENC_R_B) == HIGH)
        rightTicks++;
    else
        rightTicks--;
}


// ============================================================================
// 4. HARDWARE INITIALIZATION
// ============================================================================

void initHardware()
{
    // Motor driver
    initMotors();


    // Encoder pins
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);

    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);


    attachInterrupt(
        digitalPinToInterrupt(PIN_ENC_L_A),
        isrLeftEncoder,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(PIN_ENC_R_A),
        isrRightEncoder,
        RISING
    );


    // IR sensors are initialized by sensors.cpp
    initIRSensors();


    // User controls
    pinMode(
        PIN_BUTTON_NEXT,
        INPUT_PULLUP
    );

    pinMode(
        PIN_BUTTON_SELECT,
        INPUT_PULLUP
    );


    // Buzzer
    pinMode(
        PIN_BUZZER,
        OUTPUT
    );


    // I2C
    Wire.begin(21, 22);

    Wire.setClock(400000);


    // OLED
    if (!display.begin(
            SSD1306_SWITCHCAPVCC,
            0x3C
        ))
    {
        currentState = STATE_ERROR;

        return;
    }


    display.clearDisplay();

    display.display();
}


// ============================================================================
// 5. ENCODERS & BATTERY
// ============================================================================

void resetEncoders()
{
    noInterrupts();

    leftTicks = 0;
    rightTicks = 0;

    interrupts();
}


float readBatteryVoltage()
{
    int raw =
        analogRead(PIN_BATTERY_ADC);

    return raw * BATTERY_ADC_FACTOR;
}


void checkBatteryHealth()
{
    float voltage =
        readBatteryVoltage();


    if (
        voltage < MIN_BATTERY_VOLTAGE &&
        voltage > 2.0f
    )
    {
        stopMotors();


        display.clearDisplay();

        display.setCursor(
            0,
            16
        );

        display.setTextSize(2);

        display.setTextColor(
            SSD1306_WHITE
        );

        display.println(
            "LOW BATT!"
        );


        display.setTextSize(1);

        display.printf(
            "Voltage: %.2fV\n",
            voltage
        );

        display.display();


        while (true)
        {
            tone(
                PIN_BUZZER,
                1000,
                100
            );

            delay(200);


            tone(
                PIN_BUZZER,
                500,
                100
            );

            delay(800);
        }
    }
}


// ============================================================================
// 6. MPU6050
// ============================================================================

bool initMPU6050()
{
    Wire.beginTransmission(
        MPU6050_ADDR
    );

    Wire.write(0x6B);
    Wire.write(0);


    if (
        Wire.endTransmission(true) != 0
    )
    {
        mpu.initialized = false;

        return false;
    }


    long gyroSum = 0;

    const int samples = 500;


    for (
        int i = 0;
        i < samples;
        i++
    )
    {
        Wire.beginTransmission(
            MPU6050_ADDR
        );

        Wire.write(0x47);


        if (
            Wire.endTransmission(false) != 0
        )
        {
            mpu.initialized = false;

            return false;
        }


        Wire.requestFrom(
            MPU6050_ADDR,
            2,
            true
        );


        if (Wire.available() < 2)
        {
            mpu.initialized = false;

            return false;
        }


        int16_t rawGZ =
            (Wire.read() << 8) |
            Wire.read();


        gyroSum += rawGZ;

        delay(2);
    }


    mpu.gyroZ_offset =
        (float)gyroSum /
        (float)samples;


    mpu.yaw = 0.0f;

    mpu.lastTime = micros();

    mpu.initialized = true;


    return true;
}


void updateYaw()
{
    if (!mpu.initialized)
        return;


    unsigned long now =
        micros();


    float dt =
        (now - mpu.lastTime) /
        1000000.0f;


    if (
        dt <= 0.0f ||
        dt > 0.5f
    )
    {
        mpu.lastTime = now;

        return;
    }


    mpu.lastTime = now;


    Wire.beginTransmission(
        MPU6050_ADDR
    );

    Wire.write(0x47);


    if (
        Wire.endTransmission(false) != 0
    )
        return;


    Wire.requestFrom(
        MPU6050_ADDR,
        2,
        true
    );


    if (Wire.available() < 2)
        return;


    int16_t rawGZ =
        (Wire.read() << 8) |
        Wire.read();


    float gz_dps =
        (
            (float)rawGZ -
            mpu.gyroZ_offset
        ) / 131.0f;


    if (abs(gz_dps) > 0.5f)
    {
        mpu.yaw +=
            gz_dps * dt;
    }
}


// ============================================================================
// 7. WALL MAPPING
// ============================================================================

void scanWallsAndUpdateMap()
{
    readIRSensors();


    if (ir.wallF)
    {
        solver.setWall(
            posX,
            posY,
            heading
        );
    }


    if (ir.wallR)
    {
        solver.setWall(
            posX,
            posY,
            (Direction)(
                (heading + 1) % 4
            )
        );
    }


    if (ir.wallL)
    {
        solver.setWall(
            posX,
            posY,
            (Direction)(
                (heading + 3) % 4
            )
        );
    }


    solver.setVisited(
        posX,
        posY
    );
}


// ============================================================================
// 8. CONFIGURATION STORAGE
// ============================================================================

void loadConfiguration()
{
    nvram.begin(
        "micromouse",
        true
    );


    config.kp_drive =
        nvram.getFloat(
            "kp_d",
            2.5f
        );


    config.kd_drive =
        nvram.getFloat(
            "kd_d",
            1.2f
        );


    config.kp_turn =
        nvram.getFloat(
            "kp_t",
            2.8f
        );


    config.kd_turn =
        nvram.getFloat(
            "kd_t",
            1.5f
        );


    config.search_speed =
        nvram.getInt(
            "s_spd",
            150
        );


    config.fast_speed =
        nvram.getInt(
            "f_spd",
            230
        );


    nvram.end();
}


void saveConfiguration()
{
    nvram.begin(
        "micromouse",
        false
    );


    nvram.putFloat(
        "kp_d",
        config.kp_drive
    );


    nvram.putFloat(
        "kd_d",
        config.kd_drive
    );


    nvram.putFloat(
        "kp_t",
        config.kp_turn
    );


    nvram.putFloat(
        "kd_t",
        config.kd_turn
    );


    nvram.putInt(
        "s_spd",
        config.search_speed
    );


    nvram.putInt(
        "f_spd",
        config.fast_speed
    );


    nvram.end();
}


// ============================================================================
// 9. MOTION CONTROL
// ============================================================================

void resetOdometry()
{
    resetEncoders();

    mpu.yaw = 0.0f;

    mpu.lastTime = micros();
}


bool driveDistance(
    float distanceMM,
    int maxSpeed
)
{
    resetOdometry();


    long targetTicks =
        distanceMM /
        MM_PER_TICK;


    float lastError = 0.0f;


    unsigned long startTime =
        millis();


    while (
        millis() - startTime < 3000
    )
    {
        checkBatteryHealth();

        updateYaw();

        readIRSensors();


        noInterrupts();

        long l = leftTicks;
        long r = rightTicks;

        interrupts();


        long currentTicks =
            (l + r) / 2;


        long remainingTicks =
            targetTicks -
            currentTicks;


        if (remainingTicks <= 0)
        {
            stopMotors();

            return true;
        }


        int currentSpeed =
            maxSpeed;


        if (remainingTicks < 300)
        {
            currentSpeed =
                map(
                    remainingTicks,
                    0,
                    300,
                    60,
                    maxSpeed
                );
        }


        float steeringCorrection =
            0.0f;


        if (
            ir.wallL &&
            ir.wallR
        )
        {
            float error =
                ir.rawL -
                ir.rawR;


            float derivative =
                error -
                lastError;


            steeringCorrection =
                (error * 0.05f) +
                (derivative * 0.01f);


            lastError = error;
        }


        else if (ir.wallL)
        {
            float error =
                ir.rawL -
                WALL_THRESH_LEFT;


            steeringCorrection =
                error * 0.04f;
        }


        else if (ir.wallR)
        {
            float error =
                WALL_THRESH_RIGHT -
                ir.rawR;


            steeringCorrection =
                error * 0.04f;
        }


        else
        {
            steeringCorrection =
                mpu.yaw *
                config.kp_drive;
        }


        int leftPwm =
            constrain(
                currentSpeed -
                steeringCorrection,
                -255,
                255
            );


        int rightPwm =
            constrain(
                currentSpeed +
                steeringCorrection,
                -255,
                255
            );


        setMotors(
            leftPwm,
            rightPwm
        );


        delay(5);
    }


    stopMotors();

    return false;
}


bool turnAngle(
    float targetAngleDeg
)
{
    resetOdometry();


    float lastError = 0.0f;


    unsigned long startTime =
        millis();


    while (
        millis() - startTime < 1500
    )
    {
        checkBatteryHealth();

        updateYaw();


        float currentYaw =
            mpu.yaw;


        float error =
            targetAngleDeg -
            currentYaw;


        if (abs(error) < 1.0f)
        {
            stopMotors();

            return true;
        }


        float derivative =
            error -
            lastError;


        lastError = error;


        float output =
            (
                error *
                config.kp_turn
            )
            +
            (
                derivative *
                config.kd_turn
            );


        int speed =
            constrain(
                (int)output,
                -200,
                200
            );


        if (
            speed > 0 &&
            speed < 60
        )
        {
            speed = 60;
        }


        if (
            speed < 0 &&
            speed > -60
        )
        {
            speed = -60;
        }


        setMotors(
            speed,
            -speed
        );


        delay(5);
    }


    stopMotors();

    return false;
}


bool turnRight90()
{
    return turnAngle(90.0f);
}


bool turnLeft90()
{
    return turnAngle(-90.0f);
}


bool turn180()
{
    return turnAngle(180.0f);
}


// ============================================================================
// 10. OLED DASHBOARD
// ============================================================================

void renderDashboard(
    const char* actionState
)
{
    display.clearDisplay();

    display.setTextSize(1);

    display.setTextColor(
        SSD1306_WHITE
    );

    display.setCursor(
        0,
        0
    );


    display.printf(
        "POS: (%d,%d) H:%d\n",
        posX,
        posY,
        heading
    );


    display.printf(
        "ACT: %s\n",
        actionState
    );


    display.printf(
        "IR L:%d F:%d R:%d\n",
        ir.rawL,
        ir.rawF,
        ir.rawR
    );


    display.printf(
        "YAW: %.1f deg\n",
        mpu.yaw
    );


    display.printf(
        "BAT: %.2fV\n",
        readBatteryVoltage()
    );


    display.display();
}


// ============================================================================
// 11. MENU
// ============================================================================

const char* menuItems[] =
{
    "1. Search Run",
    "2. Fast Run",
    "3. Diagnostics",
    "4. Save Config",
    "5. Calibrate Sensors"
};


const int TOTAL_MENU_ITEMS = 5;

int currentMenuIdx = 0;


void drawMenu()
{
    display.clearDisplay();

    display.setTextSize(1);

    display.setTextColor(
        SSD1306_WHITE
    );

    display.setCursor(
        0,
        0
    );


    display.printf(
        "MICROMOUSE OS v4.0\n"
        "BATT: %.2fV\n"
        "---\n",
        readBatteryVoltage()
    );


    for (
        int i = 0;
        i < TOTAL_MENU_ITEMS;
        i++
    )
    {
        if (i == currentMenuIdx)
        {
            display.printf(
                "> %s\n",
                menuItems[i]
            );
        }
        else
        {
            display.printf(
                "  %s\n",
                menuItems[i]
            );
        }
    }


    display.display();
}


void handleMenuNavigation()
{
    drawMenu();


    while (
        currentState ==
        MENU_MAIN
    )
    {
        checkBatteryHealth();


        if (
            digitalRead(
                PIN_BUTTON_NEXT
            ) == LOW
        )
        {
            tone(
                PIN_BUZZER,
                2500,
                50
            );


            currentMenuIdx =
                (
                    currentMenuIdx + 1
                ) %
                TOTAL_MENU_ITEMS;


            drawMenu();

            delay(200);
        }


        if (
            digitalRead(
                PIN_BUTTON_SELECT
            ) == LOW
        )
        {
            tone(
                PIN_BUZZER,
                3500,
                100
            );


            delay(200);


            switch (currentMenuIdx)
            {
                case 0:

                    currentState =
                        STATE_SEARCH_RUN;

                    break;


                case 1:

                    currentState =
                        STATE_FAST_RUN;

                    break;


                case 2:

                    currentState =
                        STATE_DIAGNOSTICS;

                    break;


                case 3:

                    saveConfiguration();


                    display.clearDisplay();

                    display.setCursor(
                        10,
                        25
                    );

                    display.println(
                        "CONFIG SAVED!"
                    );

                    display.display();

                    delay(1000);

                    drawMenu();

                    break;


                case 4:

                    currentState =
                        STATE_CALIBRATION;

                    break;
            }
        }


        delay(10);
    }
}


// ============================================================================
// 12. DIAGNOSTICS
// ============================================================================

void runDiagnostics()
{
    display.clearDisplay();


    while (
        digitalRead(
            PIN_BUTTON_SELECT
        ) == HIGH
    )
    {
        checkBatteryHealth();

        updateYaw();

        readIRSensors();


        display.clearDisplay();

        display.setCursor(
            0,
            0
        );


        display.println(
            "--- DIAGNOSTICS ---"
        );


        display.printf(
            "IR L: %d W:%d\n",
            ir.rawL,
            ir.wallL
        );


        display.printf(
            "IR F: %d W:%d\n",
            ir.rawF,
            ir.wallF
        );


        display.printf(
            "IR R: %d W:%d\n",
            ir.rawR,
            ir.wallR
        );


        noInterrupts();

        long l = leftTicks;
        long r = rightTicks;

        interrupts();


        display.printf(
            "Enc L: %ld\n",
            l
        );


        display.printf(
            "Enc R: %ld\n",
            r
        );


        display.printf(
            "Yaw: %.2f deg\n",
            mpu.yaw
        );


        display.printf(
            "Press SELECT to exit\n"
        );


        display.display();


        delay(100);
    }


    tone(
        PIN_BUZZER,
        2000,
        100
    );


    delay(300);
}


// ============================================================================
// 13. COORDINATES
// ============================================================================

void updateCoordinates()
{
    switch (heading)
    {
        case NORTH:

            if (
                posY <
                MAZE_SIZE - 1
            )
                posY++;

            break;


        case EAST:

            if (
                posX <
                MAZE_SIZE - 1
            )
                posX++;

            break;


        case SOUTH:

            if (posY > 0)
                posY--;

            break;


        case WEST:

            if (posX > 0)
                posX--;

            break;
    }
}


// ============================================================================
// 14. SEARCH RUN
// ============================================================================

void executeSearchRun()
{
    solver.init();


    posX = 0;
    posY = 0;

    heading = NORTH;


    mazeSearchCompleted = false;


    renderDashboard(
        "SEARCH INIT"
    );


    delay(1000);


    while (true)
    {
        checkBatteryHealth();


        if (
            currentState ==
            STATE_ERROR
        )
        {
            return;
        }


        solver.updateFloodFill(
            7,
            7,
            8,
            8
        );


        if (
            solver.getDistance(
                posX,
                posY
            ) == 0
        )
        {
            tone(
                PIN_BUZZER,
                3000,
                1000
            );


            stopMotors();


            display.clearDisplay();

            display.setCursor(
                10,
                25
            );

            display.println(
                "GOAL REACHED!"
            );

            display.display();


            mazeSearchCompleted =
                true;


            delay(3000);

            break;
        }


        scanWallsAndUpdateMap();


        solver.updateFloodFill(
            7,
            7,
            8,
            8
        );


        MotionCmd cmd =
            solver.getNextMotionCommand(
                posX,
                posY,
                heading
            );


        bool moveSuccess = true;


        switch (cmd)
        {
            case CMD_FORWARD_1:

                renderDashboard(
                    "DRIVE FWD"
                );


                moveSuccess =
                    driveDistance(
                        CELL_DISTANCE_MM,
                        config.search_speed
                    );


                if (moveSuccess)
                    updateCoordinates();


                break;


            case CMD_TURN_RIGHT:

                renderDashboard(
                    "TURN RIGHT"
                );


                moveSuccess =
                    turnRight90();


                if (moveSuccess)
                {
                    heading =
                        (Direction)(
                            (heading + 1) % 4
                        );
                }

                break;


            case CMD_TURN_LEFT:

                renderDashboard(
                    "TURN LEFT"
                );


                moveSuccess =
                    turnLeft90();


                if (moveSuccess)
                {
                    heading =
                        (Direction)(
                            (heading + 3) % 4
                        );
                }

                break;


            case CMD_TURN_180:

                renderDashboard(
                    "TURN 180"
                );


                moveSuccess =
                    turn180();


                if (moveSuccess)
                {
                    heading =
                        (Direction)(
                            (heading + 2) % 4
                        );
                }

                break;


            case CMD_STOP:

                stopMotors();

                return;
        }


        if (!moveSuccess)
        {
            stopMotors();

            currentState =
                STATE_ERROR;

            return;
        }


        delay(50);
    }
}


// ============================================================================
// 15. FAST RUN
// ============================================================================

void executeFastRun()
{
    if (!mazeSearchCompleted)
    {
        display.clearDisplay();

        display.setCursor(
            0,
            20
        );

        display.println(
            "ERROR: NO MAZE DATA!"
        );

        display.println(
            "Run Search First"
        );

        display.display();


        tone(
            PIN_BUZZER,
            1000,
            500
        );


        delay(2000);

        return;
    }


    posX = 0;
    posY = 0;

    heading = NORTH;


    renderDashboard(
        "FAST RUN"
    );


    delay(1000);


    while (true)
    {
        checkBatteryHealth();


        if (
            currentState ==
            STATE_ERROR
        )
        {
            return;
        }


        solver.updateFloodFill(
            7,
            7,
            8,
            8
        );


        if (
            solver.getDistance(
                posX,
                posY
            ) == 0
        )
        {
            tone(
                PIN_BUZZER,
                4000,
                1000
            );


            stopMotors();


            display.clearDisplay();

            display.setCursor(
                10,
                25
            );

            display.println(
                "FAST RUN DONE!"
            );

            display.display();


            delay(3000);

            break;
        }


        MotionCmd cmd =
            solver.getNextMotionCommand(
                posX,
                posY,
                heading
            );


        bool moveSuccess = true;


        switch (cmd)
        {
            case CMD_FORWARD_1:

                moveSuccess =
                    driveDistance(
                        CELL_DISTANCE_MM,
                        config.fast_speed
                    );


                if (moveSuccess)
                    updateCoordinates();

                break;


            case CMD_TURN_RIGHT:

                moveSuccess =
                    turnRight90();


                if (moveSuccess)
                {
                    heading =
                        (Direction)(
                            (heading + 1) % 4
                        );
                }

                break;


            case CMD_TURN_LEFT:

                moveSuccess =
                    turnLeft90();


                if (moveSuccess)
                {
                    heading =
                        (Direction)(
                            (heading + 3) % 4
                        );
                }

                break;


            case CMD_TURN_180:

                moveSuccess =
                    turn180();


                if (moveSuccess)
                {
                    heading =
                        (Direction)(
                            (heading + 2) % 4
                        );
                }

                break;


            case CMD_STOP:

                stopMotors();

                return;
        }


        if (!moveSuccess)
        {
            stopMotors();

            currentState =
                STATE_ERROR;

            return;
        }
    }
}


// ============================================================================
// 16. SENSOR CALIBRATION
// ============================================================================

void executeCalibration()
{
    display.clearDisplay();

    display.setCursor(
        0,
        10
    );


    display.println(
        "CALIBRATING IMU..."
    );


    display.println(
        "KEEP STILL!"
    );


    display.display();


    if (!initMPU6050())
    {
        currentState =
            STATE_ERROR;

        return;
    }


    display.clearDisplay();

    display.setCursor(
        0,
        20
    );


    display.println(
        "IMU CALIBRATED!"
    );


    display.display();


    tone(
        PIN_BUZZER,
        2000,
        200
    );


    delay(1000);
}


// ============================================================================
// 17. SETUP
// ============================================================================

void setup()
{
    Serial.begin(115200);


    initHardware();


    loadConfiguration();


    display.clearDisplay();

    display.setTextSize(1);

    display.setTextColor(
        SSD1306_WHITE
    );


    display.setCursor(
        10,
        20
    );


    display.println(
        "SYSTEM BOOTING..."
    );


    display.display();


    if (!initMPU6050())
    {
        currentState =
            STATE_ERROR;
    }
    else
    {
        tone(
            PIN_BUZZER,
            2000,
            100
        );


        delay(500);


        currentState =
            MENU_MAIN;
    }
}


// ============================================================================
// 18. MAIN LOOP
// ============================================================================

void loop()
{
    checkBatteryHealth();


    switch (currentState)
    {
        case MENU_MAIN:

            handleMenuNavigation();

            break;


        case STATE_SEARCH_RUN:

            executeSearchRun();


            if (
                currentState !=
                STATE_ERROR
            )
            {
                currentState =
                    MENU_MAIN;
            }

            break;


        case STATE_FAST_RUN:

            executeFastRun();


            if (
                currentState !=
                STATE_ERROR
            )
            {
                currentState =
                    MENU_MAIN;
            }

            break;


        case STATE_DIAGNOSTICS:

            runDiagnostics();

            currentState =
                MENU_MAIN;

            break;


        case STATE_CALIBRATION:

            executeCalibration();


            if (
                currentState !=
                STATE_ERROR
            )
            {
                currentState =
                    MENU_MAIN;
            }

            break;


        case STATE_ERROR:

            stopMotors();


            display.clearDisplay();

            display.setCursor(
                0,
                20
            );


            display.println(
                "SYSTEM ERROR!"
            );


            display.println(
                "Hardware/Timeout Fail"
            );


            display.display();


            delay(1000);

            break;
    }
}
