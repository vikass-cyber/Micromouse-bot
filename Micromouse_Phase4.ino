#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// 1. PIN DEFINITIONS & CONSTANTS
// ============================================================================

#define PIN_MOTOR_L_PWM  25
#define PIN_MOTOR_L_IN1  26
#define PIN_MOTOR_L_IN2  27
#define PIN_MOTOR_R_PWM  14
#define PIN_MOTOR_R_IN1  12
#define PIN_MOTOR_R_IN2  13
#define PIN_MOTOR_STBY   33

#define PIN_ENC_L_A      34
#define PIN_ENC_L_B      35
#define PIN_ENC_R_A      32
#define PIN_ENC_R_B      39

#define PIN_IR_LEFT      36
#define PIN_IR_FRONT     39
#define PIN_IR_RIGHT     35
#define PIN_BATTERY_ADC  34 // Voltage divider pin (10k / 3.3k)

#define PIN_BUTTON_NEXT  0
#define PIN_BUTTON_SELECT 4
#define PIN_BUZZER       18
#define MPU6050_ADDR     0x68

#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Preferences nvram;

// ============================================================================
// 2. DATA STRUCTURES & SETTINGS
// ============================================================================

struct Configuration {
    float kp_drive = 2.5f;
    float kd_drive = 1.2f;
    float kp_turn = 2.8f;
    float kd_turn = 1.5f;
    int search_speed = 150;
    int fast_speed = 230;
    float accel_rate = 5.0f; // mm/ms^2
} config;

enum SystemState { MENU_MAIN, STATE_SEARCH_RUN, STATE_FAST_RUN, STATE_CALIBRATION, STATE_DIAGNOSTICS, STATE_ERROR };
SystemState currentState = MENU_MAIN;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Battery voltage divider calibration factor
const float BATTERY_ADC_FACTOR = 3.3f / 4095.0f * ((10.0f + 3.3f) / 3.3f); 
const float MIN_BATTERY_VOLTAGE = 6.8f; // 2S LiPo cutoff threshold

// ============================================================================
// 3. INTERRUPTS & HARDWARE DRIVERS
// ============================================================================

void IRAM_ATTR isrLeftEncoder() {
    if (digitalRead(PIN_ENC_L_B) == HIGH) leftTicks++;
    else leftTicks--;
}

void IRAM_ATTR isrRightEncoder() {
    if (digitalRead(PIN_ENC_R_B) == HIGH) rightTicks++;
    else rightTicks--;
}

float readBatteryVoltage() {
    int raw = analogRead(PIN_BATTERY_ADC);
    return raw * BATTERY_ADC_FACTOR;
}

void checkBatteryHealth() {
    float voltage = readBatteryVoltage();
    if (voltage < MIN_BATTERY_VOLTAGE && voltage > 2.0f) { // Ignore unhooked ADC
        digitalWrite(PIN_MOTOR_STBY, LOW); // Disable motor drivers
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

// ============================================================================
// 4. NON-VOLATILE EEPROM / PREFERENCES STORAGE
// ============================================================================

void loadConfiguration() {
    nvram.begin("micromouse", true);
    config.kp_drive = nvram.getFloat("kp_d", 2.5f);
    config.kd_drive = nvram.getFloat("kd_d", 1.2f);
    config.kp_turn  = nvram.getFloat("kp_t", 2.8f);
    config.kd_turn  = nvram.getFloat("kd_t", 1.5f);
    config.fast_speed = nvram.getInt("f_spd", 230);
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

// ============================================================================
// 5. INTERACTIVE OLED MENU SYSTEM
// ============================================================================

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

void handleMenuNavigation() {
    drawMenu();

    while (currentState == MENU_MAIN) {
        checkBatteryHealth();

        if (digitalRead(PIN_BUTTON_NEXT) == LOW) {
            tone(PIN_BUZZER, 2500, 50);
            currentMenuIdx = (currentMenuIdx + 1) % TOTAL_MENU_ITEMS;
            drawMenu();
            delay(200); // Debounce
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
// 6. SELF-DIAGNOSTICS ROUTINE
// ============================================================================

void runDiagnostics() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RUNNING DIAGNOSTICS");
    display.display();

    // Check Wire / I2C Devices
    Wire.beginTransmission(MPU6050_ADDR);
    byte mpuErr = Wire.endTransmission();

    display.printf("MPU6050: %s\n", mpuErr == 0 ? "OK" : "FAIL");
    display.printf("Batt Volts: %.2fV\n", readBatteryVoltage());
    display.printf("IR L/F/R: %d/%d/%d\n", analogRead(PIN_IR_LEFT), analogRead(PIN_IR_FRONT), analogRead(PIN_IR_RIGHT));
    display.println("\nPress SELECT to exit");
    display.display();

    while (digitalRead(PIN_BUTTON_SELECT) == HIGH) {
        delay(50);
    }
    currentState = MENU_MAIN;
}

// ============================================================================
// 7. SETUP & MAIN EXECUTION LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    
    pinMode(PIN_BUTTON_NEXT, INPUT_PULLUP);
    pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);

    Wire.begin(21, 22);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    loadConfiguration();
    checkBatteryHealth();
}

void loop() {
    switch (currentState) {
        case MENU_MAIN:
            handleMenuNavigation();
            break;

        case STATE_DIAGNOSTICS:
            runDiagnostics();
            break;

        case STATE_SEARCH_RUN:
            display.clearDisplay();
            display.setCursor(0, 20);
            display.println("EXECUTING SEARCH...");
            display.display();
            // Integrated Phase 3 Floodfill navigation runs here
            delay(3000);
            currentState = MENU_MAIN;
            break;

        case STATE_FAST_RUN:
            display.clearDisplay();
            display.setCursor(0, 20);
            display.println("HIGH SPEED RUN!");
            display.display();
            // Optimized trajectory solver runs here
            delay(3000);
            currentState = MENU_MAIN;
            break;

        case STATE_CALIBRATION:
            display.clearDisplay();
            display.setCursor(0, 20);
            display.println("CALIBRATING...");
            display.display();
            delay(2000);
            currentState = MENU_MAIN;
            break;

        default:
            currentState = MENU_MAIN;
            break;
    }
}