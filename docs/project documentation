# ESP32 Autonomous Micromouse Project Documentation

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Project Objective](#2-project-objective)
3. [How the Micromouse Robot Works](#3-how-the-micromouse-robot-works)
4. [Hardware Overview](#4-hardware-overview)
5. [Motor System](#5-motor-system)
6. [Sensor System](#6-sensor-system)
7. [Encoder Feedback](#7-encoder-feedback)
8. [MPU6050 / IMU System](#8-mpu6050--imu-system)
9. [PID / Motion Control](#9-pid--motion-control)
10. [Maze Mapping](#10-maze-mapping)
11. [Flood-Fill / Maze-Solving Algorithm](#11-flood-fill--maze-solving-algorithm)
12. [Navigation System](#12-navigation-system)
13. [Robot State / Operating Modes](#13-robot-state--operating-modes)
14. [Safety and Error Handling](#14-safety-and-error-handling)
15. [Configuration System](#15-configuration-system)
16. [Software Architecture](#16-software-architecture)
17. [Hardware + Software Interaction](#17-hardware--software-interaction)
18. [Engineering Challenges](#18-engineering-challenges)
19. [Current Project Status](#19-current-project-status)
20. [Limitations](#20-limitations)
21. [Future Improvements](#21-future-improvements)
22. [Conclusion](#22-conclusion)

---

## 1. Project Overview
This project presents an ESP32-based autonomous Micromouse maze-solving robot. Built on the Arduino ESP32 framework, the system integrates real-time sensor processing, closed-loop motion control, non-volatile persistent storage, and an interactive menu interface driven by an SSD1306 OLED display.

The software architecture is structured into modular components:
* `Micromouse_final.ino`: Core entry point containing system setup, main execution loop, state machine, menu system, and high-level navigation logic.
* `config.h`: System constants, pin definitions, macro declarations, data structures, and default parameters.
* `motors.h` / `motors.cpp`: Low-level motor driver initialization, PWM signal generation, and direction control interface.
* `sensors.h` / `sensors.cpp`: Analog Infrared (IR) distance sensor processing, threshold detection, and wall identification logic.
* `pid.h` / `pid.cpp`: Closed-loop feedback controller data structures and utility routines.

---

## 2. Project Objective
The primary objective of the system is to autonomously explore an unknown grid maze, detect surrounding walls, build a topological representation of the environment, calculate optimal paths to target destination coordinates using a Queue-based Flood-Fill algorithm, and execute a high-speed optimized path run (Fast Run) once exploration is complete.

---

## 3. How the Micromouse Robot Works
The robot operates through a cyclical execution pipeline:
1. **Perception**: At each maze cell, three IR proximity sensors scan for Left, Front, and Right walls. Dual quadrature wheel encoders measure linear displacement, and an MPU6050 6-axis IMU measures rotational rate to estimate robot heading.
2. **State Estimation**: The robot maintains internal grid coordinates `(posX, posY)` and current orientation `heading` (NORTH, EAST, SOUTH, WEST).
3. **Map Updating**: Sensor readings update the global wall grid `cells[MAZE_SIZE][MAZE_SIZE]` bitmask flags and mark visited cells.
4. **Path Planning**: The `MazeSolver` engine executes a Breadth-First Search (BFS) Flood-Fill routine from the destination coordinates `(7,7), (8,8)` back to the current position to recalculate cell distance values.
5. **Command Generation**: Based on adjacent distance gradients and priority preferences, the robot determines the next motion command (`CMD_FORWARD_1`, `CMD_TURN_RIGHT`, `CMD_TURN_LEFT`, `CMD_TURN_180`).
6. **Execution**: Motion control routines drive the motors using feedback control (IR side-wall centering, gyro yaw correction, and encoder tick counting) to complete cell transitions or turns safely.

---

## 4. Hardware Overview

| Subsystem / Peripheral | Hardware / Interface Pin | Description / Source Code Parameter |
| :--- | :--- | :--- |
| Microcontroller | ESP32 | Dual-core processor running Arduino ESP32 framework |
| Display | Adafruit SSD1306 (OLED) | I2C (`0x3C`), SDA: GPIO 21, SCL: GPIO 22, Clock: 400kHz |
| IMU | MPU6050 | I2C (`MPU6050_ADDR`), Gyro Z-axis integration |
| Encoders | Quadrature Encoders | Left Interrupt: `PIN_ENC_L_A`, Direction: `PIN_ENC_L_B`<br>Right Interrupt: `PIN_ENC_R_A`, Direction: `PIN_ENC_R_B` |
| IR Sensors | 3x Analog IR Proximity | Left: `rawL`, Front: `rawF`, Right: `rawR` |
| User Interface | 2x Push Buttons, 1x Buzzer | Next: `PIN_BUTTON_NEXT`, Select: `PIN_BUTTON_SELECT`, Audio: `PIN_BUZZER` |
| Power Monitoring | ADC Line | Pin: `PIN_BATTERY_ADC`, Multiplier: `BATTERY_ADC_FACTOR` |
| Non-Volatile Memory | ESP32 NVS / Preferences | Namespace: `"micromouse"` |

*Note: Specific physical robot dimensions, chassis measurements, motor gear ratios, and exact battery pack capacities are not specified in the current implementation.*

---

## 5. Motor System
The motor subsystem interface is declared in `motors.h` and implemented in `motors.cpp`. High-level motion functions invoke `setMotors(int leftPwm, int rightPwm)` and `stopMotors()`.

* **PWM Range**: Speeds are controlled via 8-bit signed PWM values mapping from `-255` (full reverse) to `+255` (full forward).
* **Speed Profiles**:
  * **Search Speed**: Loaded from NVRAM key `"s_spd"` (default: `150` PWM).
  * **Fast Speed**: Loaded from NVRAM key `"f_spd"` (default: `230` PWM).
  * **Turn Threshold**: Minimum speed threshold during dynamic turns enforced at PWM `60` to avoid deadband motor stall.

---

## 6. Sensor System
The sensor subsystem is managed via `IRSensors` structure declared in `sensors.h` and initialized in `sensors.cpp`.

### Wall Detection & Thresholding
During execution, `readIRSensors()` populates raw analog voltage readings (`rawL`, `rawF`, `rawR`) and updates boolean flags (`wallL`, `wallF`, `wallR`):
* **Front Wall Detection**: Triggered when `rawF` exceeds threshold criteria.
* **Side Wall Detection**: Left and Right wall flags are established against predefined static thresholds (`WALL_THRESH_LEFT` and `WALL_THRESH_RIGHT`).
* **Steering Error Calculation**:
  * Dual Wall Mode (`ir.wallL && ir.wallR`): Differential wall distance `error = ir.rawL - ir.rawR`.
  * Left Wall Only Mode (`ir.wallL`): Distance deviation `error = ir.rawL - WALL_THRESH_LEFT`.
  * Right Wall Only Mode (`ir.wallR`): Distance deviation `error = WALL_THRESH_RIGHT - ir.rawR`.

---

## 7. Encoder Feedback
Encoder counting is achieved using Hardware Interrupts on the ESP32:
* **Interrupt Handlers**: Attached to rising edges (`RISING`) on `PIN_ENC_L_A` (`isrLeftEncoder`) and `PIN_ENC_R_A` (`isrRightEncoder`).
* **Direction Decoding**: The ISR reads the secondary channel (`PIN_ENC_L_B` / `PIN_ENC_R_B`) state:
  * If HIGH: tick count increments (`leftTicks++` / `rightTicks++`).
  * If LOW: tick count decrements (`leftTicks--` / `rightTicks--`).
* **Distance Calculation**: Converts linear displacement into encoder pulses via `MM_PER_TICK`:
  $$	ext{targetTicks} = rac{	ext{distanceMM}}{	ext{MM\_PER\_TICK}}$$
* **Deceleration Mapping**: When driving straight, as remaining ticks drop below 300 (`remainingTicks < 300`), motion speed is dynamically scaled down to prevent overshoot:
  $$	ext{speed} = 	ext{map}(	ext{remainingTicks}, 0, 300, 60, 	ext{maxSpeed})$$

---

## 8. MPU6050 / IMU System
An MPU6050 6-axis Motion Tracking Device connected via I2C (`0x68` / `MPU6050_ADDR`) supplies angular rate feedback around the vertical Z-axis for yaw orientation control.

### Gyro Calibration & Yaw Integration
1. **Zero-Offset Calibration (`initMPU6050`)**:
   * Reads 500 samples from Gyro Z register (`0x47`) while the robot is stationary.
   * Averages samples to calculate `mpu.gyroZ_offset`.
2. **Yaw Tracking (`updateYaw`)**:
   * Computes time delta `dt = (now - lastTime) / 1000000.0f` (with sanity checking for $0 < dt \le 0.5	ext{s}$).
   * Converts raw readings to degrees per second (dps) using scale factor $131.0\,	ext{LSB}/(\deg/	ext{s})$:
     $$	ext{gz\_dps} = rac{	ext{rawGZ} - 	ext{gyroZ\_offset}}{131.0}$$
   * Applies a deadband threshold (`abs(gz_dps) > 0.5f`) to eliminate stationary drift.
   * Integrates yaw angle: $	ext{yaw} = 	ext{yaw} + (	ext{gz\_dps} 	imes dt)$.

---

## 9. PID / Motion Control
Motion control combines open-loop profile generation with closed-loop multi-sensor feedback.

### Straight Line Drive (`driveDistance`)
To maintain straight progression down cell corridors:
* **Primary Side Centering**: Uses IR sensor error inputs. If both walls exist, a Proportional-Derivative (PD) loop acts on sensor imbalance:
  $$	ext{steeringCorrection} = (K_{p,ir} 	imes 	ext{error}) + (K_{d,ir} 	imes \Delta	ext{error})$$
  *Where $K_{p,ir} = 0.05$ and $K_{d,ir} = 0.01$.*
* **Single Wall Centering**: If only one side wall is present, proportional feedback ($K_p = 0.04$) keeps the robot aligned relative to that wall.
* **Gyro Fallback**: In open corridors with no side walls (`!ir.wallL && !ir.wallR`), heading maintenance falls back onto IMU yaw integration:
  $$	ext{steeringCorrection} = 	ext{mpu.yaw} 	imes K_{p,	ext{drive}}$$
  *Where $K_{p,	ext{drive}}$ is configurable (default `2.5`).*

### Turn Angle Control (`turnAngle`)
Closed-loop turning (+90°, -90°, 180°) is executed using IMU yaw angle feedback:
* Closed-loop error: $	ext{error} = 	ext{targetAngleDeg} - 	ext{mpu.yaw}$.
* Proportional-Derivative Control calculation:
  $$	ext{output} = (	ext{error} 	imes K_{p,	ext{turn}}) + (\Delta	ext{error} 	imes K_{d,	ext{turn}})$$
  *Default configuration values: $K_{p,	ext{turn}} = 2.8$, $K_{d,	ext{turn}} = 1.5$.*
* Minimum PWM threshold enforcement prevents motor deadband stall when error is small.

---

## 10. Maze Mapping
The maze representation is stored in memory as bitmask arrays within the `MazeSolver` class:
* `cells[MAZE_SIZE][MAZE_SIZE]`: Standard grid array initialized with outer bounding walls (`WALL_N`, `WALL_E`, `WALL_S`, `WALL_W`).
* **Bitmask Flags**:
  * `WALL_N` = North Wall bit flag
  * `WALL_E` = East Wall bit flag
  * `WALL_S` = South Wall bit flag
  * `WALL_W` = West Wall bit flag
  * `VISITED` = Visited flag indicator
* **Wall Synchronization**: Calls to `setWall(x, y, dir)` set the wall flag on cell `(x,y)` and automatically update the adjacent cell's reciprocal wall flag (e.g., setting `WALL_N` on `(x,y)` updates `WALL_S` on `(x, y+1)`).

---

## 11. Flood-Fill / Maze-Solving Algorithm
The maze solver utilizes a classical Queue-based Breadth-First Search (BFS) Flood-Fill implementation.

### Distance Grid Calculation (`updateFloodFill`)
1. Resets distance array `dist[MAZE_SIZE][MAZE_SIZE]` values to `255` (unreached).
2. Sets destination center cells `(7,7), (7,8), (8,7), (8,8)` distance to `0` and pushes them into `queue`.
3. Pops cells from `queue` sequentially, inspecting four cardinal directions (NORTH, EAST, SOUTH, WEST):
   * If no wall blocks passage and neighbor distance is unassigned (`255`), set neighbor distance = `currentDistance + 1` and push neighbor into `queue`.
4. Process terminates when the queue is emptied, producing a discrete potential field where every accessible cell contains its Manhattan/topological step distance to target center.

---

## 12. Navigation System
Once distance maps are updated, `getBestNeighborDir()` selects movement paths:
* Inspects adjacent cells in priority order starting from `currentHeading`:
  1. Straight ahead (`currentHeading`)
  2. Right turn `(currentHeading + 1) % 4`
  3. Left turn `(currentHeading + 3) % 4`
  4. Turn 180° `(currentHeading + 2) % 4`
* Selects the accessible neighbor with the lowest numeric `dist[nx][ny]` value.
* `getNextMotionCommand()` translates target direction into discrete robot movement commands (`CMD_FORWARD_1`, `CMD_TURN_RIGHT`, `CMD_TURN_LEFT`, `CMD_TURN_180`).

---

## 13. Robot State / Operating Modes
System execution is governed by an explicit finite state machine (`SystemState`):

```
       +-------------------------------------------------------+
       |                                                       |
       v                                                       |
[ MENU_MAIN ] ---> [ STATE_SEARCH_RUN ] ----(Completed)--------+
      |      ---> [ STATE_FAST_RUN ]   ----(Completed)--------+
      |      ---> [ STATE_DIAGNOSTICS ] ----(Button Exit)------+
      |      ---> [ STATE_CALIBRATION ] ----(Completed)--------+
      |                                                        |
      +----------> [ STATE_ERROR ] <---(Timeout/Fault)---------+
```

| Operating State | Description |
| :--- | :--- |
| `MENU_MAIN` | Renders menu interface on OLED display. Navigated via `PIN_BUTTON_NEXT` and `PIN_BUTTON_SELECT`. |
| `STATE_SEARCH_RUN` | Explores maze, maps walls dynamically, updates flood fill grid, stops upon reaching target goal `(7,7)`. Sets flag `mazeSearchCompleted = true`. |
| `STATE_FAST_RUN` | High-speed path execution using stored wall map data. Rejects execution with error message if `mazeSearchCompleted` is false. |
| `STATE_DIAGNOSTICS` | Real-time monitoring mode displaying raw IR readings, wall state flags, encoder counters, yaw angle, and battery voltage. |
| `STATE_CALIBRATION` | Re-runs MPU6050 zero-gyro bias calculation while robot is stationary. |
| `STATE_ERROR` | Halts motor movement upon drive timeouts, IMU failures, or hardware fault detection. Displays error status on OLED display. |

---

## 14. Safety and Error Handling
* **Low Battery Protection (`checkBatteryHealth`)**: Monitors battery voltage using ADC reading (`readBatteryVoltage()`). If voltage drops below threshold (`MIN_BATTERY_VOLTAGE`) and remains above $2.0\,	ext{V}$, motors halt, "LOW BATT!" warning displays on screen, and an alternating dual-tone audio alarm loop plays indefinitely.
* **Hardware Failure Handling**: Failure to initialize OLED or MPU6050 forces state machine into `STATE_ERROR`.
* **Timeout Guards**: Motion commands include hard execution timeouts (e.g., 3000ms max for driving distance, 1500ms max for turns). If physical blockages occur, routines fail safely, stop motors, and set `STATE_ERROR`.

---

## 15. Configuration System
System tuning parameters are persistently stored using ESP32 Non-Volatile Storage (Preferences NVS) under namespace `"micromouse"`:

| Parameter Key | Code Variable | Default Value | Description |
| :--- | :--- | :--- | :--- |
| `"kp_d"` | `config.kp_drive` | `2.5f` | Proportional gain for straight line drive control |
| `"kd_d"` | `config.kd_drive` | `1.2f` | Derivative gain for straight line drive control |
| `"kp_t"` | `config.kp_turn` | `2.8f` | Proportional gain for turning angle control |
| `"kd_t"` | `config.kd_turn` | `1.5f` | Derivative gain for turning angle control |
| `"s_spd"` | `config.search_speed` | `150` | Search run motor PWM speed profile |
| `"f_spd"` | `config.fast_speed` | `230` | Fast run motor PWM speed profile |

Configurations can be adjusted and written to Flash via menu item **"4. Save Config"** (`saveConfiguration()`).

---

## 16. Software Architecture
The code architecture employs modular design principles separating driver interfaces, algorithmic solver logic, hardware input/output, and execution management:

```
+-------------------------------------------------------------------+
|                        Micromouse_final.ino                       |
|        (Setup, Main Loop, Menu Interface, OS State Machine)       |
+-------------------------------------------------------------------+
        |                    |                   |             |
        v                    v                   v             v
+---------------+   +-----------------+   +------------+  +-------------------+
|  MazeSolver   |   | Motion Control  |   | IRSensors  |  |   MPUData (IMU)   |
| (Flood-Fill,  |   | (driveDistance, |   | (sensors.h/|  | (updateYaw, Gyro  |
| Wall Bitmaps) |   |  turnAngle)     |   |   .cpp)    |  |  Zero-Calibration)|
+---------------+   +-----------------+   +------------+  +-------------------+
                             |                  |               |
                             +--------+---------+---------------+
                                      |
                                      v
                             +------------------+
                             |   Motors & PWM   |
                             | (motors.h/.cpp)  |
                             +------------------+
```

---

## 17. Hardware + Software Interaction
1. **Interrupt Service Routines (ISRs)** execute on core pin edges asynchronously to capture raw wheel tick increments.
2. **Periodic Task Loop** inside drive routines updates IMU integrate angles (`updateYaw()`) and reads analog IR distances (`readIRSensors()`).
3. **Control Loop** calculates tracking error, applies PD gain factors, constrains duty cycles to `[-255, 255]`, and outputs hardware PWM duty cycles via motor driver channels.
4. **I2C Bus** operates at standard Fast-Mode $400\,	ext{kHz}$ to multiplex display buffer writes (`Adafruit_SSD1306`) and IMU register reading.

---

## 18. Engineering Challenges
* **Sensor Noise & Drift**: Yaw integration from Gyro readings inherently suffers from integration drift over prolonged runs. The implementation addresses this via zero-offset calibration over 500 samples at boot and threshold deadbanding (`abs(gz_dps) > 0.5f`).
* **Non-Linear Motor Deadbands**: Small motor duty cycles fail to overcome mechanical friction. The turn algorithm compensates by enforcing a floor threshold of PWM `60` whenever turn corrections are active.
* **Corridor Centering Without Side Walls**: In open maze sections lacking parallel walls, IR feedback is unavailable. The control algorithm seamlessly switches between IR differential steering and IMU yaw correction.

---

## 19. Current Project Status
* **Compilation Status**: Fully compiling successfully on ESP32 Arduino framework.
* **Features Implemented**:
  * Onboard SSD1306 OLED menu system and diagnostics screen.
  * Quadrature encoder tick counting via hardware interrupts.
  * MPU6050 I2C driver with zero-bias offset calibration and yaw tracking.
  * Queue-based BFS Flood-Fill maze solver algorithm.
  * Non-volatile configuration save/load via ESP32 NVS Preferences.
  * Closed-loop drive distance and turn execution with timeouts.
  * Low battery voltage monitoring and acoustic alert loop.

---

## 20. Limitations
* **Physical Specifications**: Physical dimensions, chassis weight, gear ratio, and exact battery chemistry are *not specified in the current implementation*.
* **Diagonal / Smooth Motion**: Motion execution is currently constrained to discrete grid-step movements (`CMD_FORWARD_1`, `CMD_TURN_RIGHT`, `CMD_TURN_LEFT`, `CMD_TURN_180`). Diagonal path cutting is *not specified in the current implementation*.
* **Dynamic Velocity Profiling**: Acceleration profile generation (s-curves or trapezoidal profiles) is *not specified in the current implementation*.
* **IR Sensor Calibration**: Automated multi-point distance curve mapping for IR sensors is *not specified in the current implementation* (static thresholds are currently utilized).

---

## 21. Future Improvements
* **Diagonal Path Execution**: *Future Improvement.* Integrate smooth curve turns and diagonal cell traversal during the Fast Run phase.
* **Trapezoidal Motion Profiling**: *Future Improvement.* Implement feed-forward acceleration curves to mitigate wheel slip during quick acceleration.
* **Automatic IR Sensor Distance Calibration**: *Future Improvement.* Expand calibration modes to compute distance lookup tables dynamically.
* **Sub-Cell Odometry Fusion**: *Future Improvement.* Utilize Extended Kalman Filtering (EKF) to combine encoder ticks, IMU rates, and IR distances into a unified state estimator.

---

## 22. Conclusion
The ESP32 Autonomous Micromouse project provides a robust, modular, and functional robotics architecture combining real-time control, sensing, and algorithmic pathfinding. By coupling hardware-interrupt driven encoder counting and IMU angular tracking with a Queue-based BFS Flood-Fill solver, the system successfully realizes full exploratory maze mapping and fast path execution on embedded hardware.
