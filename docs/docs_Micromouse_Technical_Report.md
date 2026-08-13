# Micromouse Autonomous Maze-Solving Robot
## Technical Engineering Report

---

## 1. Abstract
This technical engineering report details the software architecture, control algorithms, hardware integration, and maze-solving methodology for an autonomous Micromouse robot. Powered by an ESP32 microcontroller, the robot navigates a standard maze grid using an array of infrared distance sensors, incremental quadrature wheel encoders, and an MPU6050 Inertial Measurement Unit (IMU). The core algorithm relies on a modified Breadth-First Search (BFS) Flood-Fill implementation to dynamically compute optimal target distances, coupled with real-time wall sensing, closed-loop PD steering feedback, and gyro-assisted heading correction. The system features an integrated OLED user interface, non-volatile configuration persistence via NVRAM, real-time diagnostic telemetry, and battery health monitoring.

---

## 2. Introduction
Micromouse is an engineering competition in which small, autonomous robotic vehicles are tasked with solving an unknown maze from a designated corner starting square to a central destination area. To achieve competitive run times, the robot must execute two distinct operational phases:
1. **Search Run (Exploration Phase):** The robot explores the maze, maps wall boundaries, computes cell distance fields, and identifies a viable route to the target center.
2. **Fast Run (Speed Phase):** Utilizing the stored topological map and optimized path data, the robot navigates from start to goal at maximum attainable velocity.

This document serves as a comprehensive technical reference for the software source code implemented in `Micromouse_final.ino` and its associated module interfaces (`config.h`, `motors.h`, `sensors.h`, `pid.h`).

---

## 3. Problem Statement
Autonomous maze navigation under strict hardware and physical constraints presents several classical engineering challenges:
- **Incomplete Information:** The maze layout is completely unknown *a priori*, requiring dynamic map building and real-time path updates.
- **Odometry Drift & Kinematic Errors:** Wheel slip, mechanical tolerances, and uneven surfaces cause cumulative positioning errors over long drive distances.
- **Sensor Noise & Environmental Interference:** Infrared sensors suffer from ambient light variations, surface reflectivity differences, and non-linear distance response curves.
- **Resource Constraints:** Real-time pathfinding and sensor fusion must execute within strict memory and timing limits on embedded hardware.

---

## 4. Project Objectives
- **Robust Pathfinding:** Implement a flexible Flood-Fill algorithm capable of dynamic multi-cell goal seeding (e.g., target coordinates `(7,7)`, `(7,8)`, `(8,7)`, `(8,8)`).
- **Closed-Loop Motion Control:** Develop accurate straight-line driving and turn routines utilizing combined encoder tick counting, IR wall-centering PD control, and IMU yaw integration.
- **Hardware Integration & Safety:** Integrate OLED telemetry, dual-button menu navigation, audible status feedback, and hard battery under-voltage safety cutoffs.
- **Persistent Storage:** Provide non-volatile storage capabilities (NVRAM via ESP32 `Preferences`) for PID parameters and velocity profiles.

---

## 5. System Overview
The system architecture follows a hierarchical control model where low-level motor drivers and sensor sampling loops are managed through dedicated hardware modules, while high-level state decisions and algorithm updates are coordinated by the main event loop in `Micromouse_final.ino`.

```
                  +-------------------------+
                  |    User Interface       |
                  | (OLED Display / Buttons)|
                  +------------+------------+
                               |
                               v
                  +-------------------------+
                  |    ESP32 Main Loop      |
                  |  (System State Machine) |
                  +---+------------+----+---+
                      |            |    |
       +--------------+            |    +---------------+
       |                           v                    |
       v                  +-----------------+           v
+--------------+          |   Maze Solver   |    +--------------+
| IR & IMU     |          |  (Flood-Fill /  |    | Motor & PID  |
| Sensors      | -------> |   Navigation)   | -> | Control      |
+--------------+          +-----------------+    +--------------+
```

---

## 6. Hardware Architecture

| Component | Function / Subsystem | Implementation Details |
|---|---|---|
| **Microcontroller** | Master Processor | ESP32 (32-bit Dual-Core microcontroller running Arduino framework) |
| **IMU** | Yaw / Heading Tracking | MPU6050 (I2C Address `0x6B`, Fast I2C mode @ 400kHz) |
| **Display** | Visual Telemetry & UI | 128x64 SSD1306 OLED via I2C (`0x3C`) |
| **Infrared Sensors** | Wall Detection & Centering | Analog IR Phototransistor / LED Pairs (`ir.rawL`, `ir.rawF`, `ir.rawR`) |
| **Encoders** | Distance Measurement | Dual Incremental Quadrature Encoders attached to motor shafts |
| **User Input** | Navigation & Selection | Dual Push Buttons (`PIN_BUTTON_NEXT`, `PIN_BUTTON_SELECT`) with `INPUT_PULLUP` |
| **Audio Alert** | System Status & Warnings | Piezo Buzzer on `PIN_BUZZER` |
| **Power Monitor** | Battery Voltage Sensing | Analog Voltage Divider connected to `PIN_BATTERY_ADC` |

---

## 7. ESP32 Control System
The system utilizes the ESP32 platform running at high clock frequency to perform simultaneous high-speed sensor sampling, interrupt handling, and display rendering.
- **I2C Bus Bus Speed:** Configured via `Wire.setClock(400000)` to 400 kHz for fast sensor reads from MPU6050 and frame updates to the SSD1306 OLED display.
- **Interrupt Services:** Dedicated IRAM-attributed Interrupt Service Routines (`IRAM_ATTR isrLeftEncoder` and `isrRightEncoder`) handle quadrature encoder pulse counting with zero CPU delay overhead.

---

## 8. Sensor System
The sensor pipeline consists of three front/side-facing analog IR channels managed by the `IRSensors` class (`sensors.h` / `sensors.cpp`).
- **Reading Function:** `readIRSensors()` populates the global `ir` structure with raw ADC values (`ir.rawL`, `ir.rawF`, `ir.rawR`) and evaluates boolean wall threshold flags (`ir.wallL`, `ir.wallF`, `ir.wallR`).
- **Wall Detection Thresholds:** Boolean wall presence flags are determined by comparing raw analog sensor readings against calibrated threshold constants (e.g., `WALL_THRESH_LEFT`, `WALL_THRESH_RIGHT`).

---

## 9. Motor and Motor Driver System
Motor control functions are encapsulated in `motors.h` and `motors.cpp`.
- **Interface:** `setMotors(int leftPwm, int rightPwm)` sets directional control pins and PWM duty cycles bounded between `-255` and `255`.
- **Emergency Stop:** `stopMotors()` immediately sets motor driver PWM outputs to 0, clamping vehicle movement.

---

## 10. Wheel Encoder System
Quad-encoder tick counts are captured via rising-edge interrupts on Channel A pins, while Channel B logic states determine pulse direction:
- `isrLeftEncoder()`: Increments `leftTicks` if `PIN_ENC_L_B == HIGH`, else decrements.
- `isrRightEncoder()`: Increments `rightTicks` if `PIN_ENC_R_B == HIGH`, else decrements.
- **Tick-to-Distance Conversion:** Linear travel is computed using `MM_PER_TICK` constant:
  $$	ext{Target Ticks} = rac{	ext{Distance (mm)}}{	ext{MM\_PER\_TICK}}$$

---

## 11. MPU6050 / IMU System
The MPU6050 sensor provides Z-axis angular velocity (gyroscope Z) for heading drift compensation during straight drives and precise closed-loop turn execution.
- **Calibration:** `initMPU6050()` collects 500 samples from register `0x47` while stationary to calculate zero-rate offset `mpu.gyroZ_offset`.
- **Integration:** `updateYaw()` computes instantaneous delta time $dt$ via `micros()`, converts raw reading to degrees per second ($	ext{gz\_dps} = (	ext{rawGZ} - 	ext{offset}) / 131.0$), and integrates yaw angle:
  $$	ext{Yaw}_{t} = 	ext{Yaw}_{t-1} + (	ext{gz\_dps} 	imes dt) \quad 	ext{for } |	ext{gz\_dps}| > 0.5^\circ/	ext{s}$$

---

## 12. OLED / User Interface
The UI uses an Adafruit SSD1306 128x64 display driven by custom menu and dashboard functions:
- **Main Menu (`drawMenu` / `handleMenuNavigation`):** Interactive vertical list selectable via `PIN_BUTTON_NEXT` and `PIN_BUTTON_SELECT`.
- **Dashboard (`renderDashboard`):** Real-time monitoring display printing active pose `(posX, posY)`, current heading, action description, raw IR readings, current yaw angle, and battery voltage.
- **Audible Tones:** Distinct audio feedback frequencies (2500 Hz for navigation, 3500 Hz for selection, 2000 Hz for setup/calibration, 3000-4000 Hz for goal completion).

---

## 13. Battery Monitoring and Safety
Continuous voltage checks are executed inside all blocking movement and navigation loops via `checkBatteryHealth()`:
- **Voltage Calculation:** $$	ext{Voltage} = 	ext{analogRead}(	ext{PIN\_BATTERY\_ADC}) 	imes 	ext{BATTERY\_ADC\_FACTOR}$$
- **Protection Logic:** If voltage drops below `MIN_BATTERY_VOLTAGE` (and remains $> 2.0	ext{V}$ to prevent false triggers when powered solely via USB), motors are stopped, the OLED displays `"LOW BATT!"`, and the piezo buzzer emits an alternating dual-tone alarm ($1000	ext{Hz} / 500	ext{Hz}$) in an infinite lock loop.

---

## 14. Software Architecture

### Software Modules

| Module | Responsibility | Source File Reference |
|---|---|---|
| **System Core** | State machine, hardware setup, menu, movement routines | `Micromouse_final.ino` |
| **System Config** | Hardware pinouts, dimensions, thresholds, parameters | `config.h` |
| **Motor Driver** | Low-level PWM driver interface and motor control | `motors.h` / `motors.cpp` |
| **Sensors** | IR sensor sampling, ADC reading, wall thresholding | `sensors.h` / `sensors.cpp` |
| **PID Controller** | Closed-loop feedback control definitions and data structures | `pid.h` / `pid.cpp` |

---

## 15. Configuration System
System configuration parameters are managed using ESP32 Non-Volatile Storage (NVRAM) through the `Preferences` library (`namespace: "micromouse"`):
- `loadConfiguration()`: Restores $K_{p,drive}$, $K_{d,drive}$, $K_{p,turn}$, $K_{d,turn}$, `search_speed` (default 150), and `fast_speed` (default 230).
- `saveConfiguration()`: Flashes modified parameter values permanently to flash memory.

---

## 16. Maze Representation
The maze is modeled internally within the `MazeSolver` class as a 2D matrix of dimensions `MAZE_SIZE` $	imes$ `MAZE_SIZE` (typically $16 	imes 16$):
- `uint8_t cells[MAZE_SIZE][MAZE_SIZE]`: Stores bitmasked wall data and visitation flags.
- `uint8_t dist[MAZE_SIZE][MAZE_SIZE]`: Stores distance flood values relative to target goal cells.

### Wall Bitmask Definitions
- `WALL_N` (North Wall)
- `WALL_E` (East Wall)
- `WALL_S` (South Wall)
- `WALL_W` (West Wall)
- `VISITED` (Cell Visited Flag)

### Maze Boundary Initialization
In `MazeSolver::init()`, boundary outer walls are set programmatically across all edge cells:
- $y = 	ext{MAZE\_SIZE} - 1 \implies 	ext{WALL\_N}$
- $x = 	ext{MAZE\_SIZE} - 1 \implies 	ext{WALL\_E}$
- $y = 0 \implies 	ext{WALL\_S}$
- $x = 0 \implies 	ext{WALL\_W}$

---

## 17. Maze Mapping
When `scanWallsAndUpdateMap()` is invoked:
1. `readIRSensors()` updates wall presence flags (`ir.wallF`, `ir.wallR`, `ir.wallL`).
2. Bitwise wall assignments are performed based on the robot's active orientation (`heading`):
   - Front wall $ightarrow$ set wall in direction `heading`.
   - Right wall $ightarrow$ set wall in direction `(heading + 1) % 4`.
   - Left wall $ightarrow$ set wall in direction `(heading + 3) % 4`.
3. Neighboring adjacent cells are updated symmetrically via `solver.setWall()` to maintain map consistency (e.g., setting `WALL_N` on cell $(x,y)$ automatically sets `WALL_S` on cell $(x, y+1)$).
4. The cell is marked as visited using `solver.setVisited(posX, posY)`.

---

## 18. Flood-Fill Algorithm
The pathfinder calculates shortest Manhattan-distance metrics across the grid using a Breadth-First Search (BFS) queue (`Point queue[MAX_QUEUE_SIZE]`).

### Algorithm Logic (`updateFloodFill`)
1. All values in `dist[x][y]` are initialized to 255.
2. The queue is cleared (`clearQueue()`).
3. Goal coordinates are seeded with distance `0` and pushed to the queue. The implementation supports multi-cell goal areas (e.g., center cells $(7,7), (7,8), (8,7), (8,8)$):
   - Primary target: $(x_1, y_1)$ set to 0.
   - Additional target cluster: $(x_2, y_1), (x_1, y_2), (x_2, y_2)$ set to 0.
4. While the queue is non-empty:
   - Pop front cell `curr`.
   - Inspect all 4 cardinal directions (North, East, South, West).
   - If no wall blocks movement and the neighbor cell's current distance is unassigned (`255`), set its distance to `dist[curr.x][curr.y] + 1` and push neighbor to queue.

---

## 19. Navigation Algorithm
Optimal next-step direction selection is performed by `getBestNeighborDir(x, y, currentHeading)`:
1. Evaluates all four directions relative to current heading in prioritized order:
   - Priority 1: `currentHeading` (Straight)
   - Priority 2: `(currentHeading + 1) % 4` (Right)
   - Priority 3: `(currentHeading + 3) % 4` (Left)
   - Priority 4: `(currentHeading + 2) % 4` (Backward / 180 Turn)
2. Checks whether a wall blocks passage in direction `d`.
3. Computes target neighbor cell coordinates $(nx, ny)$.
4. Selects direction $d$ that minimizes target distance `dist[nx][ny]`.

---

## 20. Direction and Robot Position Tracking
Directional headings are enumerated as integer constants:
- `NORTH = 0`
- `EAST  = 1`
- `SOUTH = 2`
- `WEST  = 3`

### Position Update Logic (`updateCoordinates`)
Whenever the mouse successfully moves forward by one cell:
- `NORTH`: `posY` incremented (if $< 	ext{MAZE\_SIZE} - 1$)
- `EAST`: `posX` incremented (if $< 	ext{MAZE\_SIZE} - 1$)
- `SOUTH`: `posY` decremented (if $> 0$)
- `WEST`: `posX` decremented (if $> 0$)

---

## 21. Motion Commands
High-level navigation maps neighbor choices directly into movement primitive commands (`MotionCmd`):
- `CMD_FORWARD_1`: Advance forward 1 cell distance (`CELL_DISTANCE_MM`).
- `CMD_TURN_RIGHT`: Execute $+90^\circ$ turn, update heading $ightarrow (h + 1) \% 4$.
- `CMD_TURN_LEFT`: Execute $-90^\circ$ turn, update heading $ightarrow (h + 3) \% 4$.
- `CMD_TURN_180`: Execute $180^\circ$ turn, update heading $ightarrow (h + 2) \% 4$.
- `CMD_STOP`: Halt motor control.

---

## 22. PID / Motion Control

### Straight Driving (`driveDistance`)
- Target encoder ticks derived from `CELL_DISTANCE_MM`.
- Deceleration profile: when remaining ticks $< 300$, PWM velocity scales down from `maxSpeed` to 60 using `map()`.
- Steering correction ($	ext{steeringCorrection}$):
  1. **Dual Wall Present (`ir.wallL && ir.wallR`):** PD controller on side sensor difference:
     $$	ext{error} = 	ext{ir.rawL} - 	ext{ir.rawR}$$
     $$	ext{steeringCorrection} = (	ext{error} 	imes 0.05) + ((	ext{error} - 	ext{lastError}) 	imes 0.01)$$
  2. **Left Wall Only (`ir.wallL`):**
     $$	ext{error} = 	ext{ir.rawL} - 	ext{WALL\_THRESH\_LEFT}$$
     $$	ext{steeringCorrection} = 	ext{error} 	imes 0.04$$
  3. **Right Wall Only (`ir.wallR`):**
     $$	ext{error} = 	ext{WALL\_THRESH\_RIGHT} - 	ext{ir.rawR}$$
     $$	ext{steeringCorrection} = 	ext{error} 	imes 0.04$$
  4. **No Side Walls (Open Space):** Gyroscope feedback:
     $$	ext{steeringCorrection} = 	ext{mpu.yaw} 	imes 	ext{config.kp\_drive}$$

### Angular Turning (`turnAngle`)
- Closed-loop angular control driven by gyroscope integration:
  $$	ext{error} = 	ext{targetAngleDeg} - 	ext{mpu.yaw}$$
  $$	ext{output} = (	ext{error} 	imes 	ext{config.kp\_turn}) + ((	ext{error} - 	ext{lastError}) 	imes 	ext{config.kd\_turn})$$
- Output PWM is constrained between $-200$ and $+200$. Minimum stiction-breaking threshold enforces $|	ext{speed}| \ge 60$.
- Termination condition: $|	ext{error}| < 1.0^\circ$ or safety timeout (1500 ms).

---

## 23. Search / Exploration Process
1. Initialize solver structure, reset position to $(0,0)$, set heading to `NORTH`.
2. Compute distance matrix toward goal $(7,7), (8,8)$.
3. Check if current position distance equals `0`. If true, trigger goal tone, stop motors, and exit loop.
4. Scan physical environment using IR sensors and update internal map (`scanWallsAndUpdateMap()`).
5. Re-run Flood-Fill algorithm to refresh distance values across updated walls (`updateFloodFill()`).
6. Query solver for next motion command (`getNextMotionCommand()`).
7. Execute physical movement primitive.
8. If movement fails (timeout/stall), trigger `STATE_ERROR`.

---

## 24. Goal Detection
Goal arrival is validated whenever `solver.getDistance(posX, posY) == 0`. Upon reaching goal:
- High-pitched confirmation tone (3000 Hz for 1000 ms) plays.
- Motors halt immediately.
- Display prints `"GOAL REACHED!"`.
- Global variable `mazeSearchCompleted` is set to `true`, enabling access to Fast Run mode.

---

## 25. Fast / Optimized Run
1. Verifies `mazeSearchCompleted == true`. If false, displays `"ERROR: NO MAZE DATA!"` and returns to menu.
2. Resets robot pose to $(0,0)$, `heading = NORTH`.
3. Runs flood-fill on stored wall representation.
4. Loops until goal distance reaches `0`:
   - Queries `getNextMotionCommand()` using mapped cell data.
   - Executes straight movement at `config.fast_speed` (PWM 230).
   - Executes turns at optimal angular speeds.
5. Displays `"FAST RUN DONE!"` upon completion.

---

## 26. State Machine
System states are defined in `SystemState` enum:
- `MENU_MAIN`: Main user menu navigation.
- `STATE_SEARCH_RUN`: Active exploration and mapping phase.
- `STATE_FAST_RUN`: Speed run along mapped optimal route.
- `STATE_DIAGNOSTICS`: Real-time display of raw IR values, encoder ticks, and IMU yaw.
- `STATE_CALIBRATION`: Stationary IMU gyroscope zero-bias calibration.
- `STATE_ERROR`: Safety lockout triggered on movement timeout or hardware failure.

---

## 27. Error Handling
- **Hardware Failures:** If MPU6050 initialization fails during setup or calibration, `currentState` immediately transitions to `STATE_ERROR`.
- **Movement Timeout:** Both `driveDistance()` (3000 ms limit) and `turnAngle()` (1500 ms limit) return `false` if target thresholds are not achieved within timeout windows.
- **Error Lockout:** Under `STATE_ERROR`, motors are stopped, and the display renders `"SYSTEM ERROR! Hardware/Timeout Fail"`.

---

## 28. Engineering Design Decisions
- **Sensor Fusion for Alignment:** Combining side IR wall-distance sensing with IMU heading tracking ensures straight travel in both narrow corridors and open intersections.
- **Dynamic Centering:** Proportional wall-centering prevents mechanical scraping against side walls without requiring absolute global positioning systems.
- **Separation of Concerns:** Modular abstraction (`motors`, `sensors`, `pid`, `solver`) ensures cleanly isolated testing and rapid firmware maintainability.

---

## 29. Challenges Encountered During Development
- **Gyroscope Integration Drift:** Gyro Z bias drift over time was mitigated by implementing a 500-sample averaging zero-calibration routine during boot and calibration modes.
- **Wall Discontinuity in Intersections:** When side walls drop off at turns, wall-based PD control becomes invalid. The controller automatically transitions steering feedback to IMU yaw tracking (`mpu.yaw * config.kp_drive`).

---

## 30. Testing and Validation
- **Firmware Compilation:** Source code compiled successfully against ESP32 Arduino framework target architecture.
- **Algorithm Verification:** Flood-Fill BFS queue dynamics and coordinate conversion verified via diagnostic logging routines.
- **Physical Test Results:** Pending physical validation.

---

## 31. Current Project Status
- Complete implementation of maze solver engine, BFS flood fill, motion primitives, and NVRAM configuration.
- Operational OLED dashboard UI, diagnostics menu, and safety battery monitoring routines.
- System ready for physical maze calibration and track trials.

---

## 32. Limitations
- **Fixed Turn Geometry:** Turns currently rely on in-place spot rotation rather than continuous smooth curved turns.
- **IR Reflectivity Sensitivity:** IR distance measurements depend on uniform side-wall material reflectivity.

---

## 33. Future Improvements
- **Diagonal Path Smoothing:** Implement diagonal cell traversal during Fast Run phase to minimize total turn overhead.
- **Dynamic Velocity Profiling:** Implement smooth S-curve acceleration and deceleration profiles for high-speed runs.

---

## 34. Conclusion
The implemented Micromouse control system provides a robust framework for autonomous maze navigation. By integrating Flood-Fill pathfinding, multi-sensor feedback control, and protective embedded safety features on the ESP32 platform, the robot is equipped to reliably explore unknown environments and perform optimized fast runs.
