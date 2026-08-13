# Micromouse-bot

> **Real-Time Autonomous Maze Navigation on a Resource-Constrained Embedded Platform**

Micromouse-bot is an autonomous mobile robotics platform designed around the integration of **embedded sensing, closed-loop motion control, online maze reconstruction, state estimation, and graph-based path planning**.

Rather than treating the Micromouse as a simple maze-solving exercise, this project is structured as a compact study in **real-time autonomous systems engineering**: noisy sensors must be converted into a consistent world model, wheel motion must be regulated despite mechanical asymmetry, and navigation decisions must execute within the computational and memory constraints of a microcontroller.

The current implementation uses an **ESP32-class Arduino-compatible platform**, three directional IR wall sensors, quadrature wheel encoders, and an MPU6050 gyroscope. The navigation stack performs **online wall discovery followed by dynamic flood-fill replanning**, while the motion layer uses encoder feedback, gyro-based heading feedback, and wall-relative steering corrections.

---

## 1. Project Objective

The central engineering problem is:

> **How can a small differential-drive robot repeatedly estimate its state, infer previously unknown maze structure, select an optimal local action, and execute that action robustly despite sensor noise and actuator imperfections?**

The system therefore separates the problem into several interacting layers:

```text
┌──────────────────────────────────────────────────────────────────┐
│                    AUTONOMOUS ROBOT SYSTEM                       │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  SENSING          STATE ESTIMATION       WORLD MODEL             │
│  ────────         ───────────────        ───────────             │
│  IR Walls   ───►  Wall / Heading   ───►  Occupancy / Walls       │
│  Encoders   ───►  Odometry              Visited Cells            │
│  MPU6050    ───►  Gyro Integration                              │
│                                                                  │
│                              │                                   │
│                              ▼                                   │
│                    ┌──────────────────┐                          │
│                    │ Flood-Fill / BFS │                          │
│                    │ Path Selection   │                          │
│                    └────────┬─────────┘                          │
│                             │                                    │
│                             ▼                                    │
│                    ┌──────────────────┐                          │
│                    │ Motion Command   │                          │
│                    │ Forward / L / R  │                          │
│                    │ / 180° Turn      │                          │
│                    └────────┬─────────┘                          │
│                             │                                    │
│                             ▼                                    │
│                    CLOSED-LOOP CONTROL                           │
│                    ──────────────────                            │
│                    Encoder Feedback                              │
│                    Gyro Feedback                                 │
│                    IR Wall Alignment                             │
│                             │                                    │
│                             ▼                                    │
│                         ACTUATORS                                │
│                       Left / Right Motor                         │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

The architecture intentionally avoids coupling the maze solver directly to motor commands. The planner produces an abstract **motion command**, while the motion layer is responsible for converting that command into physically achievable movement.

---

# 2. System Configuration

| Subsystem                      | Implementation                                      |
| ------------------------------ | --------------------------------------------------- |
| MCU / Architecture             | **ESP32-class microcontroller**                     |
| Firmware Environment           | Arduino-compatible C++                              |
| Primary Wall Sensors           | **3-channel IR wall sensing: Left / Front / Right** |
| Wheel Feedback                 | Quadrature wheel encoders                           |
| Inertial Sensor                | MPU6050 gyroscope                                   |
| Drive Architecture             | Differential drive                                  |
| Motor Control                  | Closed-loop PWM control                             |
| Translational Feedback         | Encoder tick feedback                               |
| Heading Feedback               | Gyroscope-based yaw integration                     |
| Steering Feedback              | IR wall-relative correction + heading feedback      |
| Maze Representation            | Fixed-size byte-per-cell representation             |
| Path Planning                  | Dynamic Flood-Fill using Breadth-First Search       |
| Navigation                     | Online wall discovery + replanning                  |
| Persistent Configuration       | ESP32 NVS via `Preferences`                         |
| User Interface                 | SSD1306 OLED + buttons + buzzer                     |
| Communication / Peripheral Bus | I²C at 400 kHz                                      |
| Firmware Language              | C++                                                 |

### Important implementation note

The uploaded firmware explicitly uses ESP32-specific facilities such as `IRAM_ATTR` and `Preferences`, making ESP32 the appropriate architectural classification for this implementation.

The motor model/gearbox is intentionally not specified here because the supplied firmware does not establish a specific motor part number. The control architecture is therefore described at the actuator-interface level rather than inventing a hardware BOM.

---

# 3. Hardware Architecture

## Sensor Placement

The three IR sensors form a local geometric observation model around the robot.

```text
                         MAZE WALL
              ─────────────────────────────
                         FRONT
                           │
                     ┌─────┴─────┐
                     │  IR-FRONT │
                     └─────┬─────┘
                           │
                  ┌────────┴────────┐
                  │                 │
          IR-LEFT │     ROBOT       │ IR-RIGHT
                  │                 │
                  │    ┌──────┐     │
                  │    │ MCU  │     │
                  │    └──────┘     │
                  │                 │
                  └───┬─────────┬───┘
                      │           │
                  LEFT MOTOR   RIGHT MOTOR
                    + ENCODER   + ENCODER

              ◄──── Differential Drive ────►
```

The wall sensors provide **local geometric constraints**, while encoders and the gyro provide complementary information about motion.

This combination is important because no single sensing modality is sufficient:

* IR sensors are useful for wall-relative alignment but are sensitive to surface reflectivity and geometry.
* Encoders provide displacement information but accumulate error through wheel slip, backlash, and unequal wheel diameters.
* Gyroscope integration provides short-term angular information but is subject to bias drift.

The firmware therefore uses multiple feedback signals rather than assuming perfect odometry.

---

# 4. Embedded Hardware Block Diagram

```text
                         ┌──────────────────────┐
                         │       ESP32 MCU      │
                         │                      │
                         │  Maze Solver         │
                         │  Motion Controller   │
                         │  State Machine       │
                         │  Sensor Processing   │
                         └───────┬───────┬──────┘
                                 │       │
                 ┌───────────────┘       └────────────────┐
                 │                                        │
                 ▼                                        ▼
        ┌─────────────────┐                     ┌─────────────────┐
        │  IR Wall Array  │                     │    MPU6050      │
        │                 │                     │                 │
        │ LEFT FRONT RIGHT│                     │ Gyro Z-axis    │
        └─────────────────┘                     └─────────────────┘
                 │                                        │
                 └────────────────┬───────────────────────┘
                                  │
                                  ▼
                         Sensor Interpretation
                                  │
                                  ▼
                         ┌─────────────────┐
                         │   Maze Model    │
                         │ Walls + Visited │
                         └────────┬────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │ Flood-Fill BFS  │
                         └────────┬────────┘
                                  │
                           Motion Command
                                  │
                    ┌─────────────┴─────────────┐
                    ▼                           ▼
             ┌────────────┐             ┌────────────┐
             │ Motor L    │             │ Motor R    │
             │ + Encoder  │             │ + Encoder  │
             └─────┬──────┘             └─────┬──────┘
                   │                           │
                   └──────────┬────────────────┘
                              │
                              ▼
                         Robot Motion
                              │
                              └──────► Feedback
```

Additional peripherals include:

* SSD1306 OLED over I²C
* Two user buttons
* Buzzer
* Battery-voltage ADC
* Non-volatile configuration storage

---

# 5. Software Architecture

The firmware is organized into functional layers rather than implementing the entire robot as one monolithic control loop.

```text
Micromouse-bot
│
├── Application Layer
│   ├── Search Run
│   ├── Fast Run
│   ├── Diagnostics
│   └── Calibration
│
├── Navigation Layer
│   ├── MazeSolver
│   ├── Flood-Fill / BFS
│   ├── Neighbor Selection
│   └── Motion Command Generation
│
├── World-Model Layer
│   ├── Wall Mapping
│   ├── Visited State
│   └── Grid Coordinates
│
├── Motion-Control Layer
│   ├── Encoder Odometry
│   ├── Translational Control
│   ├── Heading Control
│   └── Differential Steering
│
├── Sensor Layer
│   ├── IR Wall Sensors
│   └── MPU6050 Gyroscope
│
└── Hardware / Platform Layer
    ├── Motor Driver
    ├── OLED
    ├── NVS Preferences
    ├── GPIO / Interrupts
    └── I²C
```

This decomposition makes it possible to change the navigation algorithm without rewriting the motor-control subsystem.

---

# 6. Real-Time Navigation FSM

The high-level firmware operates as a finite state machine.

```text
                         ┌─────────────┐
                         │   BOOT      │
                         └──────┬──────┘
                                │
                         Hardware Init
                                │
                                ▼
                       ┌────────────────┐
                       │    MENU_MAIN   │◄──────────────┐
                       └───────┬────────┘               │
                               │                        │
             ┌─────────────────┼─────────────────┐      │
             │                 │                 │      │
             ▼                 ▼                 ▼      │
       ┌───────────┐     ┌───────────┐    ┌──────────┐ │
       │ SEARCH    │     │ FAST RUN  │    │DIAGNOSTIC│ │
       │   RUN     │     │           │    │          │ │
       └─────┬─────┘     └─────┬─────┘    └────┬─────┘ │
             │                 │               │       │
             │                 │               │       │
             │       ┌─────────┴─────────┐     │       │
             │       │                   │     │       │
             ▼       ▼                   ▼     ▼       │
       ┌──────────────────────────────────────────┐    │
       │       SENSOR → PLAN → CONTROL LOOP       │    │
       │                                          │    │
       │  1. Sense walls                         │    │
       │  2. Update maze model                   │    │
       │  3. Recompute flood-fill                │    │
       │  4. Select best neighbor                │    │
       │  5. Execute motion                      │    │
       │  6. Update pose                         │    │
       └──────────────────┬───────────────────────┘    │
                          │                            │
                    Goal reached?                     │
                      │       │                        │
                     NO      YES                       │
                      │       │                        │
                      │       ▼                        │
                      │  ┌─────────────┐              │
                      │  │   SUCCESS   │──────────────┘
                      │  └─────────────┘
                      │
                      └───────────────► Continue

       Any hardware / timeout failure
                      │
                      ▼
                ┌─────────────┐
                │ STATE_ERROR  │
                │ Motors STOP  │
                └─────────────┘
```

The firmware explicitly represents `MENU_MAIN`, `STATE_SEARCH_RUN`, `STATE_FAST_RUN`, `STATE_DIAGNOSTICS`, `STATE_CALIBRATION`, and `STATE_ERROR`.

---

# 7. Maze Representation

A particularly important design choice is the use of a **compact per-cell representation**.

Each maze cell is represented by an 8-bit value:

```text
        uint8_t cells[x][y]

        ┌───────┬───────┬───────┬───────┬───────┐
        │ bit 7 │ bit 6 │ bit 5 │ bit 4 │ ...   │
        ├───────┴───────┴───────┴───────┴───────┤
        │       WALL / VISITED FLAGS             │
        └────────────────────────────────────────┘

        WALL_N
        WALL_E
        WALL_S
        WALL_W
        VISITED
```

The implementation uses bit masks rather than allocating a heavyweight object for every cell.

This is a deliberate embedded-systems optimization:

```text
Cell representation
        │
        ├── Wall North
        ├── Wall East
        ├── Wall South
        ├── Wall West
        └── Visited
```

The maze solver also maintains a separate distance field:

```text
uint8_t dist[x][y]
```

where `255` represents an uninitialized/unreachable distance.

The result is a compact world model that is inexpensive to reset and recompute.

---

# 8. Online Wall Mapping

At each search position the robot reads:

```text
IR Sensor Observation
        │
        ├── Front wall?
        ├── Right wall?
        └── Left wall?
                │
                ▼
        Transform relative
        direction → global direction
                │
                ▼
        Update maze model
                │
                ▼
        Mark cell visited
```

The implementation also maintains **wall symmetry**.

For example, if an east wall is discovered at `(x, y)`, the corresponding west wall of `(x+1, y)` is updated.

Conceptually:

[
W_E(x,y) = W_W(x+1,y)
]

and similarly:

[
W_N(x,y) = W_S(x,y+1)
]

This prevents the map from representing contradictory adjacency relationships.

---

# 9. Dynamic Flood-Fill Path Planning

The navigation algorithm uses **Breadth-First Search (BFS)** to construct a dynamic flood-fill distance field.

The goal is the four central cells:

```text
(7,8) ┌───────┬───────┐
      │ GOAL  │ GOAL  │
      ├───────┼───────┤
(7,7) │ GOAL  │ GOAL  │
      └───────┴───────┘
          Center region
```

The solver initializes goal cells with:

[
d(g)=0
]

and propagates outward:

[
d(v)=d(u)+1
]

for every reachable neighboring cell `v`.

The resulting grid represents the minimum number of cell transitions required to reach the goal under the currently known wall topology.

### Why recompute after sensing?

The maze is initially unknown.

Therefore the planner cannot assume that the first path selected remains valid after new walls are discovered.

The search loop follows:

```text
Sense
  ↓
Update map
  ↓
Recompute flood-fill
  ↓
Select lowest-cost legal neighbor
  ↓
Move
  ↓
Repeat
```

This turns the robot into an **online replanning system** rather than a static path-following robot.

---

# 10. Motion Command Generation

The path planner deliberately does not directly command PWM values.

Instead:

```text
Flood-Fill Planner
       │
       ▼
Best Neighbor Direction
       │
       ├── Same heading ─────► CMD_FORWARD_1
       │
       ├── +90° ──────────────► CMD_TURN_RIGHT
       │
       ├── -90° ──────────────► CMD_TURN_LEFT
       │
       └── 180° ──────────────► CMD_TURN_180
```

Neighbor evaluation is ordered relative to the robot's current heading:

```text
1. Forward
2. Right
3. Left
4. Reverse
```

This provides a deterministic tie-breaking policy when multiple neighbors have equal flood-fill distance.

That seemingly small design decision matters because it prevents arbitrary behavior when the planner encounters multiple equally optimal branches.

---

# 11. Differential-Drive Kinematics

For a differential-drive robot with wheel velocities (v_L) and (v_R), the body velocity can be approximated as:

[
v = \frac{v_R+v_L}{2}
]

and angular velocity as:

[
\omega = \frac{v_R-v_L}{L}
]

where (L) is the effective wheel separation.

The firmware exploits this directly.

For straight-line motion:

```text
Left PWM   = Base Speed - Correction
Right PWM  = Base Speed + Correction
```

For rotation:

```text
Left PWM   = +u
Right PWM  = -u
```

This creates a clean separation between translational and rotational control modes.

---

# 12. Encoder Feedback and Interrupts

Wheel encoders are handled using hardware interrupts:

```cpp
void IRAM_ATTR isrLeftEncoder()
{
    if (digitalRead(PIN_ENC_L_B) == HIGH)
        leftTicks++;
    else
        leftTicks--;
}
```

and equivalently for the right encoder.

The encoder state is therefore updated asynchronously relative to the high-level navigation loop.

This is an important real-time design decision:

> **High-frequency sensor events are captured through ISRs rather than relying on polling inside the navigation loop.**

The main control layer snapshots the volatile tick counters with interrupts temporarily disabled:

```text
ISR
 │
 ├── Increment / decrement tick counter
 │
 ▼
volatile state
 │
 ▼
Atomic snapshot
 │
 ▼
Motion controller
```

This prevents the navigation code from reading a partially updated multi-byte counter.

---

# 13. Translational Motion Control

For a commanded cell distance (D), the firmware converts distance into encoder ticks:

[
N_{target} = \frac{D}{\text{MM_PER_TICK}}
]

The average wheel displacement is then estimated as:

[
N_{avg} = \frac{N_L+N_R}{2}
]

and the remaining motion is:

[
e_N=N_{target}-N_{avg}
]

The controller reduces speed near the endpoint:

```text
High remaining distance
        │
        ▼
   High velocity
        │
        ▼
Near target
        │
        ▼
   Velocity taper
        │
        ▼
     Stop
```

This reduces overshoot and makes cell-to-cell motion more repeatable.

---

# 14. Wall-Relative Steering

The robot does not rely exclusively on encoder odometry while traversing a corridor.

When both side walls are available, the controller uses differential IR measurements:

[
e_{wall}=IR_L-IR_R
]

with a proportional/derivative correction:

[
u_{wall}=K_p e_{wall}+K_d(e_{wall}-e_{wall,prev})
]

The resulting correction is injected into the differential-drive command:

[
PWM_L = V-u_{wall}
]

[
PWM_R = V+u_{wall}
]

When only one wall is visible, the controller instead compares that sensor against its calibrated wall threshold.

This creates a useful hierarchy:

```text
Both walls available
        │
        ▼
Left-vs-right geometric alignment
        │
        │
Only left wall ─────► Left wall error
        │
Only right wall ────► Right wall error
        │
No side wall
        │
        ▼
Gyro heading feedback
```

The system therefore degrades gracefully as wall observations change.

---

# 15. Heading Estimation

The MPU6050 gyro is used to estimate yaw.

After a startup calibration procedure, the firmware estimates angular velocity:

[
\omega_z =
\frac{rawG_z-offset}{131}
]

and integrates it:

[
\theta_k =
\theta_{k-1}+\omega_z\Delta t
]

A deadband is applied to small angular rates to suppress insignificant noise:

```text
|gyro rate| <= threshold
        │
        ▼
    ignore update
```

This is a lightweight form of state estimation suitable for a constrained platform.

It is intentionally not described as a full inertial navigation solution: the current implementation primarily uses **single-axis gyro integration for heading control**, not a fused IMU pose estimator.

---

# 16. Turning Control

For a target turn angle (\theta^*):

[
e_\theta=\theta^*-\theta
]

The controller uses proportional and derivative terms:

[
u =
K_{p,turn}e_\theta+
K_{d,turn}\frac{\Delta e_\theta}{\Delta t}
]

The implementation then applies a bounded actuator command and a minimum effective turn speed to overcome motor dead-zone/friction.

The control structure is therefore:

```text
Target Angle
     │
     ▼
 Angle Error
     │
     ├───────────────┐
     ▼               ▼
 Proportional     Derivative
     │               │
     └───────┬───────┘
             ▼
       Control Output
             │
       Saturation /
       Minimum PWM
             │
             ▼
      Differential Drive
```

---

# 17. Control Parameters

The firmware persists controller parameters using ESP32 non-volatile storage.

Current defaults include:

| Parameter      | Default |
| -------------- | ------: |
| Drive (K_p)    |   `2.5` |
| Drive (K_d)    |   `1.2` |
| Turn (K_p)     |   `2.8` |
| Turn (K_d)     |   `1.5` |
| Search speed   |   `150` |
| Fast-run speed |   `230` |

These are not treated as universal constants. They are configuration parameters exposed to calibration and should ultimately be tuned against:

* wheel diameter
* wheelbase
* motor torque
* gearbox backlash
* battery voltage
* sensor geometry
* floor friction
* encoder resolution

The important engineering decision is to keep these parameters **outside the navigation algorithm**, making controller tuning independent from the planner.

---

# 18. Search Run vs. Fast Run

The firmware separates exploration from execution.

## Search Run

```text
Unknown Maze
     │
     ▼
Sense walls
     │
     ▼
Update map
     │
     ▼
Flood-fill
     │
     ▼
Move
     │
     └──────────────► Repeat
```

The objective is to discover enough maze topology to establish a route to the center.

## Fast Run

After successful search:

```text
Known Maze
    │
    ▼
Flood-fill using stored map
    │
    ▼
Select optimal neighbor
    │
    ▼
Execute at higher speed
```

This is a useful systems-level separation: **exploration and exploitation are distinct operating modes**.

---

# 19. Engineering Trade-offs & Challenges

## 19.1 Sensor Noise vs. Computational Simplicity

IR sensors are inexpensive and computationally lightweight, but raw readings can vary with:

* wall material
* incidence angle
* ambient optical conditions
* robot lateral position
* sensor-to-wall distance

Instead of deploying an expensive perception stack, the current architecture uses thresholded wall detection combined with **relative left/right error signals**.

This is a deliberate engineering trade-off:

```text
More sophisticated perception
        ↑
        │   computational cost
        │
        │
        └───────────────►
```

For a constrained Micromouse platform, the objective is not maximum perception sophistication; it is sufficient observability at sufficiently low latency.

---

## 19.2 Motor Mismatch, Drift, and Backlash

Real DC motors are not identical.

Even with identical PWM:

[
v_L \neq v_R
]

because of manufacturing variation, friction, gearbox characteristics, wheel diameter mismatch, and load distribution.

The system mitigates this using multiple feedback mechanisms:

1. Encoder-based distance measurement
2. IR-based wall alignment
3. Gyroscope-based heading feedback
4. Separate turning and driving controller parameters
5. Reduced speed near target distance

The key design principle is:

> **Do not assume actuator symmetry; measure the resulting motion and correct it.**

---

## 19.3 Odometry Drift

Encoder odometry is inherently cumulative.

If each cell introduces an average displacement error (\epsilon), then over (n) cells the uncorrected error can grow approximately as:

[
E(n)\approx n\epsilon
]

The robot therefore avoids treating encoder position as an absolute truth.

The maze coordinate `(posX, posY)` is updated only after successful discrete cell motion, while physical motion is continuously regulated through encoder, IR, and gyro feedback.

This creates a useful distinction:

```text
Physical state
     │
     ├── Encoder ticks
     ├── Gyro yaw
     └── IR geometry
     
Logical state
     │
     ├── Maze cell
     └── Heading
```

---

## 19.4 Memory Constraints

Embedded systems reward predictable memory usage.

The solver uses statically allocated structures:

```cpp
uint8_t cells[MAZE_SIZE][MAZE_SIZE];
uint8_t dist[MAZE_SIZE][MAZE_SIZE];
Point queue[MAX_QUEUE_SIZE];
```

There is no dynamic allocation inside the core flood-fill algorithm.

Advantages include:

* predictable memory consumption
* reduced heap fragmentation risk
* deterministic allocation behavior
* easier reasoning about worst-case storage

The byte-per-cell representation also avoids unnecessary object overhead.

---

## 19.5 Queue Design

Flood-fill uses a statically allocated BFS queue with head/tail indices.

```text
┌───────────────────────────────────┐
│ Point queue[MAX_QUEUE_SIZE]       │
├───────┬───────────────────────────┤
│ Head  │ processed elements        │
├───────┼───────────────────────────┤
│ Tail  │ newly inserted elements   │
└───────┴───────────────────────────┘
```

The implementation bounds insertion through `MAX_QUEUE_SIZE`, avoiding uncontrolled memory growth.

For a production competition firmware revision, this could be further improved into a circular queue with compile-time verification that the queue capacity is sufficient for the maximum reachable grid.

---

# 20. Deterministic Response-Time Considerations

The system is not a hard real-time operating system, and it would be inaccurate to describe every firmware path as hard real-time.

Instead, the implementation establishes **bounded control operations**.

Examples include:

* encoder acquisition through interrupts
* fixed-size maze arrays
* bounded BFS traversal
* bounded motion-command loops
* motion timeouts
* explicit motor-stop behavior on timeout
* no dynamic allocation in the core navigation algorithm

Motion operations contain explicit watchdog-like time limits:

```text
Drive operation
     │
     ├── Target reached → SUCCESS
     │
     └── Timeout → STOP MOTORS → FAILURE
```

This is particularly important in robotics: a controller that fails to reach its target must not continue driving indefinitely.

---

# 21. Fault Handling

The firmware explicitly checks several failure conditions.

### Battery protection

```text
Battery voltage
      │
      ▼
Below threshold?
   │          │
  NO         YES
   │          │
   ▼          ▼
Continue   Stop motors
              │
              ▼
        STATE_ERROR
```

### Hardware initialization

The MPU6050 is initialized and calibrated during startup. Failure transitions the system into `STATE_ERROR`.

### Motion timeout

Both driving and turning operations have bounded execution windows. Failure causes the motors to stop and returns failure to the higher-level state machine.

This gives the system a basic **fail-safe actuator policy**:

> If confidence in successful motion execution is lost, stop rather than continue blindly.

---

# 22. Persistent Configuration

Controller gains and speed parameters are persisted through ESP32 NVS using the `Preferences` API.

The firmware stores:

```text
kp_drive
kd_drive

kp_turn
kd_turn

search_speed
fast_speed
```

This turns calibration into a reproducible configuration problem rather than requiring firmware source modification for every tuning iteration.

A future version could extend this mechanism to include:

* encoder calibration
* wheel circumference
* wheelbase
* IR sensor thresholds
* gyro bias
* battery calibration factor

---

# 23. User Interface and Diagnostics

The robot includes an SSD1306 OLED dashboard exposing runtime state such as:

```text
POS: (x,y) H:heading
ACT: current action
IR L:... F:... R:...
YAW: ... deg
BAT: ... V
```

This is more than a cosmetic interface.

For embedded robotics development, observability is essential. The dashboard provides a low-cost debugging channel for correlating:

```text
Sensor state
      +
Estimated state
      +
Controller action
      +
Power state
```

The firmware also includes dedicated diagnostics and calibration modes.

---

# 24. Complexity Analysis

For an (N\times N) maze, flood-fill visits each reachable cell at most once.

Therefore:

[
T_{BFS}=O(N^2)
]

and the maze distance field requires:

[
S_{distance}=O(N^2)
]

The wall representation also scales as:

[
S_{walls}=O(N^2)
]

For the fixed embedded maze size, these bounds are small enough to support complete replanning after each newly observed wall configuration.

The important point is that the system trades additional computation for **planning correctness under changing world knowledge**.

---

# 25. Why Flood-Fill Instead of A*?

A* would be a valid alternative.

For this application, flood-fill has several attractive properties:

* uniform grid topology
* unit-cost cell transitions
* simple implementation
* predictable memory usage
* easy multi-goal initialization
* natural compatibility with dynamic maze updates

With four goal cells initialized simultaneously, the algorithm effectively computes distance to the nearest goal cell.

For a constrained embedded robot, this simplicity is valuable.

A future implementation could compare:

[
\text{Flood-Fill}
\quad vs. \quad
A^*
\quad vs. \quad
D^*
]

under identical maze-update workloads and measure:

* execution time
* memory footprint
* path optimality
* replanning frequency

---

# 26. Current Architecture: Strengths and Boundaries

### Strengths

* Hardware/software co-design
* Closed-loop motion rather than open-loop PWM
* Interrupt-driven encoder acquisition
* Gyro-based heading feedback
* Wall-relative steering
* Online maze reconstruction
* Dynamic flood-fill replanning
* Fixed-memory maze representation
* Persistent controller configuration
* Explicit motion timeouts
* Separate exploration and fast-run modes
* Diagnostic and calibration operating states

### Current boundaries

The implementation is intentionally lightweight and does not currently implement:

* full sensor fusion such as EKF/UKF
* SLAM
* continuous global pose estimation
* velocity estimation from filtered encoder derivatives
* acceleration/jerk-limited trajectory generation
* model-predictive control
* full-speed path compression into long straight segments
* dynamic obstacle tracking
* formal WCET analysis

These are not shortcomings of the system architecture so much as clear directions for future research and engineering refinement.

---

# 27. Potential Research Extensions

The existing architecture provides a strong base for progressively more sophisticated autonomy.

## State Estimation

Replace independent gyro integration and encoder estimates with a fused estimator:

[
\hat{x}_{k+1}=f(\hat{x}_k,u_k)+w_k
]

with measurements:

[
z_k=h(x_k)+v_k
]

Potential implementation:

* complementary filter
* extended Kalman filter
* invariant EKF

---

## Adaptive Control

Motor behavior changes with battery voltage and mechanical load.

A future controller could adapt gains as a function of:

[
K_p=f(V_{battery},v,\text{surface})
]

rather than relying on a fixed parameter set.

---

## Trajectory-Level Fast Run

The current planner produces discrete cell-level commands:

```text
FORWARD
TURN
FORWARD
TURN
...
```

A more advanced planner could compress consecutive forward cells:

```text
F F F F R F F L F
        ↓
4-cell segment
        ↓
R
        ↓
2-cell segment
        ↓
L
```

This would reduce turning and improve competition performance.

---

## More Robust Wall Estimation

Instead of binary wall thresholds:

[
wall\in{0,1}
]

the system could maintain a confidence estimate:

[
P(wall\mid sensor)
]

and fuse repeated measurements over time.

That would make the world model more robust to ambiguous sensor readings.

---

# 28. Engineering Philosophy

The core design philosophy of Micromouse-bot is:

```text
             MEASURE
                │
                ▼
             ESTIMATE
                │
                ▼
              MODEL
                │
                ▼
              PLAN
                │
                ▼
             CONTROL
                │
                ▼
             ACTUATE
                │
                ▼
             MEASURE
                │
                └───────────────► repeat
```

This is the central idea behind autonomous robotics.

The robot is not merely executing a maze-solving algorithm. It is repeatedly closing the loop between:

**physical reality → sensing → state representation → planning → control → physical reality.**

---

# 29. Repository Structure

The firmware is designed around a modular embedded architecture:

```text
Micromouse-bot/
│
├── Micromouse_final.ino
│
├── config.h
├── motors.h
├── sensors.h
├── pid.h
│
├── motors.cpp
├── sensors.cpp
├── pid.cpp
│
└── README.md
```

The principal responsibilities are separated as:

| Module                 | Responsibility                                    |
| ---------------------- | ------------------------------------------------- |
| `Micromouse_final.ino` | System integration, FSM, maze solver, navigation  |
| `config.*`             | Hardware constants and configuration              |
| `motors.*`             | Motor driver abstraction                          |
| `sensors.*`            | IR sensing abstraction                            |
| `pid.*`                | Controller-related abstractions                   |
| `README.md`            | System architecture and engineering documentation |

The exact contents of auxiliary modules may evolve independently without changing the high-level navigation architecture.

---

# 30. Build and Deployment

The firmware targets an Arduino-compatible ESP32 environment.

A typical development workflow is:

```text
Clone repository
      │
      ▼
Configure board / pins
      │
      ▼
Compile firmware
      │
      ▼
Flash ESP32
      │
      ▼
Run calibration
      │
      ▼
Tune controller parameters
      │
      ▼
Search Run
      │
      ▼
Fast Run
```

The hardware-specific constants should be verified against the actual robot PCB/wiring before deployment.

---

# 31. Validation Strategy

A rigorous validation process should separate subsystem validation from full-system validation.

## Level 1 — Sensor Validation

Measure:

* IR threshold repeatability
* wall-distance response
* gyro bias
* encoder direction correctness
* encoder counts per wheel revolution

## Level 2 — Actuator Validation

Measure:

* minimum motor start PWM
* left/right velocity mismatch
* braking distance
* turning response
* battery-voltage sensitivity

## Level 3 — Controller Validation

Evaluate:

[
e(t)=r(t)-y(t)
]

for:

* straight-line displacement
* 90° turns
* 180° turns
* wall alignment

Relevant metrics:

* settling time
* overshoot
* steady-state error
* repeatability

## Level 4 — Navigation Validation

Evaluate:

* wall-map correctness
* flood-fill correctness
* replanning behavior
* dead-end recovery
* goal acquisition

## Level 5 — Full-System Validation

Run multiple trials across:

* different maze topologies
* different battery states
* different starting conditions
* repeated runs

The objective should be **statistical repeatability**, not a single successful run.

---

# 32. Admissions-Level Technical Contribution

From a graduate research admissions perspective, the most significant aspect of this project is not the use of a particular microcontroller or maze algorithm.

Its value lies in demonstrating an ability to reason across abstraction layers.

The project connects:

```text
Electrical / Hardware
        │
        ▼
Embedded Firmware
        │
        ▼
Real-Time Control
        │
        ▼
State Estimation
        │
        ▼
Algorithmic Planning
        │
        ▼
Autonomous Behavior
```

That cross-layer reasoning is directly relevant to graduate work in:

* Robotics
* Computer Science
* Electrical & Computer Engineering
* Embedded Systems
* Autonomous Systems
* Controls
* Cyber-Physical Systems

The system demonstrates several characteristics expected in research-oriented engineering:

### 1. Modular abstraction

The planner does not need to know PWM implementation details.

### 2. Resource awareness

The maze model uses compact fixed-size structures instead of unconstrained dynamic allocation.

### 3. Closed-loop thinking

Commands are continuously checked against measured physical behavior.

### 4. Explicit failure handling

Timeouts, battery limits, initialization failures, and sensor failures are treated as system states.

### 5. Algorithm-hardware compatibility

Flood-fill was selected not only because it works mathematically, but because its computational characteristics fit the embedded platform.

### 6. Iterative architecture

The current design provides natural extension points for state fusion, adaptive control, trajectory optimization, and more sophisticated planning.

---

# 33. Intellectual Contribution

The project's intellectual contribution can be summarized as:

> **Designing a compact autonomous navigation stack in which online world-model construction, graph-based planning, sensor feedback, and actuator control remain computationally tractable on a resource-constrained embedded platform.**

The important engineering insight is that autonomous behavior emerges from the interaction of relatively simple components:

[
\boxed{
Sensing
\rightarrow
State
\rightarrow
Map
\rightarrow
Planning
\rightarrow
Control
\rightarrow
Motion
}
]

Each component has uncertainty and failure modes. The system becomes robust not because any individual component is perfect, but because the architecture provides multiple feedback paths and explicit failure boundaries.

---

# 34. Research Questions Enabled by This Platform

Micromouse-bot can serve as a compact experimental platform for questions such as:

1. How does sensor fusion affect cell-to-cell localization error?
2. How much does wall-relative steering reduce accumulated odometry error?
3. What is the computational cost of flood-fill replanning versus A*?
4. How should controller gains adapt to battery voltage?
5. What is the optimal trade-off between exploration speed and map certainty?
6. How does path compression affect total traversal time?
7. How can uncertainty-aware wall mapping improve route reliability?
8. What controller architecture minimizes turning overshoot under motor asymmetry?
9. What is the empirical worst-case execution time of the navigation loop?
10. How does model-based control compare with the current feedback strategy?

These questions turn the robot from a one-off embedded project into a **repeatable experimental platform for autonomous systems research**.

---

# 35. Engineering Summary

```text
┌───────────────────────────────────────────────────────────────┐
│                    MICROMOUSE-BOT                             │
├───────────────────────────────────────────────────────────────┤
│                                                               │
│  SENSING                                                      │
│  ├── 3× IR wall sensors                                      │
│  ├── Quadrature encoders                                     │
│  └── MPU6050 gyro                                             │
│                                                               │
│  STATE                                                        │
│  ├── Grid coordinates                                        │
│  ├── Heading                                                  │
│  └── Yaw estimate                                             │
│                                                               │
│  WORLD MODEL                                                   │
│  ├── Bit-packed wall representation                           │
│  ├── Visited state                                            │
│  └── Dynamic distance field                                   │
│                                                               │
│  PLANNING                                                      │
│  └── Multi-goal BFS / Flood-Fill                              │
│                                                               │
│  CONTROL                                                       │
│  ├── Encoder distance feedback                                │
│  ├── IR wall alignment                                       │
│  ├── Gyro heading feedback                                   │
│  └── PD-style turning control                                │
│                                                               │
│  SAFETY                                                       │
│  ├── Motion timeouts                                          │
│  ├── Battery monitoring                                       │
│  ├── Initialization checks                                    │
│  └── Explicit motor-stop failure behavior                     │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

---

# 36. Final Perspective

Micromouse-bot is intentionally positioned at the intersection of **embedded systems, control engineering, algorithms, and autonomous robotics**.

The project demonstrates that even on a small microcontroller, a robot can maintain a structured internal representation of an unknown environment, update that representation from imperfect sensors, perform online graph search, and execute the resulting policy through closed-loop physical control.

The deeper engineering lesson is:

> **Autonomy is not a single algorithm. It is the disciplined integration of sensing, estimation, modeling, planning, control, computation, and failure handling.**

That integration is the primary engineering contribution of this project.

---

## Technology Stack

`ESP32` · `Embedded C++` · `Arduino Framework` · `IR Sensors` · `Quadrature Encoders` · `MPU6050` · `Differential Drive` · `Closed-Loop Control` · `Breadth-First Search` · `Flood-Fill` · `Finite State Machine` · `I²C` · `NVS`

---

## Keywords

`Robotics` `Autonomous Systems` `Micromouse` `Embedded Systems` `Real-Time Control` `Motion Control` `Differential Drive` `Sensor Fusion` `State Estimation` `Path Planning` `Flood Fill` `BFS` `ESP32` `C++` `Computer Engineering` `Control Systems` `Cyber-Physical Systems`
