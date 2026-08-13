<div align="center">

# MICROMOUSE-BOT

### Autonomous Maze Navigation as an Embedded Systems & Computer Science Problem

`ESP32` · `Embedded C++` · `BFS / Flood-Fill` · `Grid Graphs` · `Interrupts` · `IR Perception` · `Encoder Feedback` · `MPU6050` · `PD Control`

<br>

[![Platform](https://img.shields.io/badge/Platform-ESP32-000000?style=for-the-badge\&logo=espressif)](https://www.espressif.com/)
[![Language](https://img.shields.io/badge/Language-Embedded%20C%2B%2B-00599C?style=for-the-badge\&logo=cplusplus)](https://isocpp.org/)
[![Framework](https://img.shields.io/badge/Framework-Arduino-00979D?style=for-the-badge\&logo=arduino)](https://www.arduino.cc/)
[![Algorithm](https://img.shields.io/badge/Algorithm-BFS%20%2F%20Flood--Fill-6A5ACD?style=for-the-badge)](#-navigation-as-a-graph-problem)
[![Control](https://img.shields.io/badge/Control-Feedback%20%2B%20PD-8A2BE2?style=for-the-badge)](#-motion-control)

<br>

**A resource-constrained autonomous robot that converts partial sensor observations into a discrete world model, performs online graph search, and translates algorithmic decisions into measured physical motion.**

</div>

---

## 01 — Project Thesis

Micromouse-bot is a study in **closed-loop autonomous computation**.

The robot does not begin with a complete representation of the maze. Instead, it repeatedly:

```text
SENSE
  ↓
INTERPRET
  ↓
UPDATE WORLD MODEL
  ↓
RECOMPUTE DISTANCE FIELD
  ↓
SELECT NEXT GRAPH TRANSITION
  ↓
EXECUTE MOTION
  ↓
MEASURE PHYSICAL RESPONSE
  ↓
REPEAT
```

The interesting problem is therefore not simply *"solve a maze."*

It is the interaction between:

* **data structures** — compact grid and wall representation,
* **graph algorithms** — breadth-first flood-fill,
* **real-time acquisition** — interrupt-driven encoder counting,
* **state representation** — position + heading + world model,
* **feedback control** — encoder, IR and gyroscope measurements,
* **resource constraints** — fixed-size arrays and bounded queues,
* **systems engineering** — hardware failures, timeouts, calibration and battery protection.

The implementation reflects this systems view directly: the firmware contains a `MazeSolver` class with explicit cell, distance-field and queue structures, while motion primitives operate against real sensor feedback.

---

# 02 — System at a Glance

| Layer                   | Implementation                                       |
| ----------------------- | ---------------------------------------------------- |
| **Compute**             | ESP32 running Arduino-based C++                      |
| **Perception**          | Three IR wall observations: left / front / right     |
| **Motion feedback**     | Left + right wheel encoder tick counters             |
| **Heading feedback**    | MPU6050 Z-axis gyroscope                             |
| **World model**         | Fixed-size 2D maze grid                              |
| **Cell representation** | Directional wall bit flags + visited flag            |
| **Planner**             | BFS-based flood-fill distance field                  |
| **Goal**                | Four central cells of the maze                       |
| **Motion abstraction**  | Forward, left 90°, right 90°, 180°                   |
| **Control**             | IR steering + yaw-based correction + PD turn control |
| **Real-time input**     | GPIO encoder interrupts                              |
| **Persistence**         | ESP32 `Preferences` / NVS                            |
| **Human interface**     | SSD1306 OLED + buttons + buzzer                      |
| **Safety**              | Battery monitoring + motion timeouts + error state   |

## The ESP32-specific firmware uses `Preferences`, interrupt attributes, GPIO interrupts, I²C, and the SSD1306 display stack.

# 03 — Architecture

```text
                         ┌───────────────────────────────┐
                         │            ESP32              │
                         │                               │
                         │       Application FSM        │
                         └───────────────┬───────────────┘
                                         │
             ┌───────────────────────────┼──────────────────────────┐
             │                           │                          │
             ▼                           ▼                          ▼
      ┌───────────────┐         ┌────────────────┐         ┌────────────────┐
      │  PERCEPTION   │         │   WORLD MODEL  │         │    CONTROL     │
      │               │         │                │         │                │
      │ IR L/F/R      │         │ Maze cells     │         │ Drive control  │
      │ Encoders      │         │ Wall flags     │         │ Wall steering  │
      │ MPU6050       │         │ Visited state  │         │ Gyro turns     │
      └───────┬───────┘         │ Distance field │         │ PWM output     │
              │                 │ BFS queue      │         └───────┬────────┘
              │                 └───────┬────────┘                 │
              │                         │                          │
              └────────────────────────►│◄─────────────────────────┘
                                        │
                                        ▼
                               ┌──────────────────┐
                               │  MOTION COMMAND  │
                               │                  │
                               │ FWD / L / R /180│
                               └────────┬─────────┘
                                        │
                                        ▼
                               ┌──────────────────┐
                               │ MOTOR INTERFACE  │
                               └────────┬─────────┘
                                        │
                                  ┌─────┴─────┐
                                  ▼           ▼
                             LEFT MOTOR   RIGHT MOTOR
                                  │           │
                                  └─────┬─────┘
                                        │
                                        ▼
                                      ROBOT
                                        │
                    ┌───────────────────┴───────────────────┐
                    │                                       │
                    ▼                                       ▼
              Wheel Encoders                          IR + MPU6050
                    │                                       │
                    └─────────────── FEEDBACK ──────────────┘
```

The code implements the navigation, sensing, motion and application-state layers in one main `.ino` translation unit while delegating hardware interfaces to included modules such as `config.h`, `motors.h`, `sensors.h`, and `pid.h`.

---

# 04 — Hardware Architecture

```text
                         FRONT WALL
                  ─────────────────────

                           ▲
                           │
                      ┌─────────┐
                      │ FRONT IR│
                      └─────────┘

             ┌─────────────┐   ┌─────────────┐
             │   LEFT IR   │   │   RIGHT IR  │
             └──────┬──────┘   └──────┬──────┘
                    │                 │
                    │   ┌─────────┐   │
                    └──►│  ESP32  │◄──┘
                        │         │
                        │ MPU6050 │
                        │   I²C   │
                        └────┬────┘
                             │
                  ┌──────────┴──────────┐
                  │                     │
             ┌────▼─────┐         ┌─────▼────┐
             │ LEFT     │         │ RIGHT    │
             │ MOTOR    │         │ MOTOR    │
             │ + ENCODER│         │ + ENCODER│
             └──────────┘         └──────────┘

                         REAR
```

The firmware explicitly configures:

* left/right encoder A/B inputs,
* encoder interrupts,
* IR sensor initialization,
* user buttons,
* buzzer,
* I²C on GPIO 21/22,
* I²C clock at 400 kHz,
* SSD1306 OLED at address `0x3C`.

The exact motor model and motor-driver part number are **not specified in the supplied `.ino` file**, so they are intentionally not claimed here.

---

# 05 — Software Structure

```text
Micromouse-bot
│
├── Micromouse_final(2).ino
│   │
│   ├── MazeSolver
│   ├── Encoder ISRs
│   ├── Hardware initialization
│   ├── MPU6050 integration
│   ├── Wall mapping
│   ├── Motion control
│   ├── Search Run
│   ├── Fast Run
│   ├── Diagnostics
│   ├── Calibration
│   ├── Menu / OLED interface
│   └── Application FSM
│
├── config.h
├── motors.h
├── sensors.h
├── pid.h
└── README.md
```

### Architectural honesty

The repository has meaningful subsystem boundaries, but the supplied implementation is **not a fully decomposed object-oriented embedded framework**.

The main firmware unit currently integrates the higher-level application logic, navigation, motion primitives, state transitions and sensor orchestration.

That is preferable to pretending the project has a software architecture that the source does not actually contain.

The natural next architectural step would be to separate:

```text
Navigation
    │
    ├── MazeModel
    ├── FloodFillPlanner
    └── MotionPlanner

Control
    │
    ├── DriveController
    └── TurnController

Perception
    │
    ├── IRSensorModel
    ├── EncoderInterface
    └── IMUInterface

Platform
    │
    ├── Motors
    ├── Display
    ├── Storage
    └── Safety
```

This would make the system easier to test independently and would create cleaner interfaces between algorithms and hardware.

---

# 06 — The Maze as a Graph

The maze is represented as a **grid graph**.

Each cell corresponds to a graph vertex:

[
G=(V,E)
]

where:

* (V) = reachable maze cells,
* (E) = physically traversable adjacency relationships.

A cell can have up to four neighbors:

```text
                    NORTH
                      │
                      ▼
                 ┌─────────┐
                 │         │
          WEST ◄─┤   CELL  ├─► EAST
                 │         │
                 └─────────┘
                      ▲
                      │
                    SOUTH
```

The implementation stores each cell as a compact byte and encodes directional walls using bit flags. The `VISITED` state is also stored in the cell representation.
This creates a direct mapping between:

```text
Physical maze
     ↓
Cell
     ↓
Wall constraints
     ↓
Graph adjacency
```

A wall does not merely represent sensor information; it changes which graph transitions are legal.

---

# 07 — Consistent World-Model Updates

A particularly important implementation detail is that a discovered wall is written to **both adjacent cells**.

For example:

```text
        Cell A                 Cell B

     ┌──────────┐           ┌──────────┐
     │          │           │          │
     │          │     │     │          │
     │          │     │     │          │
     └──────────┘     │     └──────────┘
                      │
                   shared wall
```

When an east wall is assigned to `(x,y)`, the corresponding west wall is assigned to `(x+1,y)`.

This maintains the invariant:

[
W_E(x,y)=W_W(x+1,y)
]

The same relationship is implemented for all four directions.

This is a small implementation decision with an important algorithmic consequence: the physical maze representation remains consistent with the adjacency graph used by the planner.

---

# 08 — Navigation as a Graph Algorithm

## Multi-source BFS / Flood-Fill

The target is the central 2×2 region:

```text
┌───────┬───────┐
│       │       │
│  G    │   G   │
│       │       │
├───────┼───────┤
│       │       │
│  G    │   G   │
│       │       │
└───────┴───────┘
```

All four goal cells are initialized with:

[
d(g)=0
]

The queue then propagates distances through legal graph edges:

[
d(v)=d(u)+1
]

The source implements this directly using:

```text
distance field
      +
fixed-size queue
      +
four-neighbor expansion
      =
BFS flood-fill
```

The implementation initializes the distance array to `255`, inserts the goal cells into a fixed queue, and expands north/east/south/west neighbors only when a wall does not block the transition.

---

# 09 — Why BFS?

For an unweighted grid in which moving between adjacent cells has equal graph cost, BFS gives minimum edge-count distance.

For:

[
N = |V|
]

the traversal is:

[
O(|V|+|E|)
]

and because a four-connected grid has:

[
|E|=O(|V|)
]

the practical complexity is:

[
\boxed{O(N)}
]

for each flood-fill computation.

For a fixed (M\times M) maze:

[
N=M^2
]

therefore:

[
\boxed{O(M^2)}
]

The important embedded-systems decision is not merely that BFS is correct.

It is that BFS can be implemented with:

* fixed-size arrays,
* a bounded queue,
* integer distance values,
* no dynamic graph allocation,
* predictable memory behavior.

The implementation's `MazeSolver` explicitly allocates `cells`, `dist`, and `queue` as fixed-size members.

---

# 10 — Online Replanning

The robot does not calculate one permanent route and blindly execute it.

During Search Run:

```text
                 START
                   │
                   ▼
            ┌──────────────┐
            │ Sense walls  │
            └──────┬───────┘
                   ▼
            ┌──────────────┐
            │ Update map   │
            └──────┬───────┘
                   ▼
            ┌──────────────┐
            │ Flood-fill   │
            └──────┬───────┘
                   ▼
            ┌──────────────┐
            │ Choose best  │
            │ neighbor     │
            └──────┬───────┘
                   ▼
            ┌──────────────┐
            │ Execute      │
            │ motion       │
            └──────┬───────┘
                   │
                   └──────────────► repeat
```

The search routine recalculates the flood-fill field before motion selection and again after new wall observations.

This makes the planner an **online graph-search system** rather than a static path executor.

---

# 11 — Direction Selection and Tie-Breaking

Once the distance field exists, the planner evaluates candidate directions in this order:

```text
1. Current heading
2. Right
3. Left
4. 180°
```

The neighbor with the smallest distance is selected.

Thus the planner has two separate concerns:

```text
GRAPH LEVEL
"What cell should I enter?"

        ↓

MOTION LEVEL
"How must the robot rotate and translate to enter it?"
```

The source explicitly constructs this direction ordering and converts the resulting direction into one of four motion commands.

This is a useful abstraction boundary: graph search determines the desired discrete transition while the controller is responsible for physically executing that transition.

---

# 12 — Algorithmic State Machine

```text
                         ┌──────────────────┐
                         │      BOOT        │
                         └────────┬─────────┘
                                  │
                                  ▼
                         ┌──────────────────┐
                         │ HARDWARE INIT    │
                         │ IMU + OLED + I/O │
                         └────────┬─────────┘
                                  │
                                  ▼
                         ┌──────────────────┐
                         │    MENU_MAIN     │
                         └────────┬─────────┘
                                  │
             ┌────────────────────┼────────────────────┐
             │                    │                    │
             ▼                    ▼                    ▼
      ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
      │ SEARCH RUN   │    │  FAST RUN    │    │ DIAGNOSTICS  │
      └──────┬───────┘    └──────┬───────┘    └──────────────┘
             │                   │
             ▼                   │
      ┌──────────────┐           │
      │ SENSOR POLL  │           │
      └──────┬───────┘           │
             ▼                   │
      ┌──────────────┐           │
      │ MAZE MAPPING │           │
      └──────┬───────┘           │
             ▼                   │
      ┌──────────────┐           │
      │ TARGET       │           │
      │ RECALCULATION│           │
      └──────┬───────┘           │
             ▼                   │
      ┌──────────────┐           │
      │ MOTION       │───────────┘
      │ EXECUTION    │
      └──────┬───────┘
             │
             ▼
       GOAL REACHED?
          /      \
        NO        YES
        │          │
        └──►───────┘
                   ▼
              RETURN MENU

Any failed motion / hardware fault
                   │
                   ▼
          ┌──────────────────┐
          │   STATE_ERROR    │
          │   STOP MOTORS    │
          └──────────────────┘
```

The actual application state machine contains `MENU_MAIN`, `STATE_SEARCH_RUN`, `STATE_FAST_RUN`, `STATE_DIAGNOSTICS`, `STATE_CALIBRATION`, and `STATE_ERROR`.

---

# 13 — Perception Pipeline

The robot uses three directional IR observations:

```text
               FRONT
                 │
                 ▼
              ┌─────┐
              │ IR F│
              └─────┘
                 │
       ┌─────────┴─────────┐
       ▼                   ▼
   ┌───────┐           ┌───────┐
   │ IR L  │           │ IR R  │
   └───────┘           └───────┘
```

The sensor interface provides both raw values and thresholded wall states:

```text
rawL → wallL
rawF → wallF
rawR → wallR
```

The mapping layer then transforms these observations into directional maze constraints:

```text
IR observation
      ↓
wall classification
      ↓
direction relative to heading
      ↓
setWall(...)
      ↓
graph constraint
```

The `.ino` file delegates the actual sensor implementation to `sensors.h` / the sensor module, so the exact filtering algorithm used internally by that module is not claimed here. The main firmware itself uses thresholded wall states and raw IR values for control.

---

# 14 — Handling Sensor Noise

A physical IR sensor does not directly report:

> `WALL = TRUE`

It produces a measurement influenced by distance, reflectivity, alignment and electronics.

This implementation therefore works with **thresholded wall states** for maze decisions while retaining raw sensor values for feedback and diagnostics.

For two detected side walls, the controller forms:

[
e_{wall}=S_L-S_R
]

and uses the change in this error:

[
\Delta e=e_{wall}-e_{previous}
]

to generate a steering correction.

The implementation is:

[
u=0.05e_{wall}+0.01\Delta e
]

rather than a generic PID formula.

This is an important distinction:

> **The current firmware does not implement a general low-pass filter in the supplied main file. Its demonstrated robustness mechanism is threshold-based wall classification plus feedback from raw sensor differences.**

---

# 15 — Motion Control

The robot executes motion through discrete primitives:

```text
CMD_FORWARD_1
CMD_TURN_RIGHT
CMD_TURN_LEFT
CMD_TURN_180
```

These are mapped to:

```text
driveDistance()
turnRight90()
turnLeft90()
turn180()
```

The planner therefore remains independent from the low-level actuator implementation.

---

## Forward motion

Distance is converted into an encoder target:

[
N_{target}
==========

\frac{d}{MM_PER_TICK}
]

The controller reads both encoder counters and estimates progress using:

[
N_{current}
===========

\frac{N_L+N_R}{2}
]

The source performs this snapshot inside a short interrupt-disabled section.

---

# 16 — Adaptive Endpoint Control

The forward controller reduces commanded speed near the target.

```text
                  TARGET
                    │
                    ▼
        ┌─────────────────────┐
        │ remaining > 300     │
        │ use requested speed │
        └──────────┬──────────┘
                   │
                   ▼
        ┌─────────────────────┐
        │ remaining < 300     │
        │ map speed toward 60 │
        └──────────┬──────────┘
                   │
                   ▼
                 STOP
```

This introduces a simple velocity profile near the endpoint instead of maintaining maximum PWM until the target tick count is crossed.

The implementation maps remaining ticks from `[0,300]` toward `[60,maxSpeed]`.

---

# 17 — Wall-Based Closed-Loop Steering

When both side walls are available:

[
e=S_L-S_R
]

[
u=0.05e+0.01\Delta e
]

The motor commands are then:

[
PWM_L=v-u
]

[
PWM_R=v+u
]

with each command constrained to the valid PWM range.

When only one wall is visible, the controller instead references the corresponding configured wall threshold.

When neither side wall is available, the controller falls back to a yaw-based correction:

[
u=\psi K_{p,drive}
]

This creates three distinct perception regimes:

```text
Both walls
   ↓
Left-right differential


Left wall only
   ↓
Left threshold correction


Right wall only
   ↓
Right threshold correction


No side wall
   ↓
Gyro heading correction
```

This is a practical example of designing the controller around the **availability of information**, rather than assuming an ideal sensor configuration.

---

# 18 — Gyroscope Heading Estimation

The MPU6050 is used as a lightweight yaw sensor.

During initialization, the robot samples the Z-axis gyro 500 times while stationary:

[
b_z=
\frac{1}{500}
\sum_{i=1}^{500}g_i
]

The resulting average is stored as the gyro offset.

During operation:

[
\omega_z
========

\frac{g_z-b_z}{131}
]

and:

[
\psi_{k+1}
==========

\psi_k+\omega_z\Delta t
]

A small angular-rate deadband is applied before integration.

This is intentionally a lightweight estimator.

It is **not** an EKF, complementary filter, or fused inertial navigation system.

The current architecture therefore leaves a clear research direction for future sensor fusion.

---

# 19 — Turn Control

Rotations are closed-loop rather than purely time-based.

The turn controller computes:

[
e=\theta_{target}-\theta_{current}
]

and:

[
u=K_p e+K_d\Delta e
]

where the gains are persistent configuration parameters.

The output is bounded to:

[
-200\leq u\leq200
]

and a minimum magnitude of `60` is enforced for non-zero commands to overcome low-command motor behavior.

A turn succeeds when:

[
|e|<1^\circ
]

and fails after a 1.5-second timeout.

This is **PD control**, not full PID: there is no integral term in the turn controller.

---

# 20 — Interrupt-Driven Encoder Acquisition

Wheel encoder acquisition is asynchronous with respect to the main control flow.

```text
        Encoder A edge
               │
               ▼
       ┌──────────────┐
       │     ISR      │
       └──────┬───────┘
              │
       read Encoder B
              │
              ▼
       determine sign
              │
              ▼
      volatile tick count
              │
              ▼
      short critical section
              │
              ▼
        controller
```

The encoder ISRs are marked `IRAM_ATTR` and increment/decrement `volatile` tick counters based on the second encoder channel.

The main loop takes a coherent snapshot using:

```cpp
noInterrupts();

long l = leftTicks;
long r = rightTicks;

interrupts();
```

This is a useful embedded-systems pattern:

> **capture high-frequency events asynchronously, then consume a stable snapshot in the control loop.**

It should not be described as hard real-time scheduling. The application still contains blocking delays and blocking state routines.

---

# 21 — Computational & Memory Complexity

## Maze storage

The implementation uses:

```cpp
uint8_t cells[MAZE_SIZE][MAZE_SIZE];
uint8_t dist[MAZE_SIZE][MAZE_SIZE];
Point queue[MAX_QUEUE_SIZE];
```

The structures are statically allocated inside `MazeSolver`.

For a maze of (M\times M):

[
N=M^2
]

and the principal grid structures scale as:

[
O(M^2)
]

The BFS queue is separately bounded by `MAX_QUEUE_SIZE`.

### Why this matters

Dynamic graph structures would introduce:

* heap allocation,
* pointer overhead,
* fragmentation concerns,
* less predictable memory usage.

For a fixed-size embedded maze, a compact static representation is a reasonable engineering trade-off.

The algorithm is therefore constrained by the maze dimensions deliberately rather than relying on dynamically growing memory.

---

# 22 — Deterministic vs Hard Real-Time

The firmware has several useful bounded operations:

* fixed-size maze structures,
* bounded BFS queue,
* interrupt-driven encoder capture,
* 5 ms motion-loop delays,
* 3-second forward-motion timeout,
* 1.5-second turn timeout,
* fixed 500-sample IMU calibration.

However, it also contains blocking functions, delays, OLED operations and menu loops.

Therefore:

> **This project should not be described as a hard real-time operating system or formally verified hard-real-time controller.**

A technically accurate characterization is:

**interrupt-assisted embedded control with bounded motion primitives and predictable static data structures.**

That distinction matters in a Computer Science evaluation because real-time claims should follow from timing guarantees, not terminology.

---

# 23 — Engineering Challenges

## Challenge 1 — Noisy / Variable Wall Measurements

**Problem**

IR measurements vary with wall distance, reflectivity and robot alignment.

**Engineering decision**

Use thresholded wall states for topological mapping while retaining raw sensor values for steering and diagnostics.

**Control mechanism**

[
e=S_L-S_R
]

[
u=0.05e+0.01\Delta e
]

**Trade-off**

The controller remains lightweight, but its behavior depends on calibrated sensor geometry and thresholds.

---

## Challenge 2 — Motor Asymmetry

**Problem**

Two nominally identical motors do not necessarily generate identical motion for the same PWM command.

**Engineering decision**

Do not rely exclusively on open-loop PWM.

Use encoder-derived displacement and sensor-based steering corrections.

**Result**

Forward motion is terminated according to measured encoder progress rather than assuming that a fixed PWM duration corresponds exactly to one cell.

---

## Challenge 3 — Turning Accuracy

**Problem**

A timed 90° turn assumes ideal motor behavior and constant battery conditions.

**Engineering decision**

Use the MPU6050 yaw estimate as feedback.

**Controller**

[
u=K_p e+K_d\Delta e
]

**Trade-off**

Gyroscope integration is computationally simple but accumulates drift, making it suitable for short turn primitives rather than long-term absolute localization.

---

## Challenge 4 — High-Frequency Encoder Events

**Problem**

Polling encoders from a slower application loop can miss transitions.

**Engineering decision**

Use GPIO interrupts.

**Trade-off**

Interrupt handlers must remain small and shared data must be accessed safely.

The current ISRs perform only direction detection and tick updates, keeping the interrupt-side operation compact.

---

## Challenge 5 — Motion Failure

**Problem**

A robot can physically fail to reach its commanded state.

**Engineering decision**

Every motion primitive returns a success/failure result.

```text
Motion command
      │
      ▼
   execute
      │
 ┌────┴────┐
 ▼         ▼
SUCCESS   TIMEOUT
 │         │
 ▼         ▼
update    STOP
state     motors
           │
           ▼
       STATE_ERROR
```

The search and fast-run routines transition to `STATE_ERROR` if a motion primitive fails.

---

# 24 — Safety & Failure Handling

Battery voltage is measured through the configured ADC channel.

If the measured voltage falls below the configured minimum, the firmware:

```text
LOW BATTERY
     ↓
STOP MOTORS
     ↓
OLED WARNING
     ↓
BUZZER WARNING
     ↓
HALT
```

The implementation explicitly calls `stopMotors()` before entering the warning loop.

## Hardware initialization can also transition to `STATE_ERROR` if the OLED or MPU6050 initialization fails.

# 25 — Search Run vs Fast Run

The system separates exploration from exploitation.

### Search

```text
Unknown maze
     ↓
Observe walls
     ↓
Build map
     ↓
Flood-fill
     ↓
Move
     ↓
Repeat
```

### Fast Run

```text
Previously discovered maze
          ↓
       Flood-fill
          ↓
    Select transition
          ↓
 Execute using fast speed
```

The firmware refuses to execute Fast Run when no completed maze search exists.

This creates an important algorithmic distinction:

[
\text{Search} \rightarrow \text{Information acquisition}
]

[
\text{Fast Run} \rightarrow \text{Exploitation of acquired information}
]

---

# 26 — Persistent Configuration

The robot stores controller parameters using ESP32 `Preferences`.

Persisted values include:

```text
kp_drive
kd_drive

kp_turn
kd_turn

search_speed
fast_speed
```

The configuration is stored under the `micromouse` namespace.

This gives the firmware a simple experimental calibration workflow:

```text
Tune
  ↓
Test
  ↓
Observe
  ↓
Adjust
  ↓
Save
  ↓
Reboot
  ↓
Retain parameters
```

The important Computer Science concept is that experimental configuration is separated from the compiled algorithmic logic.

---

# 27 — Observability & Diagnostics

The robot exposes an embedded diagnostics mode.

The OLED displays:

```text
IR Left  : raw + wall
IR Front : raw + wall
IR Right : raw + wall

Encoder L
Encoder R

Yaw
```

The diagnostic loop periodically updates these values and allows the user to exit through the Select button.

This creates an important development principle:

> **Autonomous systems need observability, not just autonomy.**

When physical behavior diverges from expected behavior, sensor values and internal state must be inspectable.

---

# 28 — Design Decisions

| Problem             | Decision                        | Computer Science / Engineering Reason                |
| ------------------- | ------------------------------- | ---------------------------------------------------- |
| Unknown graph       | Online maze mapping             | Graph is constructed incrementally from observations |
| Grid topology       | 2D fixed arrays                 | Direct (O(1)) cell access                            |
| Wall representation | Bit flags                       | Compact state encoding                               |
| Shortest cell path  | BFS / flood-fill                | Correct for unit-cost grid edges                     |
| Multiple goals      | Four zero-distance center cells | Multi-source BFS                                     |
| BFS storage         | Fixed queue                     | Bounded memory                                       |
| Encoder acquisition | GPIO ISR                        | Asynchronous event capture                           |
| Motion state        | Position + heading              | Discrete state abstraction                           |
| Turn control        | Gyro-feedback PD                | Measured angular response                            |
| Forward control     | Encoder + IR feedback           | Reduces dependence on ideal motors                   |
| Configuration       | NVS persistence                 | Experimental parameters survive reboot               |
| Failure handling    | Explicit error state            | Prevents silent continuation                         |
| Safety              | Battery monitor                 | Protects against low-voltage operation               |

---

# 29 — Why This Is a Computer Science Project

Although the physical artifact is a robot, the core intellectual structure is computational.

### Data Structures

The robot maintains:

```text
Grid
 ├── Cell state
 ├── Wall bitfield
 ├── Visited flag
 └── Distance field

Queue
 ├── head
 ├── tail
 └── fixed capacity
```

### Graph Theory

Physical walls define graph connectivity:

[
E={(u,v)\mid u\text{ and }v\text{ are adjacent and unblocked}}
]

### Algorithms

The planner repeatedly computes a shortest-path distance field using BFS.

### Complexity

The grid representation deliberately turns spatial operations into bounded array accesses.

### Concurrency

Encoder events arrive asynchronously through interrupts while application code is executing.

### State Machines

Operating modes are represented explicitly rather than inferred from scattered Boolean conditions.

### Systems Programming

The software must respect:

* finite memory,
* timing constraints,
* hardware failures,
* interrupt safety,
* sensor uncertainty,
* actuator limitations.

The resulting system is therefore a practical intersection of:

**Algorithms + Data Structures + Embedded Systems + Control + Cyber-Physical Computing.**

---

# 30 — What Is Actually Implemented

| Capability                           | Status                                       |
| ------------------------------------ | -------------------------------------------- |
| ESP32 firmware                       | **Implemented**                              |
| Embedded C++ / Arduino framework     | **Implemented**                              |
| 16×16-style fixed maze architecture  | **Implemented via configurable `MAZE_SIZE`** |
| Wall bit encoding                    | **Implemented**                              |
| Visited-cell representation          | **Implemented**                              |
| BFS / flood-fill                     | **Implemented**                              |
| Four-cell center goal                | **Implemented**                              |
| Online wall mapping                  | **Implemented**                              |
| Online replanning                    | **Implemented**                              |
| Encoder interrupts                   | **Implemented**                              |
| MPU6050 yaw integration              | **Implemented**                              |
| Wall-relative steering               | **Implemented**                              |
| PD turn control                      | **Implemented**                              |
| Persistent controller parameters     | **Implemented**                              |
| Diagnostics                          | **Implemented**                              |
| Battery protection                   | **Implemented**                              |
| A*                                   | **Not implemented**                          |
| EKF / sensor fusion                  | **Not implemented**                          |
| Probabilistic mapping                | **Not implemented**                          |
| Formal WCET analysis                 | **Not implemented**                          |
| Hardware-in-the-loop benchmark suite | **Not implemented**                          |

This distinction is intentional: the README documents what the source demonstrates rather than using advanced terminology to imply capabilities that are not present.

---

# 31 — Current Limitations

A technically honest research portfolio should make its limitations visible.

The present implementation does not yet provide:

* encoder + IMU sensor fusion,
* absolute localization,
* probabilistic wall confidence,
* adaptive motor identification,
* acceleration/jerk-aware trajectory planning,
* formal timing analysis,
* automated experimental benchmarking,
* simulation-to-real validation.

These are not merely "missing features."

They define potential research questions.

For example:

[
\text{Can sensor fusion reduce heading error enough to permit faster motion?}
]

[
\text{Can adaptive motor identification improve cell-to-cell repeatability?}
]

[
\text{When does A* outperform flood-fill under realistic embedded constraints?}
]

[
\text{What is the measured worst-case execution time of each control primitive?}
]

---

# 32 — Research Directions

## 1. Sensor Fusion

Combine:

```text
Encoder odometry
       +
Gyroscope
       ↓
State estimator
```

A complementary filter or EKF could provide a more stable estimate of heading and pose.

---

## 2. Uncertainty-Aware Mapping

Replace binary:

```text
WALL / OPEN
```

with confidence-aware observations.

This could make the map robust to contradictory or noisy measurements.

---

## 3. Adaptive Motor Control

Estimate left/right actuator differences online and compensate for:

* battery voltage,
* motor mismatch,
* surface friction,
* mechanical variation.

---

## 4. Trajectory Optimization

The current planner optimizes discrete cell transitions.

A future planner could optimize:

[
J=
\alpha T+
\beta N_{turns}+
\gamma E_{tracking}
]

where (T) is traversal time and (E_{tracking}) measures motion error.

---

## 5. Flood-Fill vs A* Experimental Study

A rigorous experiment could compare:

| Metric                    | Flood-Fill |      A* |
| ------------------------- | ---------: | ------: |
| Runtime                   |    Measure | Measure |
| RAM                       |    Measure | Measure |
| Path length               |    Measure | Measure |
| Replanning cost           |    Measure | Measure |
| Implementation complexity |    Analyze | Analyze |

The important question is not which algorithm is theoretically "better."

It is:

> **Which algorithm provides the best system-level trade-off on this specific embedded platform?**

---

## 6. WCET Analysis

Measure:

```text
Encoder ISR
Sensor acquisition
Flood-fill
Neighbor selection
Drive controller
Turn controller
OLED update
```

Then characterize:

[
T_{worst}
=========

\max(T_1,T_2,\ldots,T_n)
]

This would turn qualitative real-time claims into measurable engineering evidence.

---

# 33 — Experimental Validation Roadmap

No controlled performance dataset is present in the supplied firmware, so no run-time or accuracy claims are fabricated.

A rigorous evaluation should measure:

| Metric                | Experimental question                                               |
| --------------------- | ------------------------------------------------------------------- |
| Maze completion rate  | How reliably does the complete system solve different mazes?        |
| Search time           | How much time is spent acquiring the world model?                   |
| Fast-run time         | How effectively is the learned map exploited?                       |
| Turn error            | How accurately does the gyro controller achieve 90°/180° rotations? |
| Cell transition error | Does physical motion agree with the discrete state model?           |
| Planner runtime       | What is the measured cost of flood-fill?                            |
| Control-loop period   | How deterministic is the motion loop?                               |
| RAM / flash usage     | How much of the MCU resources are consumed?                         |
| Battery sensitivity   | How does supply voltage affect behavior?                            |

The principle is simple:

> **Measure first. Claim second.**

---

# 34 — A Researcher's View of the System

The project can be understood as a sequence of abstractions:

```text
PHYSICAL WORLD
      │
      ▼
SENSOR SPACE
      │
      ▼
DISCRETE OBSERVATION
      │
      ▼
GRAPH REPRESENTATION
      │
      ▼
GRAPH ALGORITHM
      │
      ▼
MOTION ABSTRACTION
      │
      ▼
CONTROL LAW
      │
      ▼
PHYSICAL ACTUATION
      │
      └──────────────► feedback
```

Each layer introduces assumptions.

The engineering challenge is ensuring those assumptions remain compatible.

For example:

```text
Planner assumes:
    "North neighbor is reachable."

Controller must ensure:
    "Robot actually rotates toward North."

Sensors must ensure:
    "The wall model used by the planner reflects reality."

Encoders / IMU must ensure:
    "Physical motion can be measured sufficiently well."

Safety layer must ensure:
    "A failed motion does not silently corrupt the state machine."
```

This is the central systems lesson of the project.

---

# 35 — Engineering Takeaways

### 01 — Algorithms live inside physical systems

A theoretically correct graph algorithm is insufficient if the robot cannot execute the resulting transition reliably.

### 02 — Representation determines computation

The decision to represent the maze as compact directional cell state makes graph traversal straightforward and memory-bounded.

### 03 — Feedback closes the abstraction gap

Encoder, IR and gyroscope measurements connect discrete algorithmic decisions to continuous physical behavior.

### 04 — Resource constraints influence algorithm design

A fixed grid and bounded queue make BFS practical without introducing dynamic graph structures.

### 05 — Failure behavior is part of the algorithm

A timeout or sensor failure changes the system state and therefore must be explicitly represented.

### 06 — Observability is an engineering capability

Diagnostics provide the information required to understand why a physical system behaves differently from its software model.

---

# 36 — Repository Philosophy

This project follows a simple documentation principle:

> **Do not hide complexity behind advanced vocabulary. Expose the actual design decisions.**

The most meaningful parts of the implementation are not the names of the components.

They are the relationships:

```text
IR sensors
    ↓
Wall observation
    ↓
Maze graph
    ↓
BFS distance field
    ↓
Neighbor selection
    ↓
Motion command
    ↓
Encoder / gyro feedback
    ↓
Motor correction
    ↓
Physical trajectory
```

That pipeline is the project.

---

# 37 — Future Architecture

A natural evolution of the current codebase would be:

```text
                    ┌─────────────────────┐
                    │    Application FSM  │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
       ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
       │ Navigation  │  │   Control   │  │ Diagnostics │
       └──────┬──────┘  └──────┬──────┘  └─────────────┘
              │                │
              ▼                ▼
       ┌─────────────┐  ┌─────────────┐
       │ Maze Model  │  │ Drive Ctrl  │
       │ Flood Fill  │  │ Turn Ctrl   │
       └──────┬──────┘  └──────┬──────┘
              │                │
              └───────┬────────┘
                      ▼
               ┌─────────────┐
               │ Perception  │
               ├─────────────┤
               │ IR          │
               │ Encoders    │
               │ IMU         │
               └──────┬──────┘
                      │
                      ▼
               Hardware Layer
```

This would enable unit testing of the planner independently from the robot hardware and would make algorithmic experiments easier to reproduce.

---

# 38 — Closing Perspective

Micromouse-bot is a compact example of a broader Computer Science problem:

[
\boxed{
\text{How does an algorithm make reliable decisions when its world model is incomplete, its computation is bounded, and its actions have physical consequences?}
}
]

The implementation approaches that problem through:

```text
Grid Data Structures
        +
Graph Search
        +
Interrupt-Driven Acquisition
        +
Sensor Interpretation
        +
Feedback Control
        +
Explicit State Machines
        +
Bounded Memory
        +
Failure Handling
```

The maze is only the environment.

The deeper project is the construction of a computational system that continuously closes the loop between:

**representation → algorithm → action → observation → updated representation.**

---

<div align="center">

### MICROMOUSE-BOT

**Algorithms that have to survive contact with reality.**

`Data Structures` · `Graph Theory` · `Embedded Systems` · `Control` · `Autonomous Systems`

</div>
