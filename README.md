# 🤖 ESP32 Micromouse Robot

An autonomous Micromouse robot developed using ESP32 that can explore an unknown maze, generate a map, compute the shortest path using the Flood Fill algorithm, and perform a high-speed final run with PID wall following.

---

## 📌 Overview

This project was developed to demonstrate embedded systems programming, robotics, autonomous navigation, and path-planning algorithms.

The robot autonomously:

- Detects maze walls using IR sensors
- Explores unknown mazes
- Generates an internal maze map
- Solves the maze using the Flood Fill algorithm
- Performs a fast run on the shortest path
- Uses PID control for accurate wall centering
- Stores calibration data using ESP32 Preferences
- Displays status on an OLED display

---

## 🚀 Features

- ESP32 based controller
- Flood Fill shortest path algorithm
- PID wall-following controller
- Automatic sensor calibration
- OLED status display
- Motor speed correction
- EEPROM/Preferences memory support
- Modular repository structure
- Ready for future simulator integration

---

## 🛠 Hardware Used

| Component | Description |
|-----------|-------------|
| ESP32 | Main controller |
| TB6612FNG | Motor driver |
| N20 Gear Motors | Differential drive |
| IR Sensor Array | Wall detection |
| OLED Display | Status information |
| Li-ion Battery | Power supply |

---

## 📁 Repository Structure

```
Micromouse-Robot/
│
├── docs/
├── hardware/
├── include/
├── sim/
├── src/
│   └── micromouse_final.ino
├── test/
├── README.md
└── LICENSE
```

---

## 🧠 Algorithms Used

- Flood Fill Algorithm
- PID Controller
- Wall Detection
- Maze Mapping
- Path Optimization

---

## 🔧 Development Workflow

Phase 1
- Robot movement
- Motor testing

Phase 2
- Sensor integration
- Wall detection

Phase 3
- PID tuning
- Stable navigation

Phase 4
- Maze solving
- Flood Fill implementation
- Fast run

---

## 📈 Future Improvements

- Diagonal movement
- Dynamic speed adjustment
- Encoder-based odometry
- Simulator support
- Automatic maze visualization
- Modular C++ architecture

---

## 📄 License

This project is released under the MIT License.
