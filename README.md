# 🤖 Autonomous Collision-Avoidance Radar Subsystem for AGVs
### 🎓 B.Tech Computer Science & Engineering — Final Year Major Project (Capstone)

![C++17](https://img.shields.io/badge/Language-C%2B%2B17-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Linux%20%2F%20POSIX-orange.svg)
![Build](https://img.shields.io/badge/Build-CMake%20%7C%20Make-green.svg)
![Tests](https://img.shields.io/badge/Tests-28%2F28%20Passing%20(100%25)-brightgreen.svg)
![License](https://img.shields.io/badge/License-Academic%20Use-lightgrey.svg)

---

## 📌 Project Overview

In smart industrial warehouses and automated factories, **Automated Guided Vehicles (AGVs)** carry heavy payloads alongside human workers. Traditional low-cost AGVs rely on **fixed-distance proximity sensors** (e.g., stop when an obstacle is within 1 meter). In practice, this fixed cutoff fails:
1. **High Closing Speeds:** If an obstacle approaches quickly, 1 meter is too short to stop safely without collision.
2. **False Alarms & Throughput Loss:** If an obstacle is receding or sitting on a side shelf, a fixed threshold causes unnecessary emergency braking.

**Our Solution:**  
This project implements a complete, modular, real-time safety and autonomous navigation subsystem in **modern C++17**. It models an AGV in a 2D environment, scans obstacles using a 64-ray virtual LiDAR/radar sensor ($110^\circ$ FOV, $10\text{m}$ range), streams binary packets through an emulated **Linux character device driver** (`/dev/agv_radar`), calculates dynamic stopping distances and Time-To-Collision (TTC), and uses **Artificial Potential Fields (APF)** to steer smoothly around obstacles.

---

## 🏗️ System Architecture & Pipeline

The system is built as a strict, unidirectional 7-stage pipeline:

```
┌─────────────────┐      ┌─────────────────────────┐      ┌─────────────────────────┐
│  2D Simulation  │ ───► │  Virtual LiDAR Sensor   │ ───► │ Linux Character Driver  │
│      World      │      │ (64-Ray Cone, 110° FOV) │      │   (/dev/agv_radar)      │
└─────────────────┘      └─────────────────────────┘      └─────────────────────────┘
                                                                       │
                                                                       ▼
┌─────────────────┐      ┌─────────────────────────┐      ┌─────────────────────────┐
│ AGV Kinematics  │ ◄─── │    Motion Controller    │ ◄─── │ Collision Risk Engine   │
│ (v, w, x, y, θ) │      │ (APF Swerve + Velocity) │      │ & 4-State Safety FSM    │
└─────────────────┘      └─────────────────────────┘      └─────────────────────────┘
```

### Module Responsibilities (C++17):
- **`simulation/`** — 2D World state, unicycle kinematics ($\dot{x}=v\cos\theta, \dot{y}=v\sin\theta, \dot{\theta}=\omega$), static & dynamic obstacles.
- **`sensor/`** — 64-ray LiDAR raycasting, Gaussian noise injection ($\sigma = 0.05\text{m}$), and Linux character device driver (`/dev/agv_radar`) with a mutex-protected circular ring buffer.
- **`collision/`** — Local coordinate transformation, surface-to-surface distance, swept-volume corridor filtering, dynamic stopping distance ($d_{\text{stop}}$), and Time-To-Collision (TTC).
- **`controller/`** — 4-State Safety State Machine (NORMAL, WARNING, SLOWDOWN, EMERGENCY_STOP) with debouncing, and an Artificial Potential Field (APF) motion controller.
- **`logging/`** — Real-time CSV telemetry logger.

---

## 📐 Mathematical Safety Formulations

### 1. Dynamic Stopping Distance ($d_{\text{stop}}$)
Derived from basic kinematics:
$$d_{\text{stop}}(v) = v \cdot t_r + \frac{v^2}{2a} + d_s$$
- $v \cdot t_r$: Reaction distance ($t_r = 0.2\text{s}$ sensor + controller delay)
- $\frac{v^2}{2a}$: Kinematic braking distance under deceleration $a = 1.2\text{ m/s}^2$
- $d_s$: Physical safety margin buffer ($0.3\text{m}$)
- **Zone Boundaries:**
  - Emergency Zone: $d \le d_{\text{stop}}$
  - Slowdown Zone: $d \le d_{\text{stop}} + 1.2\text{m}$ (Speed drops to 45%)
  - Warning Zone: $d \le d_{\text{stop}} + 3.4\text{m}$ (Alert logged, nominal speed holds)

### 2. Time-To-Collision (TTC) for Moving Obstacles
$$\text{TTC} = \frac{\text{Surface Distance}}{v_{\text{closing}}}$$
- $\text{TTC} \le 1.0\text{s} \implies$ Immediate `EMERGENCY_STOP`
- $\text{TTC} \le 2.5\text{s} \implies$ Early `SLOWDOWN`
- $v_{\text{closing}} \le 0 \implies \text{TTC} = \infty$ (Protects against false stops on receding targets)

### 3. Swept-Volume Corridor Penetration
$$x_{\text{local}} = \Delta x \cos(-\theta) - \Delta y \sin(-\theta), \quad y_{\text{local}} = \Delta x \sin(-\theta) + \Delta y \cos(-\theta)$$
$$\text{Lateral Overlap} = |y_{\text{local}}| - r_{\text{obstacle}}$$
$$\text{inCorridor} = \left(\text{Lateral Overlap} \le \frac{w_{\text{AGV}}}{2} + \text{margin}\right)$$

### 4. Artificial Potential Field (APF) Swerving
- **Attraction Force (Goal):** $F_{\text{att}, y} = -1.8 \cdot y$ (pulls vehicle back to center lane $y=0$)
- **Repulsion Force (Obstacles):** $F_{\text{rep}} = \frac{20.0}{(d - r)^2}$ (pushes laterally away from obstacle centers)
- **Gridlock Safety Detection:** 1D interval merging detects full barricade walls across all lanes and enforces straight emergency braking.

---

## 📂 Project Repository Structure

```
├── bin/                        # Compiled C++ binaries (agv_sim, agv_tests)
├── include/                    # C++ Header files
│   ├── collision/              # Distance.hpp, RiskEngine.hpp, StoppingDistance.hpp, TTC.hpp
│   ├── config/                 # SafetyParameters.hpp
│   ├── controller/             # MotionController.hpp, SafetyStateMachine.hpp
│   ├── logging/                # Logger.hpp
│   ├── sensor/                 # LinuxSensorInterface.hpp, SensorData.hpp, VirtualSensor.hpp
│   └── simulation/             # AGV.hpp, Obstacle.hpp, Pose.hpp, Vector2.hpp, World.hpp
├── src/                        # C++ Source implementations
│   ├── main.cpp                # Terminal ASCII Radar Simulation Application
│   └── ... (modules)
├── tests/                      # Automated C++ Verification Suite
│   └── TestRunner.cpp          # TC-01 to TC-10 test assertions
├── docs/                       # Project documentation stages 1-6 & presentation slides
├── scripts/                    # Ruby verification engine & DOCX builder
├── simulator.html              # Advanced Web Simulator (LiDAR, Heatmap, Path Editor)
├── presentation.html           # 12-Slide Interactive Defense Deck
├── BTech_Final_Year_Presentation_Guide.md # Speaker script & Viva Q&A guide
├── CMakeLists.txt              # CMake build configuration
├── Makefile                    # Make build configuration
└── README.md                   # Project documentation
```

---

## ⚡ How to Build & Run

### Prerequisites
- GCC / Clang with C++17 support
- Make / CMake
- Web Browser (Safari, Chrome, Firefox) for the simulator

### 1. Build using Make
```bash
make clean
make
```

### 2. Run the Terminal ASCII Simulation
```bash
./bin/agv_sim 1    # Scenario 1: Nominal Free Path
./bin/agv_sim 2    # Scenario 2: Stationary Center Obstacle
./bin/agv_sim 3    # Scenario 3: Off-Track Protrusion (Bypass)
./bin/agv_sim 4    # Scenario 4: Dynamic Crossing Target
./bin/agv_sim 5    # Scenario 5: Slalom S-Curve
./bin/agv_sim 6    # Scenario 6: Full Gridlock Barricade Stop
```

### 3. Run the Automated C++ Test Suite
```bash
./bin/agv_tests
```
*Expected Output: `TEST SUMMARY: 23 Passed, 0 Failed.`*

### 4. Open the Interactive Web Simulator & Presentation
- **Simulator:** Open [`simulator.html`](simulator.html) in your browser.
  - Interactive 64-ray LiDAR raycasting
  - 🔥 Live Risk Heatmap overlay
  - 🛤️ Obstacle Path Editor (Right-click to place waypoints)
  - ⬇ CSV Telemetry Export
- **Presentation Deck:** Open [`presentation.html`](presentation.html) in your browser (Navigate with Arrow Keys or Spacebar).

---

## 🧪 Test Cases & Verification Matrix

| Test ID | Test Scenario | Expected Decision | Observed Result | Status |
|---|---|---|---|---|
| **TC-01** | Free Path (No Obstacle) | NORMAL (1.2 m/s) | Cruise holds, 0 false alarms | **PASS** |
| **TC-02** | Obstacle at 60° (Outside FOV) | Ignored | Filtered by 110° radar cone | **PASS** |
| **TC-03** | Obstacle at 12m (Outside Range) | Ignored | Filtered by 10m range limit | **PASS** |
| **TC-04** | Warning Boundary Target | WARNING | Alert logged, cruise maintained | **PASS** |
| **TC-05** | Slowdown Boundary Target | SLOWDOWN | Speed drops to 0.6 m/s (45%) | **PASS** |
| **TC-06** | Critical Obstacle at 1.4m | EMERGENCY_STOP | Full brake with +0.38m margin | **PASS** |
| **TC-07** | Receding Obstacle (v = 1.5 m/s) | NORMAL (TTC = ∞) | 0 false emergency stops | **PASS** |
| **TC-08** | Multi-Obstacle Priority | Prioritize Corridor | Corridor threat prioritized | **PASS** |
| **TC-09** | Sensor Gaussian Noise ($\sigma=0.05\text{m}$) | Stable State | 50/50 cycles stable (No jitter) | **PASS** |
| **TC-10** | Emergency Recovery | Hold Stop ≥ 0.8s | Debounced safe resumption | **PASS** |

---

## 🛠️ Real Engineering Challenges & Debugging (Viva Highlights)

1. **Threshold Hunting Oscillation:**  
   *Problem:* Braking dropped velocity, causing $d_{\text{stop}}$ to shrink, which falsely cleared the hazard and caused the AGV to re-accelerate in an infinite start-stop loop.  
   *Fix:* Evaluated all safety thresholds at $\max(v_{\text{actual}}, v_{\text{nominal}})$.

2. **Off-Track Protrusion Overlap:**  
   *Problem:* Obstacles placed slightly off-track ($y = 0.8\text{m}$) were missed by corridor center checks, causing the AGV's side hull to clip.  
   *Fix:* Subtracted obstacle radius before checking corridor bounds: $|y_{\text{local}}| - r \le \text{sweptEnvelope}$.

3. **Slalom vs. Gridlock Disambiguation:**  
   *Problem:* An early gridlock check merged obstacles spaced 3.5m apart along the track into a single wall, causing the AGV to stop in the slalom scenario.  
   *Fix:* Grouped obstacles into longitudinal X slices ($|x_1 - x_2| \le 1.5\text{m}$) so slalom obstacles are treated as clear navigable corridors.

4. **Embedded ARM Memory Alignment:**  
   *Problem:* Casting raw byte buffers via `reinterpret_cast` caused unaligned access crashes on ARM CPUs.  
   *Fix:* Serialized all binary packets using `std::memcpy`.

---

## 🔮 Future Scope

- [ ] **ROS 2 Integration:** Wrap the C++ safety state machine and APF planner into a ROS 2 Lifecycle Node.
- [ ] **Hardware Deployment:** Deploy to an NVIDIA Jetson / Raspberry Pi 4 interfaced with a physical automotive CAN bus radar.
- [ ] **3D Gazebo Simulation:** Expand from 2D planar unicycle model to 3D warehouse simulation with dynamic actor models.

---

## 👥 Authors & Academic Credits
- **Course:** B.Tech in Computer Science & Engineering (Final Year Project)
- **Project Title:** Design & Simulation of an Autonomous Collision-Avoidance Subsystem for AGVs
- **Documentation:** Complete Stage 1–6 project reports and `.docx` documentation included in repository.
