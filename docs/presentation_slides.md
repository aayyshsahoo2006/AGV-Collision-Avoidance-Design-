# Design & Simulation of an Autonomous Collision-Avoidance Subsystem for AGVs
## B.Tech Final Year Project — Presentation Deck (12 Slides)

---

### Slide 1: Title & Project Overview
- **Project Title:** Design & Simulation of an Autonomous Collision-Avoidance Subsystem for AGVs
- **Domain:** Autonomous Mobile Robots (AMRs) / Embedded Linux Safety Systems
- **Implementation:** Modern C++17, POSIX Device Driver Emulation, Interactive Web Simulator
- **Core Focus:** Dynamic Stopping Physics, Time-to-Collision (TTC), and Artificial Potential Field (APF) Swerving

---

### Slide 2: Industry Problem Statement & Motivation
- **The Problem:** Low-cost AGVs use fixed distance cutoffs (e.g. 1 meter stop line).
- **Failure 1 (Collision at high speed):** Fast oncoming obstacles cannot be stopped within 1 meter.
- **Failure 2 (False alarms & stop-and-go):** Receding obstacles or side shelves trigger unnecessary stops.
- **Our Solution:** A dynamic multi-tier safety system factoring in instant vehicle velocity, relative closing speed, and swept-volume geometry.

---

### Slide 3: Project Aim & Key Objectives
1. **Kinematic Modeling:** 2D differential-drive unicycle kinematics with acceleration and deceleration limits.
2. **Dynamic Risk Engine:** Equations for speed-dependent stopping distance and Time-To-Collision (TTC).
3. **4-State Safety FSM:** NORMAL, WARNING, SLOWDOWN, and EMERGENCY_STOP with recovery debouncing.
4. **Autonomous Bypass:** Artificial Potential Field (APF) local path planning to swerve around obstacles.
5. **POSIX Device Interface:** Hardware-independent character driver (`/dev/agv_radar`) with thread-safe ring buffering.

---

### Slide 4: System Architecture & Block Diagram
```
[ 2D Simulation World ] ──► [ 64-Ray Virtual LiDAR ] ──► [ POSIX Device Driver /dev/agv_radar ]
                                                                      │
                                                                      ▼
[ AGV Kinematics (v,w) ] ◄── [ APF Motion Controller ] ◄── [ Risk Engine & 4-State Safety FSM ]
```
- **Hardware Isolation:** Replacing virtual sensors with physical radar requires zero changes to the safety algorithm.
- **C++17 Standards:** RAII, clean memory management, and deterministic 20 Hz (50ms) control loop.

---

### Slide 5: Dynamic Stopping Distance Model
- **Governing Physics Equation:**
  $$d_{\text{stop}}(v) = v \cdot t_r + \frac{v^2}{2a} + d_s$$
- **Parameters:**
  - $v$: Current velocity ($0 \le v \le 1.5\text{ m/s}$)
  - $t_r = 0.2\text{ s}$: Sensor & processing latency
  - $a = 1.2\text{ m/s}^2$: Braking deceleration
  - $d_s = 0.3\text{ m}$: Physical clearance safety margin
- **Graduated Safety Zones:**
  - **Emergency Stop:** $d \le d_{\text{stop}}$ (Full brake, $v_{\text{cmd}} = 0$)
  - **Slowdown:** $d \le d_{\text{stop}} + 1.2\text{ m}$ (Reduce speed to 45%)
  - **Warning:** $d \le d_{\text{stop}} + 3.4\text{ m}$ (Log alert, maintain cruise)

---

### Slide 6: Time-To-Collision (TTC) for Dynamic Targets
- **TTC Formulation:**
  $$\text{TTC} = \frac{\text{Surface Distance}}{v_{\text{closing}}}$$
- **Closing Velocity:** $v_{\text{closing}} = -\frac{\vec{r} \cdot \vec{v}_{\text{rel}}}{\|\vec{r}\|}$
- **Triggers:**
  - $\text{TTC} \le 1.0\text{ s} \implies$ Immediate `EMERGENCY_STOP`
  - $\text{TTC} \le 2.5\text{ s} \implies$ Transition to `SLOWDOWN`
  - $v_{\text{closing}} \le 0 \implies \text{TTC} = \infty$ (Zero false alarms on receding targets)

---

### Slide 7: Swept-Volume Corridor Detection
- **Local Coordinate Transform:**
  $$x_{\text{local}} = \Delta x \cos(-\theta) - \Delta y \sin(-\theta)$$
  $$y_{\text{local}} = \Delta x \sin(-\theta) + \Delta y \cos(-\theta)$$
- **Swept-Volume Check:**
  $$\text{Lateral Overlap} = |y_{\text{local}}| - r_{\text{obstacle}}$$
  $$\text{inCorridor} = (\text{Lateral Overlap} \le \frac{w_{\text{AGV}}}{2} + \text{margin})$$
- **Benefit:** Accounts for obstacle physical radius, preventing clipping of off-center obstacles.

---

### Slide 8: Artificial Potential Field (APF) Dynamic Steering
- **Attractive Force ($F_{\text{att}}$):** Pulls AGV forward ($F_x = 3.0$) and restores to center lane ($F_y = -1.8 \cdot y$).
- **Repulsive Force ($F_{\text{rep}}$):** Pushes AGV sideways away from obstacles ($F_{\text{rep}} \propto \frac{1}{(d-r)^2}$).
- **Resultant Angle:** $\theta_{\text{desired}} = \text{atan2}(F_y, F_x) \implies \omega = K_p (\theta_{\text{desired}} - \theta_{\text{AGV}})$.
- **Gridlock Safety:** 1D interval merging detects full barricades and enforces straight emergency braking.

---

### Slide 9: Linux Character Device Driver (`/dev/agv_radar`)
- **Binary Hardware Packet:**
  - Magic Header: `0x52414452` ('RADR')
  - 64-bit Timestamp, 32-bit Distance, 16-bit Azimuth, 16-bit Speed, XOR Checksum
- **POSIX System Calls:** `dev_open()`, `dev_close()`, `dev_read()`, `dev_write()`.
- **Embedded Safety:**
  - `std::memcpy` for binary serialization (ARM alignment-safe).
  - Mutex-protected circular ring buffer (prevents race conditions).

---

### Slide 10: Test Scenarios & Verification (100% Pass)
1. **Free Cruise:** Nominal cruise at 1.2 m/s; 0 false alarms.
2. **Center Obstacle:** Smooth APF bypass and return to lane center.
3. **Off-Track Protrusion:** Protruding obstacle detected by swept volume; swerved safely.
4. **Dynamic Crossing:** Moving pedestrian at 0.4 m/s; TTC triggers early slowdown.
5. **Slalom S-Curve:** Continuous smooth weaving through 3 staggered obstacles without stopping.
6. **Full Gridlock Wall:** Barricade detected; straight stop with $+0.37\text{m}$ clearance buffer.

---

### Slide 11: Real Engineering Challenges We Solved
1. **Threshold Hunting Oscillation:** Speed drop shrank stopping distance, causing start-stop cycling. *Solution:* Evaluated thresholds at $\max(v_{\text{actual}}, v_{\text{nominal}})$.
2. **Slalom False Gridlock Bug:** Spaced obstacles were merged into a single wall. *Solution:* Grouped obstacles into longitudinal X slices ($|x_1 - x_2| \le 1.5\text{m}$).
3. **Recovery Hysteresis:** Sensor noise caused chattering after E-stop. *Solution:* Added 0.8s confirmation timer and $+0.5\text{m}$ clearance buffer.

---

### Slide 12: Conclusion & Future Scope
- **Conclusion:** Delivered a complete, verified C++17 safety architecture with APF swerving, POSIX driver layer, and interactive simulator. All 28 test cases passed at 100%.
- **Future Scope:**
  1. ROS 2 Lifecycle node integration.
  2. Deployment on NVIDIA Jetson with physical CAN bus automotive radar.
  3. 3D Gazebo / Isaac Sim warehouse simulation.
