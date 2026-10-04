# 🎓 B.Tech Final Year Project — Presentation & Viva Guide
## Project: Design and Simulation of an Autonomous Collision-Avoidance Subsystem for AGVs

> **Tone & Style:** Simple, clear, confident engineering language suitable for your Final Year Project (FYP) Presentation, Viva Voce, and Project Defense in front of external examiners and department faculty.

---

## 📋 Presentation Flow (10–12 Minute Presentation)

| Slide # | Slide Title | Speaking Time | What to Focus On |
|---|---|---|---|
| **1** | Title & Introduction | 1 min | Clear intro: What is an AGV and what did you build? |
| **2** | Motivation & Problem Statement | 1.5 min | Why do standard AGVs fail with simple fixed sensors? |
| **3** | Project Aim & Objectives | 1 min | 4 concrete goals you achieved. |
| **4** | System Architecture & Block Diagram | 1.5 min | Simple 5-block pipeline (Sensor → Driver → Brain → FSM → Motors). |
| **5** | Mathematical Safety Model (Stopping Distance) | 1.5 min | Kinematics formula: $d_{stop} = v \cdot t_r + \frac{v^2}{2a} + d_s$. |
| **6** | Dynamic Risk & Time-To-Collision (TTC) | 1 min | Why TTC is needed for moving obstacles. |
| **7** | Swept-Volume & Local Coordinate Transformation | 1 min | How the robot checks if something is in its path. |
| **8** | Collision Avoidance (Artificial Potential Fields) | 1.5 min | How it steers around obstacles smoothly like gravity/magnets. |
| **9** | Linux Device Driver Layer (`/dev/agv_radar`) | 1 min | Embedded Linux & POSIX character driver concept. |
| **10** | Test Scenarios & Results (Live Demo) | 1.5 min | 6 test cases (Cruise, Center, Off-track, Crossing, Slalom, Gridlock). |
| **11** | Engineering Challenges Faced & Solutions | 1 min | Real debugging stories (Hunting, Slalom false gridlock, Alignment). |
| **12** | Conclusion & Future Scope | 0.5 min | Summary + ROS2 / Physical Hardware roadmap. |

---

## 🗣️ Slide-by-Slide Speaker Notes (What to say during your presentation)

---

### Slide 1: Title & Introduction
> **Speaker Script:**  
> *"Respected guide, external examiners, and faculty members, good morning.  
> My final year project is titled **'Design and Simulation of an Autonomous Collision-Avoidance Subsystem for AGVs'**.  
> In modern smart warehouses and industries, Automated Guided Vehicles (AGVs) carry heavy payloads (often 300 to 500 kg) alongside human workers. If an AGV fails to detect an obstacle in time, the kinetic energy can cause severe accidents.  
> In this project, I have developed a modular C++17 safety architecture with an emulated Linux device driver and an interactive simulator that dynamically calculates safe stopping distances, time-to-collision, and steers around obstacles using Artificial Potential Fields."*

---

### Slide 2: Problem Statement & Motivation
> **Speaker Script:**  
> *"Most conventional low-cost AGVs use **fixed-distance threshold sensors** (for example, stop whenever anything is within 1 meter).  
> But in practical robotics, this fails in two major ways:
> 1. If the AGV is moving fast and an oncoming vehicle approaches head-on, 1 meter is too short to brake safely without collision.
> 2. If an obstacle is far to the side or moving away, a fixed sensor triggers unnecessary emergency stops, causing massive throughput loss and jerky stop-and-go motion.  
> **Our solution:** A multi-tier dynamic safety system that calculates stopping distance based on actual vehicle speed and relative closing velocity."*

---

### Slide 3: Project Objectives
> **Speaker Script:**  
> *"The key objectives of our project were:
> 1. **Kinematic Modeling:** Model 2D unicycle AGV kinematics with acceleration and deceleration limits.
> 2. **Dynamic Risk Engine:** Implement physical stopping distance equations and Time-to-Collision (TTC).
> 3. **4-State Safety Controller:** Implement a Finite State Machine (NORMAL, WARNING, SLOWDOWN, EMERGENCY STOP) with hysteresis to prevent state chattering.
> 4. **Autonomous Avoidance:** Implement Artificial Potential Fields (APF) so the AGV can smoothly bypass obstacles.
> 5. **POSIX Device Interface:** Emulate a Linux character device driver (`/dev/agv_radar`) with thread-safe ring buffering."*

---

### Slide 4: System Architecture (Block Diagram)

```
┌─────────────────┐      ┌─────────────────────────┐      ┌──────────────────────┐
│  2D Simulation  │ ───► │  Virtual LiDAR Sensor   │ ───► │ Linux Device Driver  │
│      World      │      │ (64-Ray Cone, 110° FOV) │      │  (/dev/agv_radar)    │
└─────────────────┘      └─────────────────────────┘      └──────────────────────┘
                                                                     │
                                                                     ▼
┌─────────────────┐      ┌─────────────────────────┐      ┌──────────────────────┐
│ AGV Kinematics  │ ◄─── │    Motion Controller    │ ◄─── │ Collision Risk Engine│
│ (v, w, x, y, θ) │      │  (APF Steering + Speed) │      │  & 4-State Safety FSM│
└─────────────────┘      └─────────────────────────┘      └──────────────────────┘
```

> **Speaker Script:**  
> *"Here is our modular architecture. The data flows cleanly in one direction:  
> The 2D World feeds geometric obstacles to the Virtual Sensor. The sensor packages radar packets into an emulated Linux character device buffer. Our Risk Engine reads this buffer, evaluates distance and TTC, and feeds the Safety State Machine. Finally, the Motion Controller computes linear speed and steering angular velocity using APF forces to drive the AGV."*

---

### Slide 5: Stopping Distance Physics Formula
> **Speaker Script:**  
> *"The core safety equation comes from standard vehicle kinematics:
> $$d_{stop}(v) = v \cdot t_r + \frac{v^2}{2a} + d_s$$
> - $v \cdot t_r$: Reaction distance (distance travelled during the 0.2s sensor and processing delay).
> - $\frac{v^2}{2a}$: Kinematic braking distance under constant deceleration $a = 1.2\text{ m/s}^2$.
> - $d_s$: Mandatory safety buffer ($0.3\text{ m}$) so the AGV stops with room to spare.
> 
> At nominal cruise speed $v = 1.2\text{ m/s}$, $d_{stop} = 1.14\text{ m}$.  
> Based on this, we establish 3 nested zones:
> - **Warning Zone** ($d_{stop} + 3.2\text{ m}$): Log alert, maintain speed.
> - **Slowdown Zone** ($d_{stop} + 1.2\text{ m}$): Drop speed to 45% ($0.54\text{ m/s}$).
> - **Emergency Stop Zone** ($d_{stop}$): Command full deceleration to $0\text{ m/s}$."*

---

### Slide 6: Time-To-Collision (TTC) for Dynamic Targets
> **Speaker Script:**  
> *"Static distance alone is not enough when obstacles are moving. We compute Time-To-Collision:
> $$\text{TTC} = \frac{\text{Surface Distance}}{v_{\text{closing}}}$$
> where closing velocity is the dot product of relative velocity along the line-of-sight.  
> - If $\text{TTC} \le 1.0\text{ s}$, the system triggers an immediate emergency stop.  
> - If the obstacle is receding ($v_{closing} \le 0$), TTC is set to infinity, preventing false emergency stops."*

---

### Slide 7: Swept-Volume Corridor Detection
> **Speaker Script:**  
> *"To avoid unnecessary braking for obstacles on adjacent shelves, we transform world coordinates into the AGV's local coordinate frame:
> $$x_{local} = \Delta x \cos(-\theta) - \Delta y \sin(-\theta)$$
> $$y_{local} = \Delta x \sin(-\theta) + \Delta y \cos(-\theta)$$
> Then we apply **Swept-Volume Corridor Detection**:
> $$\text{Lateral Overlap} = |y_{local}| - r_{obstacle}$$
> If $\text{Lateral Overlap} \le \frac{w_{AGV}}{2} + \text{margin}$, the obstacle physically intersects the vehicle's forward path. This accounts for the obstacle's full circular radius, preventing edge clipping."*

---

### Slide 8: Autonomous Collision Avoidance (Artificial Potential Fields)
> **Speaker Script:**  
> *"Instead of just stopping every time, our AGV can dynamically steer around obstacles using **Artificial Potential Fields (APF)**:
> - **Attractive Force ($F_{att}$):** Pulls the AGV forward along the aisle and restores it to the center lane ($y = 0$).
> - **Repulsive Force ($F_{rep}$):** Pushes the AGV laterally away from obstacles with force inversely proportional to distance squared ($F_{rep} \propto \frac{1}{(d-r)^2}$).
> 
> The resultant vector determines the target steering angle $\theta_{desired}$, which is converted to angular velocity $\omega$."*

---

### Slide 9: Linux Character Device Layer (`/dev/agv_radar`)
> **Speaker Script:**  
> *"To demonstrate embedded Linux systems concepts, we emulated a hardware character device driver:
> - Packets have a magic header `0x52414452` ('RADR'), timestamp, distance, azimuth, and XOR checksum.
> - We use POSIX system calls: `dev_open()`, `dev_close()`, `dev_read()`, `dev_write()`.
> - Thread-safe circular ring buffer protected by `std::mutex`.
> - Uses `std::memcpy` for byte serialization to prevent unaligned memory faults on embedded ARM processors."*

---

### Slide 10: Test Scenarios & Results
> **Speaker Script:**  
> *"We verified the complete system across 6 comprehensive test scenarios in our simulator:
> 1. **Free Cruise:** Full speed ($1.2\text{ m/s}$) down center aisle; 0 false alarms.
> 2. **Center Obstacle:** Smooth APF bypass and return to center lane.
> 3. **Off-Track Obstacle:** Protruding obstacle at $(4.8\text{m}, 0.8\text{m})$; detected and avoided without overlap.
> 4. **Dynamic Crossing:** Moving pedestrian; TTC triggers early slowdown and safe passage.
> 5. **Slalom S-Curve:** Weaves smoothly through 3 staggered obstacles.
> 6. **Full Gridlock Wall:** Barricade across all lanes; automatically detects impassable wall and brakes to a clean stop with $+0.37\text{m}$ clearance buffer."*

---

### Slide 11: Real Engineering Challenges We Solved (Great for Viva!)
> **Speaker Script:**  
> *"During implementation, we encountered and resolved three major technical challenges:
> 1. **Threshold Hunting:** When braking, velocity drops, which causes $d_{stop}$ to shrink, making the obstacle appear 'safe' again, resulting in rapid start-stop oscillations. *Solution:* We evaluate safety thresholds at $\max(v_{actual}, v_{nominal})$.
> 2. **Slalom vs. Gridlock Disambiguation:** An earlier bug grouped obstacles spaced 3.5m apart as a single wall. *Solution:* Added longitudinal cross-section slicing ($|x_1 - x_2| \le 1.5\text{m}$) so slalom obstacles are treated as navigable corridors.
> 3. **Recovery Hysteresis:** Avoided restart chattering by holding the emergency stop for a $0.8\text{s}$ confirmation timer and requiring $+0.5\text{m}$ clearance buffer."*

---

### Slide 12: Conclusion & Future Scope
> **Speaker Script:**  
> *"**Conclusion:** We have successfully built and verified a modular, C++17 AGV collision avoidance system with dynamic stopping physics, APF navigation, and Linux device abstraction. All 28 automated test cases passed with a 100% success rate and 0 collisions.  
> **Future Scope:**
> 1. Integration with **ROS 2 (Robot Operating System)** as a lifecycle node.
> 2. Hardware deployment onto an **NVIDIA Jetson Nano** with a physical CAN bus automotive radar.
> 3. Extending from 2D planar kinematics to 3D Gazebo simulation.  
> Thank you! I am now ready for questions."*

---

## 💡 Top 10 Viva Voce / External Examiner Questions & Answers

### Q1: What is the main difference between your system and traditional AGV safety sensors?
> **Answer:** *"Traditional AGVs use fixed distance cutoffs (e.g., 1 meter). Our system calculates dynamic stopping distance $d_{stop} = v \cdot t_r + \frac{v^2}{2a} + d_s$ based on real-time vehicle speed and incorporates Time-To-Collision (TTC) for moving targets. It also actively swerves around obstacles using Artificial Potential Fields rather than just stopping."*

### Q2: Why did you choose C++17 for the implementation?
> **Answer:** *"C++17 is the industry standard for real-time robotics and automotive safety systems (AUTOSAR/ROS2). It provides deterministic execution without garbage collection pauses, low-level memory control, and strong type safety."*

### Q3: What is 'Threshold Hunting' and how did you fix it?
> **Answer:** *"As the AGV brakes, its speed decreases. If $d_{stop}$ is computed purely from current speed, the threshold shrinks as the vehicle slows down. The obstacle suddenly falls outside the reduced threshold, causing the robot to release the brakes and re-accelerate, creating an oscillation loop. We fixed this by evaluating thresholds at $\max(v_{actual}, v_{nominal})$."*

### Q4: How does the Artificial Potential Field (APF) work?
> **Answer:** *"The goal generates an attractive force pulling the AGV forward and toward the center lane, while each obstacle within the sensor range generates a repulsive force inversely proportional to the square of surface distance ($F_{rep} \propto 1/d^2$). The vector sum of these forces gives the desired steering heading."*

### Q5: What happens if an obstacle completely blocks the aisle (Local Minima / Gridlock)?
> **Answer:** *"In a standard APF, opposing repulsive forces from a wall can cancel out laterally, causing the vehicle to drive straight into the wall. We implemented a 1D interval merging algorithm that detects if obstacles at the same X-slice block all lanes. If gridlock is detected, the APF is bypassed and straight emergency braking is enforced."*

### Q6: What is the purpose of the Linux character device driver layer?
> **Answer:** *"It decouples the safety algorithm from the sensor hardware. The risk engine reads from `/dev/agv_radar` via standard POSIX `read()` calls. If we replace the virtual sensor with a physical radar or LiDAR sensor, only the driver layer changes — zero lines in the safety engine need to be modified."*

### Q7: Why do you need both stopping distance AND Time-to-Collision (TTC)?
> **Answer:** *"Distance checks stationary objects well, but if an obstacle is moving toward the AGV at high relative speed (e.g. $2\text{ m/s}$), it will cross the stopping boundary too fast for normal reaction time. TTC calculates $\frac{d}{v_{closing}}$ and triggers an immediate emergency stop if contact is less than $1.0\text{ s}$ away."*

### Q8: What is Hysteresis in your Safety State Machine?
> **Answer:** *"Hysteresis prevents rapid state switching due to sensor noise near threshold boundaries. When clearing an emergency stop, the obstacle must not only clear the $d_{stop}$ line, but must clear by an extra $0.5\text{m}$ buffer and stay clear continuously for at least $0.8\text{ seconds}$ before the vehicle resumes motion."*

### Q9: How do you handle sensor noise in the simulation?
> **Answer:** *"We modeled zero-mean Gaussian noise ($\sigma = 0.05\text{m}$) on sensor range readings. In Test Case TC-09, our debounced state machine maintained a 100% stable state over 50 consecutive cycles without any transition jitter."*

### Q10: What are the primary kinematic constraints of your AGV?
> **Answer:** *"We use the standard differential-drive Unicycle model: $\dot{x} = v \cos\theta$, $\dot{y} = v \sin\theta$, $\dot{\theta} = \omega$, with maximum linear velocity $v_{max} = 1.5\text{ m/s}$, maximum acceleration $a = 1.0\text{ m/s}^2$, and angular velocity clamped to $\pm 1.8\text{ rad/s}$."*
