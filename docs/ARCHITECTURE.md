# AGV Collision-Avoidance Subsystem - Architecture & UML Design

This document details the software architecture, modular decomposition, data flow, and UML diagrams for the **AGV Collision-Avoidance Radar Subsystem**.

---

## 1. System Pipeline Overview

```mermaid
flowchart LR
    World["Simulation World\n(AGV & Obstacles)"] -->|"Simulated Environment"| Sensor["Virtual Sensor\n(FOV & Range Scan)"]
    Sensor -->|"Hardware Radar Packets"| Driver["Linux Device Interface\n(/dev/agv_radar)"]
    Driver -->|"Sensor Observations"| RiskEngine["Collision Risk Engine\n(d_stop & TTC)"]
    RiskEngine -->|"Risk Assessment"| FSM["Safety State Machine\n(Hysteresis & Recovery)"]
    FSM -->|"Safety State"| MotionCtrl["Motion Controller\n(Speed Ramp & Braking)"]
    MotionCtrl -->|"Velocity Command"| AGV["AGV Kinematics\n(Unicycle Model)"]
    AGV -->|"Updated Pose & Velocity"| World
    
    subgraph Support Layers
        Logger["Event & Telemetry Logger"]
        Params["Safety Parameters Config"]
    end
    
    FSM -.-> Logger
    MotionCtrl -.-> Logger
    RiskEngine -.-> Params
    FSM -.-> Params
```

---

## 2. UML Class Diagram

```mermaid
classDiagram
    class Vector2 {
        +double x
        +double y
        +length() double
        +distanceTo(Vector2 other) double
        +dot(Vector2 other) double
        +normalized() Vector2
        +toString() string
    }

    class Pose {
        +Vector2 position
        +double theta
        +forwardVector() Vector2
        +lateralVector() Vector2
        +toString() string
    }

    class SafetyParameters {
        +double agv_length
        +double agv_width
        +double max_velocity
        +double sensor_max_range
        +double sensor_fov_deg
        +double reaction_time
        +double deceleration
        +double safety_margin
        +double slowdown_margin
        +double warning_margin
        +double critical_ttc
        +double warning_ttc
        +corridorHalfWidth() double
    }

    class AGV {
        -SafetyParameters params_
        -Pose pose_
        -double linear_velocity_
        -double angular_velocity_
        +update(VelocityCommand cmd, double dt)
        +getPose() Pose
        +getLinearVelocity() double
        +getFrontBumperPosition() Vector2
    }

    class Obstacle {
        -int id_
        -Vector2 position_
        -Vector2 velocity_
        -double radius_
        -ObstacleType type_
        +update(double dt)
        +getPosition() Vector2
        +getVelocity() Vector2
    }

    class World {
        -SafetyParameters params_
        -AGV agv_
        -vector~Obstacle~ obstacles_
        -double sim_time_
        +step(VelocityCommand cmd, double dt)
        +checkPhysicalCollision() bool
    }

    class SensorObservation {
        +double timestamp
        +int obstacle_id
        +double distance
        +double angle
        +double relative_velocity
        +Vector2 relative_pos
        +bool detected
    }

    class VirtualSensor {
        -SafetyParameters params_
        -bool noise_enabled_
        -double noise_stddev_
        +scan(World world, double timestamp) vector~SensorObservation~
    }

    class LinuxSensorInterface {
        -bool is_open_
        -deque~RadarHardwarePacket~ ring_buffer_
        +dev_open() int
        +dev_close() int
        +dev_read(uint8_t* buffer, size_t count) ssize_t
        +dev_write(const uint8_t* buffer, size_t count) ssize_t
        +publishObservations(vector~SensorObservation~)
        +readObservations() vector~SensorObservation~
    }

    class RiskAssessment {
        +SafetyState recommended_state
        +int most_critical_obstacle_id
        +double min_distance
        +double min_ttc
        +double stopping_distance
        +double slowdown_distance
        +double warning_distance
        +bool obstacle_in_corridor
    }

    class RiskEngine {
        -SafetyParameters params_
        +evaluateRisk(vector~SensorObservation~, double velocity) RiskAssessment
    }

    class SafetyStateMachine {
        -SafetyParameters params_
        -SafetyState current_state_
        -SafetyState previous_state_
        -double clear_duration_
        +update(RiskAssessment risk, double dt, double time) SafetyState
    }

    class MotionController {
        -SafetyParameters params_
        +computeCommand(SafetyState, double nominal_v, RiskAssessment, double actual_v, double dt) VelocityCommand
    }

    World "1" *-- "1" AGV
    World "1" *-- "*" Obstacle
    VirtualSensor ..> World : reads
    VirtualSensor ..> SensorObservation : generates
    LinuxSensorInterface ..> SensorObservation : buffers & bridges
    RiskEngine ..> SensorObservation : evaluates
    RiskEngine ..> RiskAssessment : produces
    SafetyStateMachine ..> RiskAssessment : transitions
    MotionController ..> SafetyStateMachine : queries state
    AGV ..> MotionController : receives command
```

---

## 3. UML Sequence Diagram (Single Simulation Control Cycle)

```mermaid
sequenceDiagram
    autonumber
    participant W as World / Physics
    participant VS as Virtual Sensor
    participant LNX as Linux Device Interface (/dev/agv_radar)
    participant RE as Collision Risk Engine
    participant FSM as Safety State Machine
    participant MC as Motion Controller
    participant AGV as AGV Kinematics
    participant LOG as Telemetry Logger

    Note over W: Simulation Cycle Start (dt = 0.05s / 20Hz)
    VS->>W: Query obstacle positions & AGV pose
    VS->>VS: Perform FOV filter (+/-45°), range check (10m), corridor projection
    VS->>LNX: publishObservations(SensorObservation[])
    Note over LNX: Serializes into RadarHardwarePacket & queues in ring buffer
    LNX->>RE: dev_read() -> deserialized SensorObservation[]
    RE->>RE: Calculate d_stop(v), d_slow(v), d_warn(v), and TTC
    RE->>RE: Prioritize multiple obstacles (Corridor threat ranking)
    RE->>FSM: evaluateRisk() -> RiskAssessment
    FSM->>FSM: Evaluate state transitions & debounce recovery timer
    FSM->>MC: Current SafetyState (NORMAL / WARNING / SLOWDOWN / EMERGENCY_STOP)
    MC->>MC: Calculate target linear speed and deceleration profile
    MC->>AGV: VelocityCommand (cmd_v, brake_flag)
    AGV->>W: Update vehicle position, heading, and velocity
    FSM->>LOG: Log transitions & cycle telemetry
    Note over W: Cycle Complete
```

---

## 4. Safety State Machine Diagram

```mermaid
stateDiagram-v2
    [*] --> NORMAL

    NORMAL --> WARNING : Obstacle in corridor & d <= d_warn
    NORMAL --> SLOWDOWN : Obstacle in corridor & (d <= d_slow OR TTC <= 2.5s)
    NORMAL --> EMERGENCY_STOP : Critical hazard (d <= d_stop OR TTC <= 1.0s)

    WARNING --> SLOWDOWN : Distance closes to <= d_slow OR TTC <= 2.5s
    WARNING --> EMERGENCY_STOP : Rapid hazard escalation (d <= d_stop OR TTC <= 1.0s)
    WARNING --> NORMAL : Hazard clears beyond warning boundary

    SLOWDOWN --> EMERGENCY_STOP : Critical hazard breach (d <= d_stop OR TTC <= 1.0s)
    SLOWDOWN --> WARNING : Distance opens beyond slowdown boundary + hysteresis
    SLOWDOWN --> NORMAL : Corridor completely cleared

    EMERGENCY_STOP --> EMERGENCY_STOP : Hazard remains within recovery zone
    EMERGENCY_STOP --> SLOWDOWN : Sustained clear for >= 1.0s & d > d_stop + 0.5m
    EMERGENCY_STOP --> NORMAL : Corridor fully clear for >= 1.0s
```

---

## 5. Linux Device Interface & System Abstraction

```
+------------------------------------------------------------------------+
|                      USER-SPACE APPLICATION LAYER                      |
|                                                                        |
|   +-----------------------+           +----------------------------+   |
|   | Collision Risk Engine | <-------- | Safety State Machine (FSM) |   |
|   +-----------------------+           +----------------------------+   |
|               ^                                                        |
|               |  std::vector<SensorObservation>                        |
|   +-----------------------+                                            |
|   | LinuxSensorInterface  |                                            |
|   +-----------------------+                                            |
+---------------|--------------------------------------------------------+
| POSIX API     |  dev_read() / dev_write() / dev_ioctl()                |
+---------------v--------------------------------------------------------+
|                   VIRTUAL CHARACTER DEVICE DRIVER                      |
|                           (/dev/agv_radar)                             |
|                                                                        |
|   +----------------------------------------------------------------+   |
|   |  Hardware Packet Ring Buffer (Mutex Protected, Max 1024 pkts)  |   |
|   |  - Magic: 0x52414452 ('RADR')                                  |   |
|   |  - 64-bit Timestamp (us)                                       |   |
|   |  - 32-bit Distance (mm), 16-bit Azimuth (mrad), Relative V     |   |
|   |  - 8-bit Detection Flag & XOR Checksum Verification            |   |
|   +----------------------------------------------------------------+   |
+------------------------------------------------------------------------+
```
