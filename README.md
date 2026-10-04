# 🐼 Panda-Bot

**ROS 2-powered autonomous restaurant delivery robot for indoor mobile robotics.**

Panda-Bot is a cylindrical differential-drive robot developed as a complete mobile robotics platform, covering **robot modeling, control, odometry, sensor processing, state estimation, SLAM, localization, and autonomous navigation**.

<p align="center">
  <img src="media/panda_cad.png" width="45%">
  <img src="media/panda_simulation_1.png" width="45%">
</p>

---

## 🚀 Overview

Panda-Bot is designed as a foundation for autonomous restaurant delivery.

### Current Capabilities

* ROS 2 mobile robot architecture
* CAD-based URDF/Xacro model
* `ros2_control` integration
* Differential-drive control and kinematics
* Wheel odometry and TF2
* Joystick teleoperation
* LiDAR, IMU, and RGB camera simulation
* Gazebo simulation and RViz2 visualization
* Noisy odometry
* Kalman Filter and `robot_localization` EKF
* Probabilistic odometry motion model
* LiDAR self-filtering and range filtering
* LiDAR-based safety stop
* SLAM Toolbox mapping
* Occupancy-grid map generation
* AMCL global localization
* Nav2 navigation framework

---

# 🧠 System Architecture

```text
                         Panda-Bot
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
   Description            Control              Sensors
   URDF / Xacro        ros2_control        LiDAR / IMU / Camera
        │                    │                    │
        └────────────────────┼────────────────────┘
                             ▼
                      State Estimation
                       EKF / Odometry
                             │
                ┌────────────┴────────────┐
                │                         │
                ▼                         ▼
             Mapping                Localization
          SLAM Toolbox                  AMCL
                │                         │
                ▼                         ▼
            Saved Map                map → odom
                │                         │
                └────────────┬────────────┘
                             ▼
                            Nav2
                             │
                             ▼
                   Autonomous Navigation
```

### Mapping

```text
LiDAR + Odometry
       ↓
SLAM Toolbox
       ↓
Occupancy Map
       ↓
Saved Map
```

### Localization & Navigation

```text
Saved Map
   ↓
Map Server
   ↓
AMCL
   ↓
map → odom
   ↓
Nav2
   ↓
cmd_vel
   ↓
Robot
```

> SLAM Toolbox and AMCL are used in different operating modes. Only one should provide the `map → odom` transform at a time.

---

# 📐 Differential Drive

| Parameter        |     Value |
| ---------------- | --------: |
| Wheel Radius     | `0.065 m` |
| Wheel Separation |  `0.37 m` |
| Robot Diameter   |  `0.50 m` |

### Forward Kinematics

```text
v = r/2 · (φR + φL)
ω = r/L · (φR - φL)
```

### Inverse Kinematics

```text
φR = v/r + Lω/(2r)
φL = v/r - Lω/(2r)
```

Detailed derivation:

```text
media/DDR_kinematics.pdf
```

---

# ⚙️ Control & Odometry

Panda-Bot provides both a standard ROS 2 differential-drive controller and a custom controller exposing the underlying DDR mathematics.

```text
                    Twist
                      ↓
              Inverse Kinematics
                      ↓
                Wheel Velocity
                      ↓
              Wheel Joint Control
                      ↓
                 Joint States
                      ↓
              Forward Kinematics
                      ↓
                Odometry + TF
```

The custom controller integrates the robot pose using the midpoint orientation:

```text
θmid = θ + Δθ/2

x += Δs · cos(θmid)
y += Δs · sin(θmid)
```

Launch:

```bash
ros2 launch panda_controller controller.launch.py
```

---

# 📊 State Estimation

The project includes both educational and practical state-estimation implementations.

### EKF

`robot_localization` fuses wheel odometry and IMU measurements in planar mode.

```text
Wheel Odometry ──┐
                 ├──► EKF ──► Robot State
IMU ─────────────┘
```

Launch:

```bash
ros2 launch panda_localization local_localization.launch.py
```

### Probabilistic Motion Model

Robot motion is decomposed into:

```text
Δrot1 → Δtrans → Δrot2
```

with motion-dependent noise parameters:

```text
α1  α2  α3  α4
```

The model generates pose samples for particle-filter-based localization.

```text
Odometry
   ↓
Motion Decomposition
   ↓
Noise Model
   ↓
Random Sampling
   ↓
Pose Samples
```

---

# 📡 LiDAR Processing

Panda-Bot uses a simulated 2D LiDAR.

| Parameter |           Value |
| --------- | --------------: |
| Samples   |           `360` |
| Rate      |          `5 Hz` |
| Range     | `0.12 – 12.0 m` |

### Processing Pipeline

```text
/scan
  ↓
Self Filter
  ↓
/scan_self_filtered
  ↓
Range Filter
  ↓
/scan_filtered
  ├──► Safety Stop
  ├──► SLAM
  ├──► AMCL
  └──► Nav2
```

The self-filter removes measurements generated by the robot's own cylindrical body using its actual geometry.

```text
Robot Radius = 0.25 m
LiDAR X      = 0.175 m
LiDAR Y      = 0.0 m
```

### Safety Stop

```text
FREE       > 1.2 m
WARNING    ≤ 1.2 m
DANGER     ≤ 0.8 m
```

Topics:

```text
/safety_stop
/zones
```

---

# 🗺️ SLAM & Mapping

**SLAM Toolbox mapping is completed.**

```text
/scan_filtered
      +
Odometry
      ↓
SLAM Toolbox
      ↓
Occupancy Grid
      ↓
Saved Map
```

Maps are stored in:

```text
panda_mapping/maps/
├── small_house/
└── small_warehouse/
```

Launch:

```bash
ros2 launch panda_mapping slam.launch.py
```

---

# 📍 Global Localization

Panda-Bot uses **AMCL** for global localization on previously generated maps.

```text
Saved Map
    ↓
Map Server
    ↓
AMCL
 ↑  ↑
 │  └── Odometry
 └───── LiDAR
    ↓
Robot Pose
    ↓
map → odom
```

Launch:

```bash
ros2 launch panda_localization global_localization.launch.py
```

Configuration:

```text
panda_localization/
├── config/
│   ├── ekf.yaml
│   └── amcl.yaml
└── launch/
    ├── local_localization.launch.py
    └── global_localization.launch.py
```

---

# 🧭 Nav2

**Nav2 is the current navigation stage of the project.**

The navigation pipeline is:

```text
Goal Pose
    ↓
Nav2
    ↓
Planner
    ↓
Controller
    ↓
cmd_vel
    ↓
Panda Controller
    ↓
Wheels
```

Nav2 will handle:

* Global path planning
* Local control
* Global and local costmaps
* Obstacle avoidance
* Recovery behaviors
* Autonomous goal navigation

---

# 📷 Sensors

### LiDAR

```text
/scan
```

### Camera

```text
640 × 480
30 Hz
/camera/image
```

### IMU

```text
100 Hz
/imu/out
```

---

# 🌍 Simulation

Panda-Bot is developed and tested using:

* **ROS 2 Humble**
* **Gazebo Sim / Ignition Gazebo**
* **RViz2**
* **ros2_control**
* **TF2**
* **SLAM Toolbox**
* **Nav2**
* **Python / C++**

---

# 🎮 Teleoperation

```text
Joystick
   ↓
joy_node
   ↓
joy_teleop
   ↓
twist_mux
   ↓
Safety / Velocity Control
   ↓
Panda Controller
   ↓
Wheels
```

The joystick uses an **R1 deadman switch**.

```bash
ros2 launch panda_controller joystick_teleop.launch.py
```

---

# 📁 Workspace

```text
panda_ws/
└── src/
    ├── panda_description/
    ├── panda_controller/
    ├── panda_hardware/
    ├── panda_localization/
    ├── panda_mapping/
    └── panda_utils/
```

| Package              | Responsibility                   |
| -------------------- | -------------------------------- |
| `panda_description`  | URDF/Xacro, sensors, worlds      |
| `panda_controller`   | Control, odometry, teleoperation |
| `panda_hardware`     | Hardware interfaces              |
| `panda_localization` | EKF and AMCL                     |
| `panda_mapping`      | SLAM and map generation          |
| `panda_utils`        | LiDAR filtering and safety       |

---

# 🗺️ Roadmap

### Completed

* [x] Robot CAD & URDF/Xacro
* [x] Differential-drive control
* [x] Wheel odometry
* [x] TF2
* [x] `ros2_control`
* [x] Joystick teleoperation
* [x] Gazebo simulation
* [x] LiDAR / IMU / Camera simulation
* [x] Kalman Filter
* [x] EKF sensor fusion
* [x] Probabilistic odometry model
* [x] LiDAR filtering
* [x] Safety stop
* [x] SLAM Toolbox
* [x] Map generation
* [x] AMCL global localization

### In Progress

* [ ] Nav2 navigation
* [ ] Global & local costmaps
* [ ] Path planning
* [ ] Dynamic obstacle avoidance
* [ ] Autonomous navigation

### Future

* [ ] Physical hardware integration
* [ ] Computer vision
* [ ] Interactive touchscreen
* [ ] Restaurant delivery workflow
* [ ] Full autonomous delivery

---

# 📚 Technical Documentation

| Document                                     | Topic                         |
| -------------------------------------------- | ----------------------------- |
| `DDR_kinematics.pdf`                         | Differential-drive kinematics |
| `Probability.pdf`                            | Probability fundamentals      |
| `BayesRule and SensorFusion(KF and EKF).pdf` | Bayes, KF & EKF               |
| `OdometryMotionModel.pdf`                    | Probabilistic motion model    |

---

# 📸 Project Media

### CAD

<p align="center">
  <img src="media/panda_cad.png" width="75%">
</p>

### Gazebo

<p align="center">
  <img src="media/panda_simulation_1.png" width="75%">
</p>

<p align="center">
  <img src="media/panda_simulation_2.png" width="75%">
</p>

### RViz2

<p align="center">
  <img src="media/panda_visualization.png" width="75%">
</p>

---

# 👨‍💻 Author

**Ahmed Gaber**

Mechatronics Engineer | Robotics Software Engineer

`ROS 2` · `C++` · `Python` · `Robotics` · `Embedded Systems` · `Computer Vision`

---

# 📄 License

This project is developed for educational, portfolio, and robotics development purposes.

