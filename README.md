# 🐼 Panda-Bot

**Panda-Bot** is a ROS 2-powered autonomous restaurant delivery robot designed for indoor mobile robotics.

The project focuses on building a complete mobile robotics stack from **robot modeling and differential-drive control to odometry, sensor simulation, state estimation, probabilistic localization, and autonomous navigation**.

<p align="center">
  <img src="media/panda_cad.png" width="45%">
  <img src="media/panda_simulation_1.png" width="45%">
</p>

---

## 🚀 Overview

Panda-Bot is a cylindrical differential-drive mobile robot designed as a foundation for autonomous restaurant delivery.

### Current capabilities

* 🤖 ROS 2 mobile robot architecture
* 🧩 URDF/Xacro robot description
* 🛠️ CAD-based robot model
* ⚙️ `ros2_control` integration
* 🚗 Differential-drive control
* 📐 Forward & inverse DDR kinematics
* 🧭 Wheel-based odometry
* 🔄 TF2 transforms
* 🎮 Joystick teleoperation
* 📡 2D LiDAR simulation
* 📷 RGB camera simulation
* 🧭 IMU simulation
* 🌍 Gazebo physics simulation
* 👁️ RViz2 visualization
* 🌉 ROS 2 ↔ Gazebo communication
* 📊 Noisy odometry simulation
* 📈 Kalman Filter implementation
* 🧮 Extended Kalman Filter with `robot_localization`
* 🎲 Probabilistic odometry motion model
* 📍 Probabilistic pose sampling
* 🛡️ LiDAR-based safety stop
* 🧹 LiDAR self-filtering
* 🔎 Laser scan range filtering

The platform is being developed toward **SLAM, particle-filter localization, autonomous navigation, perception, and restaurant delivery**.

---

## 🧠 System Architecture

```text
                         ┌─────────────────────┐
                         │      Panda-Bot      │
                         │   Mobile Platform   │
                         └──────────┬──────────┘
                                    │
          ┌─────────────────────────┼─────────────────────────┐
          │                         │                         │
          ▼                         ▼                         ▼
   Robot Description          Control & Odometry          Sensors
          │                         │                         │
    URDF / Xacro              ros2_control                  LiDAR
    CAD Meshes                DDR Control                   Camera
    TF Frames                 Wheel Odometry                IMU
                                    │
                                    ▼
                             Noisy Odometry
                                    │
                     ┌──────────────┴──────────────┐
                     ▼                             ▼
              State Estimation              Motion Modeling
                     │                             │
                Kalman Filter              Δrot1 / Δtrans / Δrot2
                     │                             │
                     ▼                             ▼
                   EKF                     Probabilistic Samples
                     │                             │
                     └──────────────┬──────────────┘
                                    ▼
                              Localization
                                    │
                                    ▼
                         SLAM / Navigation
```

---

# 📐 Differential-Drive Kinematics

Panda-Bot uses a differential-drive configuration.

```text
Wheel Radius      = 0.065 m
Wheel Separation  = 0.37 m
```

### Forward Kinematics

```text
v = r/2 · (φR + φL)

ω = r/L · (φR - φL)
```

Where:

* `r` = wheel radius
* `L` = wheel separation
* `φR` = right wheel angular velocity
* `φL` = left wheel angular velocity
* `v` = linear velocity
* `ω` = angular velocity

### Inverse Kinematics

```text
φR = v/r + Lω/(2r)

φL = v/r - Lω/(2r)
```

A complete mathematical derivation is available in:

```text
media/DDR_kinematics.pdf
```

---

# ⚙️ ros2_control

Panda-Bot uses `ros2_control` to separate the robot control layer from the simulated hardware interface.

```text
                         ROS 2
                           │
                           ▼
                  ┌─────────────────┐
                  │ Controller      │
                  │ Manager         │
                  └────────┬────────┘
                           │
             ┌─────────────┴─────────────┐
             │                           │
             ▼                           ▼
      DiffDriveController       JointGroupVelocity
             │                           │
             └─────────────┬─────────────┘
                           ▼
                      Wheel Joints
                           │
                           ▼
                    Gazebo / Hardware
```

Two differential-drive control paths are available.

### Standard Controller

Uses:

```text
diff_drive_controller/DiffDriveController
```

Provides:

* `cmd_vel` processing
* Differential-drive kinematics
* Wheel velocity commands
* Odometry
* Odometry TF
* Velocity limiting

### Custom Controller

A Python implementation exposes the internal DDR mathematics:

```text
Twist
 │
 ▼
Inverse Kinematics
 │
 ▼
Wheel Velocities
 │
 ▼
JointGroupVelocityController
 │
 ▼
Wheel Joints
 │
 ▼
Joint States
 │
 ▼
Forward Kinematics
 │
 ▼
Odometry + TF
```

Launch:

```bash
ros2 launch panda_controller controller.launch.py
```

Custom controller:

```bash
ros2 launch panda_controller controller.launch.py \
  use_simple_controller:=true
```

Standard controller:

```bash
ros2 launch panda_controller controller.launch.py \
  use_simple_controller:=false
```

> The two controllers are alternative control paths and should not command the same wheel joints simultaneously.

---

# 🧭 Odometry

Wheel joint positions are converted into robot motion using differential-drive forward kinematics.

```text
Wheel Positions
      │
      ▼
ΔθR , ΔθL
      │
      ▼
Forward Kinematics
      │
      ├──────────────► Linear Velocity
      │
      ├──────────────► Angular Velocity
      │
      ▼
Δs , Δθ
      │
      ▼
x , y , θ
      │
      ├──────────────► /panda_controller/odom
      │
      └──────────────► odom → base_footprint
```

The custom controller uses the midpoint orientation when integrating the robot pose:

```text
θmid = θ + Δθ/2

x += Δs · cos(θmid)
y += Δs · sin(θmid)
```

A noisy odometry controller is also included to simulate realistic wheel encoder uncertainty.

```text
Joint States
     │
     ▼
Encoder Noise
     │
     ▼
Noisy Wheel Measurements
     │
     ▼
Differential-Drive Odometry
     │
     ▼
/panda_controller/odom_noisy
```

---

# 📊 State Estimation

## Kalman Filter

A basic Kalman Filter implementation is included for studying probabilistic state estimation.

```text
Prediction
    │
    ▼
Measurement Update
    │
    ▼
Filtered Estimate
```

The project also contains supporting material covering probability, Bayes' rule, Kalman filtering, and sensor fusion.

---

## 🧮 Extended Kalman Filter

`robot_localization` is used for practical sensor fusion.

The current configuration operates in planar mode:

```yaml
two_d_mode: true
```

The localization pipeline combines:

```text
Noisy Wheel Odometry
        │
        ▼
   EKF Prediction
        ▲
        │
     IMU Data
        │
        ▼
 Estimated Robot State
```

Launch:

```bash
ros2 launch panda_localization local_localization.launch.py
```

Main configuration:

```text
panda_localization/
├── config/
│   └── ekf.yaml
├── launch/
│   └── local_localization.launch.py
└── panda_localization/
    ├── imu_republisher.py
    ├── kalman_filter.py
    └── odometry_motion_model.py
```

---

# 🎲 Probabilistic Odometry Motion Model

Panda-Bot implements a probabilistic odometry motion model for the prediction stage of particle-filter-based localization.

The robot motion is decomposed into:

```text
Δrot1 → Δtrans → Δrot2
```

where:

* `Δrot1` = initial rotation
* `Δtrans` = translation
* `Δrot2` = final rotation

Odometry uncertainty is modeled using:

```text
α1
α2
α3
α4
```

These parameters represent motion-dependent noise caused by factors such as:

* Encoder uncertainty
* Wheel slip
* Wheel radius errors
* Wheel separation errors
* Mechanical inaccuracies
* Accumulated odometry drift

The model generates multiple possible poses:

```text
                  Odometry
                     │
                     ▼
             Motion Decomposition
                     │
           ┌─────────┼─────────┐
           ▼         ▼         ▼
        Δrot1      Δtrans    Δrot2
           │         │         │
           └─────────┼─────────┘
                     ▼
                Noise Model
                     │
                     ▼
              Random Sampling
                     │
                     ▼
          ● ● ● ● ● ● ● ●
        ● ● ● ● ● ● ● ● ●
          ● ● ● ● ● ● ● ●
                     │
                     ▼
                Pose Samples
```

The generated samples are published on:

```text
/odometry_motion_model/samples
```

Default number of samples:

```text
300
```

Mathematical documentation:

```text
media/OdometryMotionModel.pdf
```

---

# 📡 LiDAR & Sensor Processing

Panda-Bot uses a simulated 2D GPU LiDAR.

### LiDAR configuration

```text
Samples:       360
Configured Rate: 10 Hz
Range:         0.12 – 12.0 m
Noise:         Gaussian
```

Raw topic:

```text
/scan
```

The LiDAR processing pipeline is:

```text
                         /scan
                           │
                           ▼
                 ┌───────────────────┐
                 │  Scan Self Filter │
                 │  Robot Geometry   │
                 └─────────┬─────────┘
                           │
                           ▼
                  /scan_self_filtered
                           │
                           ▼
                 ┌───────────────────┐
                 │   laser_filters   │
                 │    Range Filter   │
                 └─────────┬─────────┘
                           │
                           ▼
                    /scan_filtered
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
         Safety Stop      SLAM         Nav2
```

### LiDAR Self-Filtering

`scan_self_filter.py` removes measurements caused by the robot's own cylindrical body.

The filter uses the actual robot geometry:

```text
Robot Radius = 0.25 m
LiDAR X      = 0.175 m
LiDAR Y      = 0.0 m
```

Instead of removing a fixed angular region, the expected intersection between each laser ray and the robot surface is calculated geometrically.

A measurement is removed only when it matches the calculated robot surface within a small tolerance.

This preserves nearby external obstacles, including obstacles very close to the robot.

### Safety Stop

The `safety_stop` node analyzes `/scan_filtered` and provides three states:

```text
FREE
  │
  │ distance > 1.2 m
  ▼
WARNING
  │
  │ distance ≤ 0.8 m
  ▼
DANGER
```

Thresholds:

```text
Warning = 1.2 m
Danger  = 0.8 m
```

The node publishes:

```text
/safety_stop
/zones
```

and integrates with joystick speed control through `twist_mux`.

---

# 📷 Camera

The simulated RGB camera provides:

```text
Resolution:      640 × 480
Update Rate:     30 Hz
Horizontal FOV:  ~60°
```

Topic:

```text
/camera/image
```

The camera provides the foundation for future computer-vision perception.

---

# 🧭 IMU

The simulated IMU operates at:

```text
Update Rate: 100 Hz
```

Topic:

```text
/imu/out
```

The IMU is used as an additional source of motion information for state estimation and sensor fusion.

---

# 🌍 Gazebo Simulation

Panda-Bot uses **Gazebo Sim / Ignition Gazebo** for physics-based simulation.

The simulation includes:

* Robot spawning
* Custom environments
* Wheel friction
* Physics parameters
* `ros2_control`
* LiDAR
* RGB camera
* IMU
* ROS 2 ↔ Gazebo bridges
* Simulation time

Launch:

```bash
ros2 launch panda_description gazebo.launch.py
```

Specific world:

```bash
ros2 launch panda_description gazebo.launch.py \
  world_name:=small_house
```

---

# 👁️ RViz2

RViz2 is used for:

* Robot visualization
* TF inspection
* LiDAR visualization
* Sensor frames
* Robot pose
* Odometry
* Localization
* Probabilistic pose samples
* Safety zones

Launch:

```bash
ros2 launch panda_description display.launch.py
```

---

# 🎮 Joystick Teleoperation

The robot supports joystick teleoperation through:

```text
Joystick
   │
   ▼
joy_node
   │
   ▼
joy_teleop
   │
   ▼
input_joy/cmd_vel_stamped
   │
   ▼
twist_mux
   │
   ├──────────────► Safety Lock
   │
   ▼
panda_controller/cmd_vel_unstamped
   │
   ▼
Twist Relay
   │
   ▼
panda_controller/cmd_vel
```

The joystick uses an **R1 deadman switch** to enable motion commands.

Launch:

```bash
ros2 launch panda_controller joystick_teleop.launch.py
```

The architecture also allows higher-priority safety commands to override joystick velocity commands.

---

# 🌉 ROS 2 ↔ Gazebo

Gazebo sensor and simulation data are connected to ROS 2 through the Gazebo ROS bridge.

Main interfaces include:

```text
/clock
/scan
/camera/image
/imu/out
```

This allows the same ROS 2 nodes to process simulated sensor data similarly to real hardware.

---

# 📁 Repository Structure

```text
panda_ws/
├── media/
│   ├── DDR_kinematics.pdf
│   ├── OdometryMotionModel.pdf
│   ├── Probability.pdf
│   ├── BayesRule and SensorFusion(KF and EKF).pdf
│   ├── panda_cad.png
│   ├── panda_simulation_1.png
│   ├── panda_simulation_2.png
│   └── panda_visualization.png
│
├── src/
│   ├── panda_controller/
│   │   ├── config/
│   │   │   ├── joy_config.yaml
│   │   │   ├── joy_teleop.yaml
│   │   │   ├── panda_controllers.yaml
│   │   │   ├── twist_mux_joy.yaml
│   │   │   ├── twist_mux_locks.yaml
│   │   │   └── twist_mux_topics.yaml
│   │   ├── launch/
│   │   │   ├── controller.launch.py
│   │   │   └── joystick_teleop.launch.py
│   │   └── panda_controller/
│   │       ├── simple_controller.py
│   │       ├── noisy_controller.py
│   │       └── twist_relay.py
│   │
│   ├── panda_description/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── meshes/
│   │   ├── rviz/
│   │   ├── urdf/
│   │   └── worlds/
│   │
│   ├── panda_hardware/
│   │
│   ├── panda_localization/
│   │   ├── config/
│   │   │   ├── ekf.yaml
│   │   │   └── odometry_motion_model.rviz
│   │   ├── launch/
│   │   │   └── local_localization.launch.py
│   │   └── panda_localization/
│   │       ├── imu_republisher.py
│   │       ├── kalman_filter.py
│   │       └── odometry_motion_model.py
│   │
│   └── panda_utils/
│       ├── config/
│       │   └── laser_filter.yaml
│       ├── launch/
│       │   └── laser_filter.launch.py
│       └── panda_utils/
│           ├── safety_stop.py
│           └── scan_self_filter.py
│
├── .gitignore
└── README.md
```

---

# 🗺️ Roadmap

## Completed

* [x] Robot CAD model
* [x] URDF/Xacro robot description
* [x] TF frame structure
* [x] Differential-drive model
* [x] DDR forward kinematics
* [x] DDR inverse kinematics
* [x] `ros2_control` integration
* [x] Standard `DiffDriveController`
* [x] Custom differential-drive controller
* [x] Wheel-based odometry
* [x] Noisy odometry simulation
* [x] `odom → base_footprint` TF
* [x] Joystick teleoperation
* [x] Gazebo simulation
* [x] RViz2 visualization
* [x] LiDAR simulation
* [x] Camera simulation
* [x] IMU simulation
* [x] ROS 2 ↔ Gazebo communication
* [x] Basic Kalman Filter
* [x] `robot_localization` EKF
* [x] Probabilistic odometry motion model
* [x] Probabilistic pose sampling
* [x] LiDAR self-filtering
* [x] Laser scan range filtering
* [x] Safety stop

## In Progress

* [ ] Particle-filter localization
* [ ] LiDAR measurement model
* [ ] Particle weighting
* [ ] Particle resampling
* [ ] SLAM
* [ ] Autonomous navigation
* [ ] Dynamic obstacle avoidance

## Future

* [ ] Physical hardware integration
* [ ] Interactive touchscreen application
* [ ] Computer vision perception
* [ ] Restaurant delivery workflow
* [ ] Full autonomous delivery system

---

# 📚 Technical Documentation

The `media/` directory contains mathematical and technical notes developed alongside the project.

| Document                                     | Topic                               |
| -------------------------------------------- | ----------------------------------- |
| `DDR_kinematics.pdf`                         | Differential-drive kinematics       |
| `Probability.pdf`                            | Probability fundamentals            |
| `BayesRule and SensorFusion(KF and EKF).pdf` | Bayes, Kalman Filter & EKF          |
| `OdometryMotionModel.pdf`                    | Probabilistic odometry motion model |

---

# 📸 Project Media

### CAD

<p align="center">
  <img src="media/panda_cad.png" width="75%">
</p>

### Gazebo Simulation

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

`ROS 2` · `C++` · `Python` · `Robotics` · `Embedded Systems` · `Computer Vision` · `Autonomous Robots`

---

# 📄 License

This project is developed for educational, portfolio, and robotics development purposes.

