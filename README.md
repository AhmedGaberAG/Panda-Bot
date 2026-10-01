# 🐼 Panda-Bot

**Panda-Bot** is a ROS 2-powered autonomous restaurant delivery robot designed for indoor food delivery applications.

The project combines **robot modeling, CAD integration, physics simulation, ros2_control, differential-drive kinematics, sensor simulation, visualization, and joystick teleoperation** into a mobile robotic platform.

<p align="center">
  <img src="media/panda_cad.png" width="45%">
  <img src="media/panda_simulation_1.png" width="45%">
</p>

---

## 🚀 Overview

Panda-Bot is designed as an indoor mobile robot capable of transporting food and interacting with its environment.

The current platform includes:

* 🤖 ROS 2 robot architecture
* 🧩 URDF/Xacro robot description
* ⚙️ ros2_control integration
* 🚗 Differential-drive control
* 📐 Forward & inverse DDR kinematics
* 🧭 Wheel-based odometry
* 🔄 TF2 frame broadcasting
* 🎮 Joystick teleoperation
* 📡 LiDAR simulation
* 📷 RGB camera simulation
* 🧭 IMU simulation
* 🌍 Gazebo physics simulation
* 👁️ RViz2 visualization
* 🌉 ROS 2 ↔ Gazebo communication

The project is structured as a foundation for future **SLAM, localization, autonomous navigation, perception, and restaurant delivery applications**.

---

## 🧠 System Architecture

```text
                         ┌──────────────────────┐
                         │      Panda-Bot       │
                         │   Mobile Platform    │
                         └──────────┬───────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              │                     │                     │
        Robot Description       Control System        Simulation
              │                     │                     │
       ┌──────┴──────┐       ┌──────┴──────┐       ┌──────┴──────┐
       │ URDF/Xacro  │       │ ros2_control │       │ Gazebo Sim  │
       │ CAD Meshes  │       │ Controllers  │       │ Physics     │
       │ TF Frames   │       │ Hardware     │       │ Sensors     │
       └─────────────┘       └──────┬───────┘       └─────────────┘
                                    │
                         ┌──────────┴──────────┐
                         │ Differential Drive  │
                         │                    │
                         │ cmd_vel             │
                         │      ↓              │
                         │ Wheel Velocities    │
                         │      ↓              │
                         │ Joint States        │
                         │      ↓              │
                         │ Odometry + TF       │
                         └──────────┬──────────┘
                                    │
                         ┌──────────┴──────────┐
                         │ Sensors / ROS 2     │
                         │ LiDAR • Camera • IMU│
                         └─────────────────────┘
```

---

## 📐 Differential-Drive Kinematics

The Panda-Bot uses a two-wheel differential-drive configuration with:

```text
Wheel Radius       = 0.065 m
Wheel Separation   = 0.37 m
```

### Forward Kinematics

Wheel angular velocities are converted into robot linear and angular velocity:

$$
v = \frac{r}{2}(\phi_R + \phi_L)
$$

$$
\omega = \frac{r}{L}(\phi_R - \phi_L)
$$

Where:

* `r` = wheel radius
* `L` = wheel separation
* `φ_R` = right wheel angular velocity
* `φ_L` = left wheel angular velocity
* `v` = robot linear velocity
* `ω` = robot angular velocity

### Inverse Kinematics

Robot velocity commands are converted into wheel angular velocities:

$$
\phi_R = \frac{v}{r} + \frac{L\omega}{2r}
$$

$$
\phi_L = \frac{v}{r} - \frac{L\omega}{2r}
$$

The complete mathematical derivation is documented in:

**`media/DDR_kinematics.pdf`**

---

## ⚙️ ros2_control

The robot uses `ros2_control` to separate the control layer from the robot hardware/simulation interface.

```text
                 ROS 2
                   │
                   ▼
          ┌─────────────────┐
          │ Controller      │
          │ Manager         │
          └────────┬────────┘
                   │
          ┌────────┴────────┐
          │                 │
          ▼                 ▼
   panda_controller   simple_velocity_controller
   DiffDriveController    JointGroupVelocity
          │                 │
          └────────┬────────┘
                   ▼
             Wheel Joints
                   │
                   ▼
          ros2_control Hardware
                   │
                   ▼
             Gazebo / Robot
```

The robot defines velocity command interfaces and position/velocity state interfaces for:

```text
wheel_right_joint
wheel_left_joint
```

The simulation currently uses:

```text
ign_ros2_control/IgnitionSystem
```

---

## 🎛️ Differential-Drive Controllers

The project contains two control approaches.

### Standard Controller

The production-oriented approach uses:

```text
diff_drive_controller/DiffDriveController
```

It provides:

* `cmd_vel` processing
* Differential-drive kinematics
* Wheel velocity commands
* Odometry
* Odometry TF
* Velocity limiting
* Wheel data publishing

### Custom Controller

A custom Python controller is also included for understanding the internal mathematics of differential-drive control.

```text
TwistStamped
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

The custom controller implements:

* Inverse DDR kinematics
* Forward DDR kinematics
* Wheel velocity estimation
* Differential-drive odometry
* `odom → base_footprint` TF broadcasting

The controller can be selected at launch time:

```bash
ros2 launch panda_controller controller.launch.py
```

Custom controller:

```bash
ros2 launch panda_controller controller.launch.py \
  use_simple_controller:=true
```

Standard `DiffDriveController`:

```bash
ros2 launch panda_controller controller.launch.py \
  use_simple_controller:=false
```

> The two controllers are alternative control paths and should not control the same wheel joints simultaneously.

---

## 🧭 Odometry & TF

The custom controller calculates the robot pose from wheel displacement.

```text
Wheel Positions
      │
      ▼
ΔθR , ΔθL
      │
      ▼
Forward Kinematics
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

The odometry message contains:

* Position
* Orientation
* Linear velocity
* Angular velocity

The TF transform provides the spatial relationship:

```text
odom
  │
  ▼
base_footprint
  │
  ▼
base_link
  ├── wheel_right_link
  ├── wheel_left_link
  ├── laser_link
  ├── camera_link
  └── imu_link
```

---

## 🎮 Joystick Teleoperation

The robot supports joystick control through:

```text
joy
  │
  ▼
joy_teleop
  │
  ▼
TwistStamped
  │
  ▼
panda_controller/cmd_vel
  │
  ▼
Differential-Drive Controller
```

Launch joystick teleoperation with:

```bash
ros2 launch panda_controller joystick_teleop.launch.py
```

The current configuration uses:

* Right analog stick → linear velocity
* Left analog stick → angular velocity
* R1 → deadman switch
* 20 Hz joystick autorepeat

---

## 🤖 Robot Description

The Panda-Bot model is built using **URDF/Xacro** and includes:

* Differential-drive wheels
* Four caster wheels
* LiDAR
* RGB camera
* IMU
* Camera optical frame
* `base_footprint`
* `base_link`
* Wheel and sensor TF frames
* CAD-derived STL meshes
* Gazebo simulation properties
* ros2_control interfaces

Main Xacro files:

```text
panda.urdf.xacro
properties.xacro
gazebo.xacro
ros2_control.xacro
```

---

## 📡 Sensor Simulation

### LiDAR

A GPU-based 2D LiDAR is simulated for indoor perception.

```text
Samples:       360
Update Rate:   5 Hz
Range:         0.12 – 12.0 m
Noise:         Gaussian
```

ROS 2 topic:

```text
/scan
```

### Camera

The robot includes a simulated RGB camera.

```text
Resolution:    640 × 480
Update Rate:   30 Hz
Horizontal FOV: ~60°
```

ROS 2 topic:

```text
/camera/image
```

### IMU

The simulated IMU operates at:

```text
100 Hz
```

ROS 2 topic:

```text
/imu/out
```

---

## 🌍 Gazebo Simulation

Panda-Bot uses **Gazebo Sim** for physics-based simulation.

The simulation includes:

* Robot spawning
* Custom worlds
* Wheel friction
* Physics parameters
* ros2_control
* LiDAR
* Camera
* IMU
* ROS 2 ↔ Gazebo bridges
* Simulation time

Launch the default simulation:

```bash
ros2 launch panda_description gazebo.launch.py
```

Launch a specific world:

```bash
ros2 launch panda_description gazebo.launch.py \
  world_name:=small_house
```

---

## 👁️ RViz2 Visualization

RViz2 is configured for:

* Robot model visualization
* TF inspection
* Sensor frames
* Laser scan visualization
* Navigation visualization
* Robot pose
* Initial pose
* Coordinate frames

Launch RViz2:

```bash
ros2 launch panda_description display.launch.py
```

---

## 🌉 ROS 2 ↔ Gazebo Bridge

`ros_gz_bridge` is used to exchange simulation data between Gazebo and ROS 2.

Current interfaces include:

```text
/clock
/scan
/camera/image
/imu/out
```

This allows ROS 2 nodes to consume simulated sensor data using standard ROS 2 message types.

---

## 📁 Repository Structure

```text
panda_ws/
├── media/
│   ├── DDR_kinematics.pdf
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
│   │   │   └── panda_controllers.yaml
│   │   ├── launch/
│   │   │   ├── controller.launch.py
│   │   │   └── joystick_teleop.launch.py
│   │   └── panda_controller/
│   │       └── simple_controller.py
│   │
│   ├── panda_description/
│   │   ├── config/
│   │   │   └── gz_bridge.yaml
│   │   ├── launch/
│   │   │   ├── display.launch.py
│   │   │   └── gazebo.launch.py
│   │   ├── meshes/
│   │   ├── rviz/
│   │   ├── urdf/
│   │   │   ├── panda.urdf.xacro
│   │   │   ├── properties.xacro
│   │   │   ├── gazebo.xacro
│   │   │   └── ros2_control.xacro
│   │   └── worlds/
│   │
│   └── panda_hardware/
│
├── .gitignore
└── README.md
```

---

## 🗺️ Project Roadmap

* [x] Robot CAD model
* [x] URDF/Xacro robot description
* [x] TF frame structure
* [x] Differential-drive model
* [x] DDR forward kinematics
* [x] DDR inverse kinematics
* [x] ros2_control integration
* [x] Standard DiffDriveController configuration
* [x] Custom differential-drive controller
* [x] Wheel-based odometry
* [x] `odom → base_footprint` TF
* [x] Joystick teleoperation
* [x] Gazebo simulation
* [x] RViz2 visualization
* [x] LiDAR simulation
* [x] Camera simulation
* [x] IMU simulation
* [x] ROS 2 ↔ Gazebo bridge
* [ ] Physical hardware integration
* [ ] SLAM
* [ ] Localization
* [ ] Autonomous navigation
* [ ] Obstacle avoidance
* [ ] Interactive touchscreen application
* [ ] Restaurant delivery workflow
* [ ] Full autonomous delivery system

---

## 📸 Project Media

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

### DDR Kinematics

The mathematical derivation of the differential-drive model is available in:

```text
media/DDR_kinematics.pdf
```

It covers:

* Forward kinematics
* Inverse kinematics
* Linear velocity
* Angular velocity
* Wheel velocity relationships
* Differential-drive motion equations

---

## 👨‍💻 Author

**Ahmed Gaber**

Mechatronics Engineer | Robotics Software Engineer

`ROS 2` · `C++` · `Python` · `Robotics` · `Embedded Systems` · `Computer Vision` · `Autonomous Robots`

---

## 📄 License

This project is currently intended for educational, portfolio, and robotics development purposes.

