# 🐼 Panda-Bot

**Panda-Bot** is a ROS 2-powered autonomous restaurant delivery robot designed for indoor food delivery applications.

The project combines **robot description, simulation, multi-sensor perception, navigation, and interactive interfaces** into a mobile robotic platform designed to operate in restaurant environments.

<p align="center">
  <img src="media/panda_cad.png" width="45%">
  <img src="media/panda_simulation_1.png" width="45%">
</p>

---

## 🚀 Overview

Panda-Bot is designed as an indoor autonomous mobile robot capable of transporting food and interacting with its environment.

The platform integrates:

* 🤖 ROS 2 robot architecture
* 🗺️ SLAM and autonomous navigation
* 📡 LiDAR-based perception
* 📷 Camera perception
* 🧭 IMU sensing
* 🖥️ Interactive touchscreen interface
* 🌐 ROS 2 ↔ Gazebo communication
* 🏠 Indoor restaurant simulation environments

The current repository focuses on the robot's **description, simulation, visualization, and sensor integration**, providing the foundation for higher-level autonomous navigation and application software.

---

## 🧩 System Architecture

```text
                        ┌─────────────────────┐
                        │      Panda-Bot      │
                        │   Mobile Platform   │
                        └──────────┬──────────┘
                                   │
             ┌─────────────────────┼─────────────────────┐
             │                     │                     │
        Robot Model            Sensors              Simulation
             │                     │                     │
      ┌──────┴──────┐       ┌──────┼──────┐       ┌──────┴──────┐
      │ URDF/Xacro  │       │ LiDAR       │       │ Gazebo Sim   │
      │ TF2         │       │ Camera      │       │ Worlds       │
      │ Meshes      │       │ IMU         │       │ Physics      │
      └─────────────┘       └─────────────┘       └─────────────┘
                                   │
                                   ▼
                            ROS 2 Interfaces
                                   │
                         ┌─────────┴─────────┐
                         │ Navigation / SLAM │
                         │ Perception        │
                         │ Applications      │
                         └───────────────────┘
```

---

## 🛠️ Technologies

| Technology        | Purpose                                  |
| ----------------- | ---------------------------------------- |
| **ROS 2**         | Robot middleware and system architecture |
| **URDF / Xacro**  | Robot modeling and description           |
| **RViz2**         | Robot visualization and TF inspection    |
| **Gazebo Sim**    | Physics-based robot simulation           |
| **ros_gz_bridge** | ROS 2 ↔ Gazebo communication             |
| **LiDAR**         | 2D environment perception                |
| **Camera**        | Visual perception                        |
| **IMU**           | Orientation and inertial sensing         |
| **STL Meshes**    | CAD-based robot visualization            |
| **Python**        | ROS 2 launch and integration             |

---

## 📁 Repository Structure

```text
panda_ws/
├── media/
│   ├── panda_cad.png
│   ├── panda_simulation_1.png
│   ├── panda_simulation_2.png
│   └── panda_visualization.png
│
├── src/
│   ├── panda_description/
│   │   ├── config/
│   │   │   └── gz_bridge.yaml
│   │   ├── launch/
│   │   │   ├── display.launch.py
│   │   │   └── gazebo.launch.py
│   │   ├── meshes/
│   │   ├── models/
│   │   ├── photos/
│   │   ├── rviz/
│   │   │   └── panda.rviz
│   │   ├── urdf/
│   │   │   ├── panda.urdf.xacro
│   │   │   ├── properties.xacro
│   │   │   └── gazebo.xacro
│   │   └── worlds/
│   │
│   └── panda_hardware/
│
├── .gitignore
└── README.md
```

---

## 🤖 Robot Description

The Panda-Bot model is built using **URDF/Xacro** and includes:

* Differential-drive wheels
* Front and rear caster wheels
* LiDAR
* RGB camera
* IMU
* Camera optical frame
* `base_footprint` and `base_link`
* Sensor and wheel TF frames

The robot uses reusable Xacro properties for dimensions such as wheel radius, wheel separation, and base height.

---

## 📡 Sensor Simulation

### LiDAR

A 2D GPU LiDAR is simulated for indoor environment perception.

```text
Samples:      360
Update rate:  5 Hz
Range:        0.12 – 12.0 m
Noise:        Gaussian
```

The simulated scan is bridged to ROS 2 through:

```text
/scan
```

### Camera

The robot includes a simulated RGB camera:

```text
Resolution:   640 × 480
Update rate:  30 Hz
Horizontal FOV: ~60°
```

ROS 2 topic:

```text
/camera/image
```

### IMU

The robot includes a simulated IMU operating at:

```text
100 Hz
```

ROS 2 topic:

```text
/imu/out
```

The sensor configuration and ROS–Gazebo topic mappings are defined inside the description package.

---

## 🌍 Gazebo Simulation

Panda-Bot uses **Gazebo Sim** for physics-based simulation.

The Gazebo launch system supports:

* Custom world selection
* Robot spawning from `/robot_description`
* Gazebo resource paths
* Sensor simulation
* ROS 2 ↔ Gazebo bridging
* Simulation time

Example:

```bash
ros2 launch panda_description gazebo.launch.py
```

Launch a specific world:

```bash
ros2 launch panda_description gazebo.launch.py world_name:=small_house
```

The launch system automatically configures the Gazebo resource path and spawns the robot from the generated Xacro description.

---

## 👁️ RViz2 Visualization

RViz2 is configured to visualize:

* Robot model
* TF tree
* Wheels
* Sensors
* Robot frames
* Navigation goals
* Initial pose

Launch the visualization:

```bash
ros2 launch panda_description display.launch.py
```

The RViz configuration uses `base_footprint` as the fixed frame and loads the robot description from `/robot_description`.

---

## 🔄 ROS 2 ↔ Gazebo Bridge

The project uses `ros_gz_bridge` to exchange simulation data between ROS 2 and Gazebo.

Current bridged interfaces include:

```text
/clock
/scan
/camera/image
/imu/out
```

This allows ROS 2 nodes to consume simulated sensor data using standard ROS 2 message types.

---

## 🎯 Project Roadmap

* [x] Robot CAD model
* [x] URDF/Xacro robot description
* [x] TF frame structure
* [x] Differential-drive model
* [x] Gazebo simulation
* [x] RViz2 visualization
* [x] LiDAR simulation
* [x] Camera simulation
* [x] IMU simulation
* [x] ROS 2 ↔ Gazebo bridge
* [ ] Hardware integration
* [ ] SLAM
* [ ] Autonomous navigation
* [ ] Localization
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

### Simulation

<p align="center">
  <img src="media/panda_simulation_1.png" width="75%">
</p>

<p align="center">
  <img src="media/panda_simulation_2.png" width="75%">
</p>

### RViz2 Visualization

<p align="center">
  <img src="media/panda_visualization.png" width="75%">
</p>

---

## 👨‍💻 Author

**Ahmed Gaber**

Mechatronics Engineer | Robotics Software Engineer

Focused on:

`ROS 2` · `C++` · `Python` · `Robotics` · `Embedded Systems` · `Computer Vision` · `Autonomous Robots`

---

## 📄 License

This project is currently intended for educational, portfolio, and robotics development purposes.

