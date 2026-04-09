# FCM Digital Twin & Supervisory Control

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-34ce57?logo=ros)
![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

This repository contains the core software stack for the **Information Technology of Supervisory Control Based on a Digital Twin**. It is designed specifically for the reconnaissance and extraction of Lava-like Fuel-Containing Materials (LFCM) in hazardous environments, such as the "Shelter" object (Chernobyl NPP).

This package implements a **Shared Autonomy (Edge-Fog-Cloud)** architecture, bridging the gap between high-level operator dispatching (Digital Twin) and reactive, onboard safety behaviors.

## 🌟 Core Scientific Contributions

- **Digital Twin Integration:** Full integration with Foxglove Studio for predictive planning, semantic SDF-parsing, and situational awareness in unstructured environments.
- **ALARA Speed Reflex (C++):** A hardware-accelerated node that dynamically modulates the robot's kinematic limits (`/speed_limit`) based on real-time radiation gradients, minimizing the accumulated dose.
- **Hierarchical Mission Control:** A robust Python-based dispatcher implementing a Finite State Machine (FSM). It handles macro-commands (Transit, Sampling) and complex survival protocols.
- **Network Resilience:** An asynchronous Heartbeat Monitor capable of detecting telemetry loss, safely preempting current tasks, and autonomously executing an evacuation route (Return to Base).
- **Shadow Mode Teleoperation:** Predictive visualization tools to counteract transport delays during teleoperation in "radio-blind" zones.

## 🧩 Key Nodes & Modules

### 1. `mission_control.py` (The Supervisor)
The main dispatcher for the robot. It listens to the Digital Twin's macro-commands and translates them into Nav2 action goals (`MapsToPose`, `MapsThroughPoses`). Includes the **Two-Stage Survival Protocol** for autonomous evacuation upon connection loss.

### 2. `alara_speed_reflex` (Safety Loop)
Written in C++ for maximum efficiency. Subscribes to the `/radiation_map` and `/odom`. When the robot enters a high-radiation zone (>30%), it bypasses global planners and instantly boosts speed to 100% to reduce exposure time, restoring normal limits upon exiting the zone.

### 3. `radiation_field_server.py` (Simulation)
Generates dynamic, gradient-based radiation fields for SITL testing. Supports loading customizable radiation maps via JSON configuration files.

### 4. `stabilized_frame_publisher.py` (2.5D Perception)
Corrects the 3D LiDAR point cloud projection based on IMU data (Roll/Pitch), preventing the generation of "phantom obstacles" on the `Local Costmap` when the robot traverses extreme debris or concrete spills.

## 🏗️ System Dependencies

This package acts as the "Brain" and requires the following "Body" and "Nervous System" repositories to function:
- **`linorobot2` (Custom Fork):** Provides the URDF, EKF sensor fusion, and micro-ROS agent.
- **`tankbot_esp32`:** Custom firmware for the ESP32-S3 microcontroller handling Skid-Steer kinematics.
- **ROS 2 Nav2 & Slam_Toolbox:** Core navigation stack.

## 🚀 Quick Start (SITL Simulation)

Ensure your workspace is built and sourced:
```bash
cd ~/ros2_ws
colcon build --packages-select fcm_digital_twin
source install/setup.bash

Launch the complete Digital Twin simulation (Gazebo + Nav2 + Mission Control + Radiation Server):
Bash

ros2 launch fcm_digital_twin master_sim.launch.py world_file:=shelter_zero.sdf use_3d_lidar:=true

Foxglove Studio Setup

    Open Foxglove Studio.

    Connect via Foxglove WebSocket to ws://localhost:8765.

    Import the pre-configured layout from config/foxglove_layout.json to access the Mission Control button panel and the 3D Digital Twin view.

🔬 Academic Reference

This repository is part of a PhD dissertation focused on robotic LFCM extraction at the Chernobyl "Shelter" object. If you use this code for academic purposes, please cite the underlying methodology regarding the Edge-Fog-Cloud continuum and ALARA integration.