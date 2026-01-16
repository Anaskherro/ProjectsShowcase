# Portfolio: Robotics & Autonomous Systems Projects
### Anas KHERRO | Electromechanical & Robotics Engineer

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue?style=flat&logo=linkedin)](https://www.linkedin.com/in/anaskherro/)
[![Email](https://img.shields.io/badge/Email-Contact_Me-red?style=flat&logo=gmail)](mailto:anaskherro@gmail.com)

---

## 🚀 Professional Summary
I am a Robotics Engineer focused on bridging the gap between theoretical simulation and rugged, real-world application. My work emphasizes **robust control systems**, **state estimation**, and **autonomous navigation** in resource-constrained environments.

Below is a collection of my key projects, ranging from aerial platforms (UAVs) to ground vehicles (UGVs) and marine systems (USVs).

---

## 🛠 Skills Snapshot
* **Core Stack:** ROS / ROS2, C++, Python, MATLAB/Simulink
* **Flight Controllers:** Pixhawk, ArduPilot, PX4
* **Control & Planning:** MPC, PID, EKF, Hybrid A*, Pure Pursuit
* **Simulation:** Gazebo, CARLA, IsaacSim, Rviz

---

## 🚁 Aerial Robotics (UAVs)

### 1. Quadrotor Navigation in Obstacle-Dense Environments
**Focus:** Path Planning, Control Theory, Benchmarking
* **Github:** [Path Planning](https://github.com/anaskherro/quadrotor_path_plannig) | [Path Tracking](https://github.com/anaskherro/quadrotor_path_tracking)
* [Video](https://drive.google.com/file/d/1VvDDkERO4ItsNEfG7CMlqEyuqMEE_Cq_/view?usp=drive_link)

> **The Challenge:** Navigating a quadcopter through complex, cluttered environments requires precise global planning and reactive control.
> **The Solution:** Implemented a full autonomy stack using **ROS and Pixhawk**. I developed a **Hybrid A*** global planner for efficient trajectory generation and conducted a rigorous benchmark between **Model Predictive Control (MPC)** and **PID** controllers to analyze performance under dynamic constraints.

### 2. Nonlinear Model Predictive Control (NMPC) for Quadcopters
**Focus:** Advanced Control, Optimization
* **Github:** [Rotors MPC Controller](https://github.com/Anaskherro/rotors_mpc_controller)

Developed a high-performance **NMPC controller** capable of handling nonlinear dynamics and constraints. This project focuses on optimization-based control strategies to ensure stability during aggressive maneuvers.

---

## 🚗 Ground Robotics (UGVs) & Autonomous Driving

### 3. CARLA/ROS Autonomous Path Tracking
**Focus:** Computer Vision, Sensor Fusion, Simulation
* **Github:** [Carla Path Tracking](https://github.com/anaskherro/carla_path_tracking)

An autonomous driving system developed within the **CARLA Simulator**.
* **Control:** Implemented a **Pure Pursuit** algorithm for geometric path tracking.
* **Perception:** Integrated a computer vision pipeline for real-time **Traffic Sign Detection**, closing the loop between perception and actuation.

### 4. REVER: Outdoor Autonomous Rover
**Focus:** Hardware and sensors Integration, System Design and Tuning, Long-Range Comms
A custom-built, rugged outdoor rover designed for efficiency and modularity.
* **Hardware:** Optimized for weight and energy efficiency using low-consumption motors.
* **Autonomy:** ArduPilot-based architecture featuring GPS waypoint navigation and obstacle avoidance (LiDAR).
* **Modularity:** Designed with a scalable payload bay to support Lidar and Camera integration.
* [Gallery](https://drive.google.com/drive/folders/10yuC23ChjQP-XX6tkt6CHY55PI3pUgJr?usp=drive_link)

### 5. LUMEN: Indoor Autonomous Robot
**Focus:** Indoor Navigation
* **Description:** A compact autonomous platform designed for indoor mapping and navigation challenges. Utilizes LIDAR and wheel odometry for localization, can fuse imu data if needed using robot_localization package and utilizes Nav2 for navigation.
* Links: [Lumen_bringup](https://github.com/anaskherro/lumen_bringup)
* [Gallery](https://drive.google.com/drive/folders/1Z2_rpDoNiELnXS25UgHCnB0VAam_mEan?usp=drive_link)
---

## 🚢 Marine Robotics (USVs)

### 6. TRITON: ROS2/Pixhawk Autonomous USV
**Focus:** ROS2 Migration, State Estimation (EKF)
* **Github:** [EKF State Estimation](https://github.com/anaskherro/ros_ekf) | [Visualizations](https://github.com/anaskherro/usv_visualizations)
* [Gallery](https://drive.google.com/drive/folders/14xPx6Xt03T0g7zk6x1JF4C5E2yV9lM6p?usp=drive_link)
* A maritime robotics project leveraging the newer **ROS2** framework.
* **State Estimation:** Implemented an **Extended Kalman Filter (EKF)** to fuse IMU and GPS data for precise localization in water environments.
* **Visualization:** Custom Rviz plugins for monitoring USV telemetry and trajectory.
* **Autopilot Integration :** Can be used with Ardupilot for other puposes.

---

## 🔬 Ongoing Research & Development

### 🚧 Data-Driven Control for High-Speed Drones
Investigating the use of Machine Learning and data-driven methods to enhance control authority during high-speed flight, compensating for unmodeled aerodynamic effects.

### 🚧 D.A.R.T: Autonomous Delivery Robot
Developing a last-mile delivery solution focusing on urban navigation and payload security.

---
*Open to technical discussions regarding implementation details and architectural choices.*
