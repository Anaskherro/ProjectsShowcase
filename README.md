# Portfolio: Robotics & Autonomous Systems Engineering
### Anas [Your Last Name] | Electromechanical & Robotics Engineer

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue?style=flat&logo=linkedin)](YOUR_LINKEDIN_URL)
[![Email](https://img.shields.io/badge/Email-Contact_Me-red?style=flat&logo=gmail)](mailto:YOUR_EMAIL)

---

## 🚀 Professional Summary
I am a Robotics Engineer focused on bridging the gap between theoretical simulation and rugged, real-world application. My work emphasizes **robust control systems**, **state estimation**, and **autonomous navigation** in resource-constrained environments.

Below is a collection of my key projects, ranging from aerial platforms (UAVs) to ground vehicles (UGVs) and marine systems (USVs).

---

## 🛠 Skills Snapshot
* **Core Stack:** ROS / ROS2, C++, Python, MATLAB/Simulink
* **Flight Controllers:** Pixhawk, ArduPilot, PX4
* **Control & Planning:** MPC, PID, EKF, A*, Hybrid A*, Pure Pursuit
* **Simulation:** Gazebo, CARLA, Rviz

---

## 🚁 Aerial Robotics (UAVs)

### 1. Quadrotor Navigation in Obstacle-Dense Environments
**Focus:** Path Planning, Control Theory, Benchmarking
* **Github:** [Path Planning](https://github.com/anaskherro/quadrotor_path_plannig) | [Path Tracking](https://github.com/anaskherro/quadrotor_path_tracking)

> **The Challenge:** Navigating a quadcopter through complex, cluttered environments requires precise global planning and reactive control.
> **The Solution:** Implemented a full autonomy stack using **ROS and Pixhawk**. I developed a **Hybrid A*** global planner for efficient trajectory generation and conducted a rigorous benchmark between **Model Predictive Control (MPC)** and **PID** controllers to analyze performance under dynamic constraints.

![Quadrotor Navigation Demo](link_to_your_gif_or_image.gif)

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

![Carla Simulation Demo](link_to_your_gif_or_image.gif)

### 4. REVER: Outdoor Autonomous Rover
**Focus:** Hardware Integration, System Design, Long-Range Comms

A custom-built, rugged outdoor rover designed for efficiency and modularity.
* **Hardware:** Optimized for weight and energy efficiency using high-torque/low-consumption motors.
* **Autonomy:** ArduPilot-based architecture featuring GPS waypoint navigation and obstacle avoidance.
* **Modularity:** Designed with a scalable payload bay to support Lidar and Camera integration for future SLAM implementations.

### 5. LUMEN: Indoor Autonomous Robot
**Focus:** Indoor Navigation
* **Description:** A compact autonomous platform designed for indoor mapping and navigation challenges. (Add 1 more sentence here about sensors, e.g., "Utilizes LIDAR and odometry for localization").

---

## 🚢 Marine Robotics (USVs)

### 6. TRITON: ROS2 Autonomous Surface Vehicle
**Focus:** ROS2 Migration, State Estimation (EKF)
* **Github:** [EKF State Estimation](https://github.com/anaskherro/ros_ekf) | [Visualizations](https://github.com/anaskherro/usv_visualizations)

A maritime robotics project leveraging the newer **ROS2** framework.
* **State Estimation:** Implemented an **Extended Kalman Filter (EKF)** to fuse IMU and GPS data for precise localization in water environments.
* **Visualization:** Custom Rviz plugins for monitoring USV telemetry and trajectory.

---

## 🔬 Ongoing Research & Development

### 🚧 Data-Driven Control for High-Speed Drones
Investigating the use of Machine Learning and data-driven methods to enhance control authority during high-speed flight, compensating for unmodeled aerodynamic effects.

### 🚧 D.A.R.T: Autonomous Delivery Robot
Developing a last-mile delivery solution focusing on urban navigation and payload security.

---
*Open to technical discussions regarding implementation details and architectural choices.*