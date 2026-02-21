# IRPP-25: Autonomous Object Fetching via Waypoint Navigation 🤖📦

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-orange)](https://gazebosim.org/)
[![Status](https://img.shields.io/badge/Status-Complete (Bonus Achieved)-success)](#)

An end-to-end autonomous robotic system that retrieves objects from multiple scattered pickup zones and delivers them to a central Home Base. Developed using **ROS 2 Humble**, **Nav2**, and **OpenCV**.

---

## 📑 Submission Materials
*   **Technical Report (LaTeX)**: Located in [`docs/mid_submission_report.tex`](./docs/mid_submission_report.tex).
*   **System Diagram**: [`docs/computation_graph.md`](./docs/computation_graph.md).
*   **Demo Video**: [Click here for the demonstration video](#) (Update placeholder in LaTeX).

---

## ✨ Key Features
*   **Full Autonomy**: Self-contained mission coordination from startup to completion.
*   **Robust Navigation**: Integrated Nav2 stack with tuned TurtleBot3 Waffle parameters.
*   **Visual Intelligence**: ArUco marker detection for precise object verification.
*   **⭐ Bonus logic**: Distance-based task scheduling — the robot always chooses the most efficient next path.
*   **Multi-Threaded Architecture**: Prevents deadlocks during complex action/service callbacks.

---

## 🏗️ Quick Start (Recommended)

This project is fully dockerized to ensure it runs on any system without dependency issues.

### 1. Prerequisites
*   Ubuntu (Linux) recommended.
*   [Docker](https://docs.docker.com/get-docker/) installed.
*   An X11 server for Gazebo/RViz GUI support.

### 2. Execution Steps
We have provided a one-command script to handle building, mapping, and launching the simulation.

```bash
# 1. Step into the workspace
cd ros2_workspace

# 2. Build the environment (First time only)
docker build -t object_fetcher_ros2 .

# 3. Enable GUI support & Start the mission
# This will launch Gazebo, RViz, Nav2, and the Mission Brain
./quick_start.sh
```

### 3. Trigger the Mission
Once the simulation is fully loaded (Wait for RViz and Gazebo to appear), open a **new terminal** and run:
```bash
docker exec -it object_fetcher_container bash -c "source install/setup.bash && ros2 service call /start_mission std_srvs/srv/Trigger '{}'"
```

---

## 🛠️ System Overview

### Core Nodes
- **`main_controller`**: The brain. Manages the global state machine.
- **`task_scheduler`**: The logic. Implements Euclidean distance-based prioritization.
- **`marker_detector`**: The eyes. Uses OpenCV to identify assets via ArUco markers.
- **`waypoint_navigator`**: The driver. Action client for the Nav2 navigation stack.

---

## � Maintenance & Build
To build manually without the start script:
```bash
colcon build --packages-select object_fetcher --symlink-install
source install/setup.bash
ros2 launch object_fetcher full_nav2.launch.py
```

---
**Author**: Siddhant
**Project**: IRPP-25 | Robotics & Automation
