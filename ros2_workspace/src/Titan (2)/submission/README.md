# Autonomous Object Fetching — ROS2 / TurtleBot3

## Requirements
- Docker Desktop
- VS Code with the **Dev Containers** extension

## Setup

1. Open this folder in VS Code.
2. When prompted, click **"Reopen in Container"** (or press `F1` → *Dev Containers: Reopen in Container*).
   This builds the Docker environment automatically using the `Dockerfile`.

## Build and Launch

**Terminal 1 — build and launch simulation:**
```bash
cd /workspaces/ros2_workspace   # this is where VS Code mounts the project
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
colcon build --symlink-install
source install/setup.bash
ros2 launch object_fetcher full_nav2.launch.py gui:=true rviz:=true
```

Wait until Gazebo and RViz are fully loaded.

**Terminal 2 — start the mission:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /start_mission std_srvs/srv/Trigger "{}"
```

The robot will autonomously navigate to all three pickup zones, detect ArUco markers, and return to home base.

## Notes
- If old Gazebo/RViz processes are running from a previous session, kill them first:
  ```bash
  pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f rviz2; sleep 2
  ```
- The report is included as `report.pdf`.
