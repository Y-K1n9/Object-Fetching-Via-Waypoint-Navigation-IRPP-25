# Object Fetching Via Waypoint Navigation — IRPP-25

Autonomous object fetching system using a **TurtleBot3 Waffle** robot in a simulated Gazebo environment. The robot autonomously navigates a 10 m × 10 m walled room with corridors, detects objects using **ArUco markers**, picks them up, and delivers them to a home base — all orchestrated by a finite state machine running on **ROS 2 Humble** with **Nav2**.

---

## Table of Contents

- [Problem Statement](#problem-statement)
- [Approach](#approach)
  - [System Architecture](#system-architecture)
  - [State Machine](#state-machine)
  - [Task Scheduling Strategies](#task-scheduling-strategies)
  - [Perception — ArUco Marker Detection](#perception--aruco-marker-detection)
  - [Navigation — Nav2 Integration](#navigation--nav2-integration)
  - [Dynamic Object Spawning](#dynamic-object-spawning)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Setup & Installation](#setup--installation)
- [Running the Simulation](#running-the-simulation)
  - [Quick Start (One Command)](#quick-start-one-command)
  - [Start the Mission](#start-the-mission)
  - [Monitor the Mission](#monitor-the-mission)
  - [Stop the Mission](#stop-the-mission)
  - [Launch Options](#launch-options)
- [Configuration](#configuration)
- [ROS 2 Topics & Services](#ros-2-topics--services)
- [Node Descriptions](#node-descriptions)

---

## Problem Statement

A mobile robot must autonomously:
1. **Navigate** to multiple pickup zones within an indoor environment containing walls, corridors, and obstacles.
2. **Detect and identify** specific objects at each zone using vision (ArUco markers).
3. **Collect** the objects and **deliver** them to a designated drop-off (home base) location.
4. **Optimise** the visitation order to minimise total travel distance.

The pickup zones are **randomised** at every simulation run, requiring the system to dynamically plan routes at runtime rather than relying on hard-coded paths.

---

## Approach

### System Architecture

The system is composed of four cooperating ROS 2 nodes running inside a single process via a `MultiThreadedExecutor`:

```
┌──────────────────────────────────────────────────────────────┐
│                      main_controller                         │
│              (Mission Brain / State Machine)                  │
│                                                              │
│   ┌─────────────────┐  ┌──────────────────┐                 │
│   │  task_scheduler  │  │ waypoint_navigator│                │
│   │  (Zone ordering) │  │ (Nav2 action      │                │
│   │                  │  │  client)           │                │
│   └─────────────────┘  └──────────────────┘                 │
│   ┌─────────────────┐  ┌──────────────────┐                 │
│   │ pickup_site_node │  │  marker_detector  │                │
│   │ (Object spawning │  │  (ArUco vision)   │                │
│   │  & confirmation) │  │                   │                │
│   └─────────────────┘  └──────────────────┘                 │
└──────────────────────────────────────────────────────────────┘
         │                         │
         ▼                         ▼
   ┌───────────┐           ┌──────────────┐
   │  Gazebo   │           │   Nav2 Stack  │
   │ Simulator │           │ (AMCL, Path   │
   │           │           │  Planner,     │
   │           │           │  Controller)  │
   └───────────┘           └──────────────┘
```

### State Machine

The `main_controller` implements a finite state machine that drives the entire mission:

```
IDLE
  ↓  (receive /start_mission service call)
PLANNING              ← Ask scheduler: "where should I go next?"
  ↓
NAVIGATING_TO_PICKUP  ← Drive to the nearest unvisited pickup zone
  ↓
DETECTING_MARKER      ← Wait for camera to see the ArUco marker
  ↓
CONFIRMING_PICKUP     ← Verify object identity via pickup_site_node
  ↓
NAVIGATING_TO_DROPOFF ← Drive back to Home Base (0, 0)
  ↓
DELIVERING            ← "Drop off" the object (simulated)
  ↓
PLANNING              ← Loop back for the next zone
  ↓  (all zones done)
COMPLETED
```

**Error Handling:** If navigation to a zone fails, the robot retries from up to 4 different approach angles (0°, 90°, −90°, 180°) before skipping that zone. Stuck-detection cancels navigation if the robot makes less than 0.3 m of progress within a 30-second window.

### Task Scheduling Strategies

Two strategies are available (configurable at launch):

| Strategy   | Description | Behaviour |
|------------|-------------|-----------|
| `distance` | **Nearest-first** (greedy) — always visits the closest unvisited zone based on Euclidean distance from the robot's current position. | Minimises total travel distance. |
| `priority` | **Priority-ordered** — visits zones in order of a pre-assigned priority value (1 = highest). | Useful when certain objects are more urgent. |

Default: **`distance`**

### Perception — ArUco Marker Detection

- **ArUco Dictionary:** `DICT_4X4_50` (4×4 grid markers, up to 50 unique IDs)
- Each pickup zone has an associated marker ID:
  - Zone Alpha → Marker ID `0`  (Red Box)
  - Zone Beta  → Marker ID `1`  (Blue Cube)
  - Zone Gamma → Marker ID `2`  (Green Cylinder)
- The `marker_detector` node subscribes to `/camera/image_raw`, converts frames via `cv_bridge`, detects markers using OpenCV, and publishes detected IDs on `/aruco/marker_ids`.
- Falls back to **simulation mode** (timeout-based) if OpenCV/cv_bridge is unavailable.

### Navigation — Nav2 Integration

- Uses the **Nav2** stack with **AMCL** localisation against a pre-built occupancy grid map.
- The `waypoint_navigator` node acts as a Nav2 `NavigateToPose` action client.
- Nav2 parameters are tuned for the environment:
  - **A*** planner for efficient path-finding through rooms.
  - **0.40 m inflation radius** — tight enough for doorway passage.
  - **5 × 5 m local costmap** to see around corridor corners.
  - Generous transform tolerance (3.0 s) for headless Gazebo.

### Dynamic Object Spawning

The `pickup_site_node`:
1. **Generates 3 random positions** inside the room at every launch (avoiding walls, obstacles, and the home base via exclusion rectangles).
2. **Spawns** a coloured platform + object + ArUco marker placeholder at each position through Gazebo's `/spawn_entity` service.
3. **Overrides** the static waypoints in the task scheduler so the robot navigates to the new positions.

This ensures the robot faces a **different layout every run**, validating generalisation of the navigation and planning approach.

---

## Project Structure

```
ros2_workspace/
├── Dockerfile                        # Docker image (ROS 2 Humble desktop)
├── .devcontainer/
│   └── devcontainer.json             # VS Code Dev Container config
└── src/
    ├── box_spawner.py                # Standalone random box spawner (legacy)
    ├── perception_node.py            # Placeholder perception node
    └── object_fetcher/               # Main ROS 2 package
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── config/
        │   ├── waypoints.yaml        # Pickup zone & dropoff definitions
        │   ├── nav2_params.yaml      # Nav2 stack parameters
        │   ├── marker_config.yaml    # ArUco detector parameters
        │   └── nav2_view.rviz        # RViz2 config for visualisation
        ├── launch/
        │   ├── full_nav2.launch.py   # Full stack: Gazebo + Nav2 + brain nodes
        │   └── object_fetcher.launch.py  # Brain nodes only
        ├── maps/
        │   ├── map.yaml              # Map metadata
        │   ├── map.pgm               # Occupancy grid map (10 m × 10 m)
        │   └── generate_map.py       # Map generation script
        ├── worlds/
        │   └── object_fetching.world # Gazebo world (walls, corridors, obstacles)
        └── object_fetcher/           # Python package
            ├── __init__.py
            ├── main_controller.py    # Mission orchestrator (state machine)
            ├── waypoint_navigator.py # Nav2 action client
            ├── task_scheduler.py     # Zone ordering (distance / priority)
            ├── marker_detector.py    # ArUco marker detection (OpenCV)
            └── pickup_site_node.py   # Random spawning & pickup confirmation
```

---

## Prerequisites

- **ROS 2 Humble** (desktop install)
- **Gazebo 11** (Classic)
- **TurtleBot3 packages:**
  - `turtlebot3_gazebo`
  - `turtlebot3_navigation2`
- **Nav2** (`navigation2` stack)
- **Python 3** with `opencv-python`, `cv_bridge`, `numpy`
- **colcon** build tool

> **Tip:** Use the included Dev Container / Docker setup to get a fully pre-configured environment.

---

## Setup & Installation

### Option A: Dev Container (Recommended)

1. Install [Docker](https://docs.docker.com/get-docker/) and [VS Code](https://code.visualstudio.com/) with the **Dev Containers** extension.
2. Open the `ros2_workspace/` folder in VS Code.
3. When prompted, **Reopen in Container** (or use the Command Palette → *Dev Containers: Reopen in Container*).
4. Inside the container, build the workspace:

```bash
cd ~/ws
colcon build --symlink-install
source install/setup.bash
```

### Option B: Native Install

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Install dependencies
sudo apt update && sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cv-bridge \
  ros-humble-robot-state-publisher \
  python3-opencv

# 3. Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# 4. Navigate to workspace and build
cd /path/to/ros2_workspace
colcon build --symlink-install

# 5. Source the workspace overlay
source install/setup.bash
```

---

## Running the Simulation

### Quick Start (One Command)

Launch Gazebo, Nav2, the robot, and all brain nodes:

```bash
# Terminal 1 — Source workspace & launch the full stack
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch object_fetcher full_nav2.launch.py
```

This starts:
1. **Gazebo server** (headless by default) with the walled-room world
2. **TurtleBot3 Waffle** spawned at the origin (0, 0)
3. **Nav2 stack** (AMCL localisation + planner + controller) after 10 s
4. **Object fetcher brain nodes** (main_controller + marker_detector) after 15 s
5. **3 pickup zones** spawned at random positions

### Start the Mission

Once all nodes are ready (watch for `MainController ready!` in the logs):

```bash
# Terminal 2 — Trigger the autonomous mission
ros2 service call /start_mission std_srvs/srv/Trigger "{}"
```

The robot will autonomously:
1. Plan the optimal visitation order
2. Navigate to each pickup zone
3. Detect the ArUco marker
4. Confirm the object
5. Return to home base and deliver
6. Repeat until all zones are completed

### Monitor the Mission

```bash
# Watch the mission state in real-time
ros2 topic echo /mission_state

# Watch navigation status
ros2 topic echo /navigation_status

# See detected ArUco markers
ros2 topic echo /aruco/marker_ids

# See current pickup site object
ros2 topic echo /pickup_site/current_object
```

### Stop the Mission

```bash
ros2 service call /stop_mission std_srvs/srv/Trigger "{}"
```

### Launch Options

| Argument | Default | Description |
|----------|---------|-------------|
| `scheduling_strategy` | `distance` | `distance` (nearest-first) or `priority` (priority-ordered) |
| `use_sim_time` | `true` | Use Gazebo simulation clock |
| `gui` | `false` | Launch Gazebo GUI (`gzclient`) for 3D visualisation |
| `rviz` | `false` | Launch RViz2 with Nav2 visualisation |

**Examples:**

```bash
# With Gazebo GUI and RViz
ros2 launch object_fetcher full_nav2.launch.py gui:=true rviz:=true

# Using priority-based scheduling
ros2 launch object_fetcher full_nav2.launch.py scheduling_strategy:=priority

# Brain nodes only (when Gazebo & Nav2 are already running separately)
ros2 launch object_fetcher object_fetcher.launch.py
```

---

## Configuration

| File | Purpose |
|------|---------|
| `config/waypoints.yaml` | Defines pickup zones (positions, marker IDs, priorities) and the drop-off zone. Positions are overridden at runtime by random spawning. |
| `config/nav2_params.yaml` | Full Nav2 parameter set — AMCL, global/local costmaps, planner, controller, recovery behaviours. |
| `config/marker_config.yaml` | ArUco detector settings — dictionary type, marker size, camera topic. |
| `maps/map.yaml` + `map.pgm` | Pre-built occupancy grid map of the 10 m × 10 m room (0.05 m/pixel resolution). |
| `worlds/object_fetching.world` | Gazebo world file — outer walls, interior partitions, static obstacles (tables, barrels, crates). |

---

## ROS 2 Topics & Services

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mission_state` | `std_msgs/String` | Current state of the mission FSM |
| `/navigation_status` | `std_msgs/String` | Nav2 navigation status (NAVIGATING / SUCCESS / FAILED) |
| `/aruco/marker_ids` | `std_msgs/Int32MultiArray` | Detected ArUco marker IDs |
| `/aruco/marker_poses` | `geometry_msgs/PoseArray` | 3D positions of detected markers |
| `/aruco/debug_image` | `sensor_msgs/Image` | Camera feed with markers highlighted |
| `/pickup_site/current_object` | `std_msgs/String` | Object info at the active pickup zone |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/start_mission` | `std_srvs/Trigger` | Begin the autonomous fetch mission |
| `/stop_mission` | `std_srvs/Trigger` | Abort the current mission |
| `/confirm_pickup` | `std_srvs/Trigger` | Confirm object identity at the active zone |

---

## Node Descriptions

| Node | Executable | Role |
|------|-----------|------|
| `main_controller` | `main_controller` | Mission brain — runs the state machine, coordinates all other nodes |
| `waypoint_navigator` | `waypoint_navigator` | Sends `NavigateToPose` goals to Nav2, monitors progress, detects stuck conditions |
| `task_scheduler` | `task_scheduler` | Decides zone visitation order using distance or priority strategy |
| `marker_detector` | `marker_detector` | Detects ArUco markers from camera feed using OpenCV |
| `pickup_site_node` | `pickup_site_node` | Generates random zone positions, spawns objects in Gazebo, confirms pickups |

---

## License

Apache-2.0
