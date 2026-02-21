# Object Fetching via Waypoint Navigation — IRPP-25

Autonomous object fetching robot using **TurtleBot3 + Nav2 + ArUco markers** in Gazebo.

## 🏗️ Project Structure

```
ros2_workspace/
├── Dockerfile                          ← Docker environment (all deps included)
├── src/
│   ├── box_spawner.py                  ← Teammate's box spawner
│   └── object_fetcher/                 ← Our ROS 2 package
│       ├── object_fetcher/
│       │   ├── main_controller.py      ← Mission brain (state machine)
│       │   ├── waypoint_navigator.py   ← Nav2 action client
│       │   ├── marker_detector.py      ← ArUco camera detection
│       │   ├── task_scheduler.py       ← Distance/priority scheduling (BONUS)
│       │   └── pickup_site_node.py     ← Object confirmation service
│       ├── config/
│       │   ├── waypoints.yaml          ← Zone coordinates + priorities
│       │   ├── marker_config.yaml      ← ArUco detector settings
│       │   └── nav2_params.yaml        ← Nav2 tuned for TurtleBot3
│       ├── worlds/
│       │   └── object_fetching.world   ← Gazebo world (3 zones + home base)
│       ├── maps/
│       │   ├── map.pgm                 ← Navigation map image
│       │   └── map.yaml                ← Map metadata
│       └── launch/
│           ├── object_fetcher.launch.py ← Brain nodes only
│           └── full_nav2.launch.py      ← Complete stack (one command!)
```

## 🗺️ World Layout

```
        Zone Beta (-1.5, 2.5) [BLUE]
              🟦

Zone Alpha (2.0, 1.5) [GREEN]
      🟩

  Home Base (0, 0) [YELLOW]
        🟨

              Zone Gamma (2.5, -1.0) [RED]
                    🟥
```

## 🚀 How to Run

### Option A: Full Autonomous Stack (Recommended for eval)

**Step 1 — Build the Docker image** (one time):
```bash
cd ros2_workspace
docker build -t object_fetcher_ros2 .
```

**Step 2 — Start the container** (with GUI support):
```bash
xhost +local:docker
docker run -it --rm \
  --network=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/src:/home/ros/ws/src \
  object_fetcher_ros2 bash
```

**Step 3 — Inside the container, build the package**:
```bash
cd /home/ros/ws
colcon build --packages-select object_fetcher --symlink-install
source install/setup.bash
```

**Step 4 — Generate the navigation map**:
```bash
python3 src/object_fetcher/maps/generate_map.py
cp src/object_fetcher/maps/map.* install/object_fetcher/share/object_fetcher/maps/
```

**Step 5 — Launch everything**:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch object_fetcher full_nav2.launch.py
```

**Step 6 — Start the mission** (in a new terminal inside the container):
```bash
ros2 service call /start_mission std_srvs/srv/Trigger "{}"
```

---

### Option B: Quick Test (without full Nav2)

**Terminal 1** — Gazebo with TurtleBot3:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2** — Brain nodes:
```bash
ros2 launch object_fetcher object_fetcher.launch.py
```

**Terminal 3** — Start mission:
```bash
ros2 service call /start_mission std_srvs/srv/Trigger "{}"
```

---

### Original teammate setup (box spawner):
```bash
# Terminal 1
python3 src/box_spawner.py

# Terminal 2
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 3
ros2 run turtlebot3_teleop teleop_keyboard
```

## 🧠 How It Works

### Mission Flow
```
START → PLANNING → NAVIGATE TO PICKUP → DETECT MARKER → CONFIRM PICKUP
                                                              ↓
                        PLANNING ← DELIVER ← NAVIGATE TO HOME BASE
```

### Task Scheduling (Bonus Feature ⭐)
The robot uses **distance-based scheduling** — it always visits the **nearest unvisited zone first**.

To switch to priority-based:
```bash
ros2 launch object_fetcher full_nav2.launch.py scheduling_strategy:=priority
```

### Pickup Zones
| Zone | Location | Marker ID | Object |
|------|----------|-----------|--------|
| Alpha | (2.0, 1.5) | 0 | Red Box |
| Beta | (-1.5, 2.5) | 1 | Blue Cube |
| Gamma | (2.5, -1.0) | 2 | Green Cylinder |
| Home Base | (0.0, 0.0) | — | Drop-off |

## 📡 ROS 2 Topics & Services

| Name | Type | Description |
|------|------|-------------|
| `/start_mission` | Service (Trigger) | Start the autonomous mission |
| `/stop_mission` | Service (Trigger) | Abort the mission |
| `/mission_state` | Topic (String) | Current state machine state |
| `/aruco/marker_ids` | Topic (Int32MultiArray) | Detected ArUco IDs |
| `/navigation_status` | Topic (String) | Nav2 navigation status |
| `/confirm_pickup` | Service (Trigger) | Confirm object at pickup site |
