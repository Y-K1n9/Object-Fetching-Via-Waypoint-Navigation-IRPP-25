# 🤖 Object Fetching via Waypoint Navigation (IRPP-25) - Final Report

## 1. Problem Statement
The goal is to implement an **Autonomous Object Fetching System** using a TurtleBot3 Waffle robot in a simulated Gazebo environment. The robot must:
1.  Navigate successfully to 3 distinct Pickup Zones.
2.  Use computer vision to detect an ArUco marker at each zone.
3.  "Pick up" an object (simulated) and return it to a Home Base.
4.  **Bonus:** Optimize the sequence by always visiting the **nearest available zone** first (Distance-based scheduling).

---

## 2. System Architecture (Nodes & Logic)

The project consists of 5 specialized ROS 2 nodes running within a `MultiThreadedExecutor` to ensure concurrent processing.

### The Nodes
- **`main_controller` (The Brain)**: A State Machine that orchestrates the mission.
- **`task_scheduler` (The Logistician)**: Calculates the path sequence using $L_2$ distance logic.
- **`waypoint_navigator` (The Driver)**: Translates coordinates into Nav2 Action goals.
- **`marker_detector` (The Eyes)**: Uses OpenCV to find ArUco IDs (0, 1, 2).
- **`pickup_site_node` (The Handler)**: Simulates the physical pickup/drop-off confirmation.

### Communication (Topics & Services)
| Type | Name | Data Type | Purpose |
| :--- | :--- | :--- | :--- |
| **Service** | `/start_mission` | `std_srvs/Trigger` | Initiates the autonomous loop. |
| **Service** | `/stop_mission` | `std_srvs/Trigger` | Aborts immediately. |
| **Topic** | `/aruco/marker_ids` | `Int32MultiArray` | Camera detections sent to Brain. |
| **Topic** | `/mission_state` | `String` | Current status (e.g., "NAVIGATING"). |
| **Action** | `NavigateToPose` | `nav2_msgs/Action` | High-level navigation commands. |

---

## 3. Environment Details (The "World")

The robot operates in a **10m x 10m** arena (Map Origin: -5, -5).

### Coordinates (Meters)
| Feature | Name | X | Y | Object | Marker ID |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Drop-off** | Home Base | **0.0** | **0.0** | - | - |
| **Zone 1** | Alpha | **2.0** | **1.5** | Red Box | 0 |
| **Zone 2** | Beta | **-1.5** | **2.5** | Blue Cube | 1 |
| **Zone 3** | Gamma | **2.5** | **-1.0** | Green Cylinder | 2 |

### Obstacles (Static Boxes)
These are placed to force Nav2 to plan intelligent paths:
- **Obstacle 1**: (1.0, 0.5)
- **Obstacle 2**: (-0.5, 1.5)
- **Obstacle 3**: (1.5, -0.5)

---

## 4. Key Implementation Logic

### Distance-Based Bonus Strategy
The `TaskScheduler` uses the robot's current pose to sort remaining zones. 
Formula: $Distance = \sqrt{(x_{robot} - x_{zone})^2 + (y_{robot} - y_{zone})^2}$. 
It always selects the zone with the smallest result.

### Robustness Fixes Applied
1.  **Executor Deadlock**: Nodes share a `MultiThreadedExecutor` (4 threads) so the Action Client and State Machine don't block each other.
2.  **Platform "Clipping"**: Collision physics were removed from the 2cm platforms in the world file. This prevents the robot from getting "stuck" on edges or seeing them as obstacles.
3.  **Navigation Guards**: Logic flags prevent the 0.5s timer loop from sending redundant goals while the robot is already moving.
4.  **Tolerance**: `goal_tolerance` is set to **0.5m** to ensure reliable "Arrival" triggers despite simulation jitter.

---

## 5. How to Launch (Quick Reference)
1.  **Simulation**: `./quick_start.sh`
2.  **Visualization**: `./run_rviz.sh`
3.  **Command**: `ros2 service call /start_mission std_srvs/srv/Trigger "{}"`
