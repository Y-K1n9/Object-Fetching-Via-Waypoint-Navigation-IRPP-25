# ROS 2 Computation Graph (rqt_graph)

This diagram visualizes how our nodes communicate via topics and actions.

```mermaid
graph LR
    %% Nodes
    MC["/main_controller"]
    MD["/marker_detector"]
    WN["/mc_waypoint_navigator"]
    TS["/mc_task_scheduler"]
    PS["/mc_pickup_site"]
    NAV2["Nav2 Stack"]
    GZ["Gazebo / Robot"]

    %% Connections
    GZ -- "/camera/image_raw" --> MD
    MD -- "/aruco/marker_ids" --> MC
    MC -- "/mission_state" --> UI["Monitoring / RViz"]
    
    MC -- Service Call --> TS
    TS -- "Returns Next Zone" --> MC
    
    MC -- "NavigateToPose" --> WN
    WN -- "Action Call" --> NAV2
    NAV2 -- "/cmd_vel" --> GZ
    GZ -- "/odom & /tf" --> NAV2
    
    MC -- Service Call --> PS
    PS -- "Confirm Pickup" --> MC
```

## How to see the interactive version:
1. Start your simulation with `./quick_start.sh`.
2. Open a new terminal and enter the docker container:
   ```bash
   docker exec -it $(docker ps -q --filter ancestor=object_fetcher) bash
   ```
3. Run the following command:
   ```bash
   ros2 run rqt_graph rqt_graph
   ```
   *Note: This will open a window showing the live connections between all running nodes.*
