#!/usr/bin/env python3
"""
full_nav2.launch.py
-------------------
ONE COMMAND to launch the ENTIRE autonomous stack:
  1. Gazebo with our custom world
  2. TurtleBot3 Waffle robot spawned in the world
  3. Nav2 navigation stack (AMCL + path planner + controller)
  4. All object_fetcher brain nodes

USAGE:
  export TURTLEBOT3_MODEL=waffle
  ros2 launch object_fetcher full_nav2.launch.py

THEN START THE MISSION:
  ros2 service call /start_mission std_srvs/srv/Trigger "{}"
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('object_fetcher')

    # Try to find Nav2 bringup package
    try:
        pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    except Exception:
        pkg_nav2_bringup = None

    # Try to find TurtleBot3 Gazebo package
    try:
        pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    except Exception:
        pkg_tb3_gazebo = None

    # ── Launch Arguments ──
    strategy_arg = DeclareLaunchArgument(
        'scheduling_strategy', default_value='distance',
        description='Task scheduling: "distance" (bonus) or "priority"'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation time'
    )

    # ── File Paths ──
    world_file = os.path.join(pkg_share, 'worlds', 'object_fetching.world')
    map_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    marker_config_file = os.path.join(pkg_share, 'config', 'marker_config.yaml')

    # TurtleBot3 Waffle SDF model path
    tb3_model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'

    # ══════════════════════════════════════════════════════════════════════
    # STEP 1: Launch Gazebo with our custom world (immediately)
    # ══════════════════════════════════════════════════════════════════════
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 2: Spawn TurtleBot3 Waffle (after 5s, giving Gazebo time to start)
    # ══════════════════════════════════════════════════════════════════════
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_waffle',
                    '-file', tb3_model_path,
                    '-x', '0', '-y', '0', '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 3: Robot State Publisher (publishes TF transforms)
    # The turtlebot3_waffle.urdf is actually a xacro file — must process it
    # with xacro first, otherwise robot_description is invalid and base_link
    # TF is never published (Nav2 will then fail to activate).
    # ══════════════════════════════════════════════════════════════════════
    urdf_xacro_path = '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf'
    robot_description = ''
    try:
        result = subprocess.run(
            ['xacro', urdf_xacro_path],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0 and result.stdout.strip():
            robot_description = result.stdout
        else:
            # xacro failed — try reading the file directly as a fallback
            with open(urdf_xacro_path, 'r') as f:
                robot_description = f.read()
    except Exception:
        try:
            with open(urdf_xacro_path, 'r') as f:
                robot_description = f.read()
        except Exception:
            robot_description = ''

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 4: Nav2 Stack (after 10s, giving robot time to spawn)
    # ══════════════════════════════════════════════════════════════════════
    nav2_actions = []
    if pkg_nav2_bringup:
        nav2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': nav2_params_file,
                'autostart': 'true',
            }.items()
        )
        nav2_actions.append(nav2_bringup)
    else:
        # Fallback: use TurtleBot3's built-in navigation
        try:
            pkg_tb3_nav = get_package_share_directory('turtlebot3_navigation2')
            tb3_nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_tb3_nav, 'launch', 'navigation2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': map_file,
                }.items()
            )
            nav2_actions.append(tb3_nav2)
        except Exception:
            pass  # Nav2 not available, nodes will still start

    nav2_launch = TimerAction(period=10.0, actions=nav2_actions) if nav2_actions else None

    # ══════════════════════════════════════════════════════════════════════
    # STEP 5: Object Fetcher Brain Nodes (after 15s)
    # ══════════════════════════════════════════════════════════════════════
    brain_nodes = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='object_fetcher',
                executable='main_controller',
                name='main_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'scheduling_strategy': LaunchConfiguration('scheduling_strategy'),
                    'marker_timeout_sec': 15.0,
                }]
            ),
            Node(
                package='object_fetcher',
                executable='marker_detector',
                name='marker_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'aruco_dict': 'DICT_4X4_50',
                    'marker_size': 0.15,
                    'camera_image_topic': '/camera/image_raw',
                    'publish_debug_image': True,
                }]
            ),
            Node(
                package='object_fetcher',
                executable='pickup_site_node',
                name='pickup_site_node',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }]
            ),
        ]
    )

    # ── Assemble launch description ──
    ld = LaunchDescription([
        strategy_arg,
        use_sim_time_arg,
        gazebo,
        spawn_robot,
        robot_state_publisher,
        brain_nodes,
    ])

    if nav2_launch:
        ld.add_action(nav2_launch)

    return ld
