#!/usr/bin/env python3
"""
object_fetcher.launch.py
------------------------
Launches all the object_fetcher brain nodes.

This launch file starts:
  1. main_controller  — the mission brain
  2. marker_detector  — ArUco camera detection
  3. task_scheduler   — zone ordering logic
  4. pickup_site_node — object confirmation service

NOTE: This does NOT start Gazebo or Nav2.
      Use full_nav2.launch.py for the complete stack.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('object_fetcher')

    # ── Launch Arguments (can be overridden from command line) ──
    strategy_arg = DeclareLaunchArgument(
        'scheduling_strategy',
        default_value='distance',
        description='Task scheduling strategy: "distance" or "priority"'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    # ── Config file paths ──
    waypoints_config = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    marker_config = os.path.join(pkg_share, 'config', 'marker_config.yaml')

    # ── Node definitions ──

    # 1. Main Controller — the mission brain
    main_controller_node = Node(
        package='object_fetcher',
        executable='main_controller',
        name='main_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scheduling_strategy': LaunchConfiguration('scheduling_strategy'),
            'marker_timeout_sec': 15.0,
        }]
    )

    # 2. Marker Detector — ArUco vision node
    marker_detector_node = Node(
        package='object_fetcher',
        executable='marker_detector',
        name='marker_detector',
        output='screen',
        parameters=[
            marker_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 3. Pickup Site Node — object confirmation service
    pickup_site_node = Node(
        package='object_fetcher',
        executable='pickup_site_node',
        name='pickup_site_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        strategy_arg,
        use_sim_time_arg,
        main_controller_node,
        marker_detector_node,
        pickup_site_node,
    ])
