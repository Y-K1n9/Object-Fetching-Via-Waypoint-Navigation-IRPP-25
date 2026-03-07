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

# Ensure Gazebo can find built-in models (ground_plane, sun) without internet
os.environ.setdefault('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models')

# ── ArUco model generation (must happen BEFORE gzserver starts) ──
_ARUCO_MODEL_ROOT = '/tmp/aruco_gazebo_models'


def _pre_generate_aruco_models():
    """Create Gazebo model directories with ArUco marker textures (IDs 0-2).

    These models MUST exist before gzserver starts so that Ogre can
    register the material scripts and resolve texture paths via
    model:// URIs.
    """
    script = r'''
import sys, os
try:
    import cv2, cv2.aruco, numpy as np
except ImportError:
    sys.exit(0)                 # silently skip if OpenCV missing

MODEL_ROOT = "/tmp/aruco_gazebo_models"
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

for mid in range(3):
    model_dir = os.path.join(MODEL_ROOT, f"aruco_marker_{mid}")
    tex_dir   = os.path.join(model_dir, "materials", "textures")
    scr_dir   = os.path.join(model_dir, "materials", "scripts")
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(scr_dir, exist_ok=True)

    # ArUco PNG: 150px inner + 25px white border = 200x200
    png_path = os.path.join(tex_dir, f"marker_{mid}.png")
    inner = cv2.aruco.drawMarker(dictionary, mid, 150)
    img = np.full((200, 200), 255, dtype=np.uint8)
    img[25:175, 25:175] = inner
    cv2.imwrite(png_path, img)

    # Ogre material script
    with open(os.path.join(scr_dir, "marker.material"), "w") as f:
        f.write(f"material ArUco/Marker{mid}\n")
        f.write("{\n  technique\n  {\n    pass\n    {\n")
        f.write("      texture_unit\n      {\n")
        f.write(f"        texture marker_{mid}.png\n")
        f.write("        filtering trilinear\n")
        f.write("      }\n    }\n  }\n}\n")

    # model.config (required for model:// URI resolution)
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write('<?xml version="1.0"?>\n<model>\n')
        f.write(f"  <name>aruco_marker_{mid}</name>\n")
        f.write("  <version>1.0</version>\n")
        f.write(f"  <description>ArUco 4x4 marker ID {mid}</description>\n")
        f.write("</model>\n")

print("aruco_models_ready")
'''
    try:
        result = subprocess.run(
            ['python3', '-c', script],
            capture_output=True, text=True, timeout=15
        )
        if 'aruco_models_ready' in (result.stdout or ''):
            print('[launch] ArUco marker models generated OK')
        else:
            print(f'[launch] ArUco model generation issue: {result.stderr}')
    except Exception as e:
        print(f'[launch] ArUco model generation failed: {e}')


# Run generation immediately (before any launch actions)
_pre_generate_aruco_models()

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


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
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Launch Gazebo GUI client (gzclient) for 3D visualization'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 for Nav2 visualization'
    )

    # ── File Paths ──
    world_file = os.path.join(pkg_share, 'worlds', 'object_fetching.world')
    map_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    marker_config_file = os.path.join(pkg_share, 'config', 'marker_config.yaml')

    # TurtleBot3 Waffle SDF model path
    tb3_model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'

    # ══════════════════════════════════════════════════════════════════════
    # STEP 1: Launch Gazebo physics server ONLY (headless — no GUI client)
    # The world file already loads gazebo_ros_init + gazebo_ros_factory as
    # world plugins, so we must NOT also pass them via -s (causes type error).
    # Running gzserver avoids the black-screen / freeze issue on containers.
    # ══════════════════════════════════════════════════════════════════════
    gazebo = ExecuteProcess(
        cmd=[
            'gzserver',
            '--minimal-comms',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # Gazebo GUI client -- launches the 3D visualization window
    # Render engine params reduce software-rendering overhead
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--render-engine-server', 'ogre'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # RViz2 for Nav2 visualisation (map, costmaps, path, robot footprint)
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 2: Spawn TurtleBot3 Waffle (after 5s, giving Gazebo time to start)
    # ══════════════════════════════════════════════════════════════════════
    spawn_robot = TimerAction(
        period=3.0,
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
    urdf_xacro_path = '/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf'
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

    nav2_launch = TimerAction(period=6.0, actions=nav2_actions) if nav2_actions else None

    # ══════════════════════════════════════════════════════════════════════
    # STEP 5: Object Fetcher Brain Nodes (after 15s)
    # ══════════════════════════════════════════════════════════════════════
    brain_nodes = TimerAction(
        period=10.0,
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
            # pickup_site_node is created INSIDE main_controller as a sub-node,
            # so we do NOT launch it separately (avoids duplicate Gazebo spawning).
        ]
    )

    # ── Assemble launch description ──
    ld = LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                               f'/usr/share/gazebo-11/models:{_ARUCO_MODEL_ROOT}'),
        # Disable online model database fetch (huge startup stall)
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        # Software rendering -- critical for containers without GPU
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        # Mesa/llvmpipe speedups: skip error checks, cache shaders, use 4 threads
        SetEnvironmentVariable('MESA_NO_ERROR', '1'),
        SetEnvironmentVariable('MESA_SHADER_CACHE_DISABLE', 'false'),
        SetEnvironmentVariable('LP_NUM_THREADS', '4'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        # Disable FSAA in software rendering (massively expensive)
        SetEnvironmentVariable('OGRE_FSAA', '0'),
        strategy_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        gazebo,
        gzclient,
        spawn_robot,
        robot_state_publisher,
        rviz,
        brain_nodes,
    ])

    if nav2_launch:
        ld.add_action(nav2_launch)

    return ld
