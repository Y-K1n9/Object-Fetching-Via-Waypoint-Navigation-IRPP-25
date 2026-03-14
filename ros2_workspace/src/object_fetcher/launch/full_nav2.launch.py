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

MODEL_ROOT = "/tmp/aruco_gazebo_models" # here we find the aruco_marker textures
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

for mid in range(3):
    # this creates the correct path of directory whether the system is ubuntu(joining of path by /) or windows(joining of path by \)
    model_dir = os.path.join(MODEL_ROOT, f"aruco_marker_{mid}")
    tex_dir   = os.path.join(model_dir, "materials", "textures")
    scr_dir   = os.path.join(model_dir, "materials", "scripts")
    # the following lines will make sure that this code will make the requisite directories if they're not present in the computer... 
    # exist_ok = True , If the folder already exixts, don't try to create it...
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(scr_dir, exist_ok=True)

    # ArUco PNG: 150px inner + 25px white border = 200x200
    png_path = os.path.join(tex_dir, f"marker_{mid}.png") # we store the png as: /tmp/aruco_gazebo_models/materials/textures/marker_2.png
    # we draw the dictionary['mid'] design as a 150X150 pixel image and put it in inner variable
    inner = cv2.aruco.drawMarker(dictionary, mid, 150)
    img = np.full((200, 200), 255, dtype=np.uint8) # firstly create a blank 200X200 pixel image
    img[25:175, 25:175] = inner # and then draw the inner in img[25:175, 25:175]...
    cv2.imwrite(png_path, img) # we make a png file in png_path location...

    # Ogre material script
    # makes a gazebo texture from the generated png image by cv2...
    with open(os.path.join(scr_dir, "marker.material"), "w") as f:
        f.write(f"material ArUco/Marker{mid}\n")
        f.write("{\n  technique\n  {\n    pass\n    {\n")
        f.write("      texture_unit\n      {\n")
        f.write(f"        texture marker_{mid}.png\n")
        f.write("        filtering trilinear\n") # filtering setting that makes the rendering not be fuzzy when robo moves (close to)/(far from) the object...
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
    # the following command runs a subprocess-stops the program for a second
    # and run the specific python command...
    try:
        result = subprocess.run(
            # specific python command::(this needs to be done since the
            #                           launch file itself is just a function
            #                           that can't run any code, so we run it in this way)
            # it is a common way to handle pre-trigger setup 
            # here -> the generation of ArUco markers before 
            # ROS2 nodes actually start running...
            # we pass list of arguments to the os: 
            # ['python3', '-c', script]
            # python3 -> the executable to run...
            # -c: the command flag... it tells python to not look for
            # a py file rather, the string script = r'''</>'''
            ['python3', '-c', script],
            # capture_output = True :: stores the output of the script ran
            # into the result.stdout and result.stderr, 
            # if the subprocess runs for 15 sec, script probably ran into 
            # an error. Thus, we simply timeout in that case...
            capture_output=True, text=True, timeout=15
        )
        if 'aruco_models_ready' in (result.stdout or ''):
            # If the print("aruco_models_ready") was really seen 
            # in the terminal(without any exceptions) as output while running 
            # the script, it means that actually the functionality was done...
            print('[launch] ArUco marker models generated OK')
        else:
            print(f'[launch] ArUco model generation issue: {result.stderr}')
    except Exception as e:
            # if the running of the script ran into an exception, we log the user...
        print(f'[launch] ArUco model generation failed: {e}')


# Run generation immediately (before any launch actions, pre-trigger process)
_pre_generate_aruco_models()

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# we import node for this launch file from launch_ros.actions....
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # installation path of our package(object_fetcher) is stored in pkg_share...
    pkg_share = get_package_share_directory('object_fetcher')

    # Try to find Nav2 bringup package, if exception comes, proceed with None
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
    # These declare the arguments that we can change :::
    # strategy, use_sim_time, gui, rviz
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
    # the world file is located in: pkg_share/worlds/object_fetching.world
    # the map_file is located in: pkg_share/maps/map.yaml
    # the nav2 parameters file that specifies nav2 to use RPP in place
    # of DWB is located at: pkg_share/config/nav2_params.yaml
    world_file = os.path.join(pkg_share, 'worlds', 'object_fetching.world')
    map_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    marker_config_file = os.path.join(pkg_share, 'config', 'marker_config.yaml')

    # TurtleBot3 Waffle SDF model path > directly from ros2...
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
    # path for rviz2 is found in this way::
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2', # we rename this node to rviz2..
        # following saves the settings we do in the rviz2 config file and 
        # uses them for -d(display).
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen', # sends all the logs and error messages form RViz 
        # directly to the terminal for catching errors... 
        
        # Following is a condition i.e. it requires us to explicitly tell
        # ros2 launch ... rviz:=true or ros2 launch ... rviz:=false in command line
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 2: Spawn TurtleBot3 Waffle (after 3s, giving Gazebo time to start)
    # ══════════════════════════════════════════════════════════════════════
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros', # look in the system's gazebo_ros folder for the
                # package for this node.
                executable='spawn_entity.py',
                # the executable(made by gazebo itself) is also present in that same folder 
                # and not in the package folder...
                arguments=[
                    '-entity', 'turtlebot3_waffle',
                    '-file', tb3_model_path,
                    '-x', '0', '-y', '0', '-z', '0.1'
                ],
                output='screen' # put all the logs on the terminal
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 3: Robot State Publisher (publishes TF transforms)
    # The turtlebot3_waffle.urdf is actually a xacro file — must process it
    # with xacro first, otherwise robot_description is invalid and base_link
    # TF is never published (Nav2 will then fail to activate).
    # ══════════════════════════════════════════════════════════════════════
    ## this also given by turtlebot3 only...
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
    # syntax: 
    # robot_state_publisher = Node(
    #     package='robot_state_publisher', -> package name(***)
    #     executable='robot_state_publisher', -> same name as from setup.py
    # )
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
    # launch description 
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
    # return launch description(minimal requirement of the launch file)
    return ld
