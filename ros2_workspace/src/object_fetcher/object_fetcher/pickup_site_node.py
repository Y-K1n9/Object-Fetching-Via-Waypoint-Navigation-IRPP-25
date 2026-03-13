"""Spawns pickup zones at random positions in Gazebo each run."""

import random
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from gazebo_msgs.srv import SpawnEntity
from rclpy.executors import MultiThreadedExecutor

# Zone object definitions (name, ArUco marker ID, weight)
ZONE_OBJECTS = {
    'zone_1': {'name': 'Red Box',        'marker_id': 0, 'weight': 1.2},
    'zone_2': {'name': 'Blue Cube',      'marker_id': 1, 'weight': 0.8},
    'zone_3': {'name': 'Green Cylinder', 'marker_id': 2, 'weight': 1.5},
}

# SDF visual snippets per zone (platform colour + object geometry)
ZONE_SDF = {
    'zone_1': {
        'platform_color': '0.0 0.8 0.0 1',
        'object_sdf': '<visual name="v"><geometry><box><size>0.15 0.15 0.15</size></box></geometry>'
                       '<material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>',
    },
    'zone_2': {
        'platform_color': '0.0 0.3 0.9 1',
        'object_sdf': '<visual name="v"><geometry><box><size>0.15 0.15 0.15</size></box></geometry>'
                       '<material><ambient>0.1 0.3 0.9 1</ambient><diffuse>0.1 0.3 0.9 1</diffuse></material></visual>',
    },
    'zone_3': {
        'platform_color': '0.9 0.1 0.1 1',
        'object_sdf': '<visual name="v"><geometry><cylinder><radius>0.07</radius><length>0.2</length></cylinder></geometry>'
                       '<material><ambient>0.1 0.8 0.1 1</ambient><diffuse>0.1 0.8 0.1 1</diffuse></material></visual>',
    },
}

# Exclusion rectangles (cx, cy, half_w, half_h) — zones must not spawn here
EXCLUSION_RECTS = [
    (0, 5.0, 5.2, 0.6), (0, -5.0, 5.2, 0.6),              # N/S walls
    (-5.0, 0, 0.6, 5.2), (5.0, 0, 0.6, 5.2),               # W/E walls
    (-3.0, -1.5, 2.8, 1.0), (-3.0, 1.5, 2.8, 1.0),         # horiz. partitions
    (1.5, -2.0, 1.0, 3.5), (1.5, 4.5, 1.0, 1.0),           # vert. partitions
    (0.0, -3.0, 0.9, 0.7), (3.5, 0.0, 0.7, 0.7),           # table, barrel_1
    (-0.5, 3.5, 0.7, 0.7), (3.5, -3.0, 0.7, 0.7),          # crate_1, barrel_2
    (-1.8, 0.0, 0.7, 0.7), (-0.5, -4.0, 0.8, 0.6),         # crate_2, box_south
    (-1.3, -2.8, 0.6, 0.7),                                 # barrel_3
    (0.0, 0.0, 1.0, 1.0),                                   # home base
]


def _is_valid(x, y, existing, min_sep=1.5):
    """Check position is inside room, outside exclusion zones, and far from others."""
    if abs(x) > 4.3 or abs(y) > 4.3:
        return False
    for cx, cy, hw, hh in EXCLUSION_RECTS:
        if abs(x - cx) < hw and abs(y - cy) < hh:
            return False
    return all(math.hypot(x - ex, y - ey) >= min_sep for ex, ey in existing)


def _random_positions(count=3):
    """Generate `count` valid random positions, with fallbacks."""
    fallbacks = [(-2.5, -3.0), (-2.5, 3.0), (3.0, 3.0)]
    positions = []
    for i in range(count):
        for _ in range(500):
            x, y = random.uniform(-4.3, 4.3), random.uniform(-4.3, 4.3)
            if _is_valid(x, y, positions):
                positions.append((x, y))
                break
        else:
            positions.append(fallbacks[i])
    return positions


def _wrap_sdf(inner):
    """Wrap link-level SDF content in a static model."""
    return (f'<?xml version="1.0"?><sdf version="1.6">'
            f'<model name="m"><static>true</static>'
            f'<link name="link">{inner}</link></model></sdf>')


def _platform_sdf(color):
    return _wrap_sdf(
        f'<visual name="v"><geometry><box><size>0.6 0.6 0.02</size></box></geometry>'
        f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')


def _marker_sdf(marker_id):
    """ArUco textured cube using Gazebo model:// material references."""
    return _wrap_sdf(
        f'<visual name="v"><geometry><box><size>0.15 0.15 0.15</size></box></geometry>'
        f'<material><script>'
        f'<uri>model://aruco_marker_{marker_id}/materials/scripts</uri>'
        f'<uri>model://aruco_marker_{marker_id}/materials/textures</uri>'
        f'<name>ArUco/Marker{marker_id}</name>'
        f'</script></material></visual>')


class PickupSiteNode(Node):
    def __init__(self):
        super().__init__('mc_pickup_site') # name of this node is mc_pickup_site

        # Generate random positions for this run
        self._zone_positions = {}
        for i, (x, y) in enumerate(_random_positions(3)):
            zoneID = f'zone_{i + 1}'
            self._zone_positions[zoneID] = {
                'x': round(x, 2),
                'y': round(y, 2),
                'name': f'Zone {i + 1}'
            }
            # make the user know at instant of initializtion where
            # the zones have been spawned
            self.get_logger().info(f'{zoneID} -> ({x:.2f}, {y:.2f})')

        # Gazebo spawn client (ReentrantCallbackGroup for async service calls)
        self._cb_group = ReentrantCallbackGroup()
        # syntax for::
        # self.create_client(
        #   service tType,
        #   service name,
        #   callback_group to control concurrency in callbacks
        #)
        self._spawn_client = self.create_client(
            SpawnEntity,
            '/spawn_entity',
            callback_group=self._cb_group
        )
        self._spawned = False
        # current_zone_id is the ID of zone which is 
        # used by robo to track which pickup zone the robot 
        # is focussing on currently.
        
        
        # while TaskScheduler has a list of all zones, current_zone_id
        # stores only one ID that the robot is currently driving towards or
        # inspecting.

        # when robo is navigating to dropoff, current_zone_id
        # usually stays set to ID of the zone it picked up from.
        # because robo needs to know where it's cargo came from until
        # it is oficially delivered.
        # in PLANNING state it is None for a brief amt of time....
        self.current_zone_id = None

        # Spawn after 2 s delay (gives Gazebo time to initialise)
        # self.create_timer is the standard way to 
        # to make a piece of code run repeatedly at a specific
        # frequency
        # syntax:
        # self.create_timer(
        #   time_period (float seconds),
        #   callback that we want to call every time_period seconds,
        #   callback_group to control concurrency in callbacks
        #)
        self.create_timer(
            2.0,
            self._spawn_all,
            callback_group=self._cb_group
        )
        self.get_logger().info('PickupSiteNode ready')

    # --- Public API (called by main_controller) ---

    def get_zone_positions(self):
        """Return {zone_id: {x, y, name}} for the random positions."""
        return self._zone_positions

    def set_active_zone(self, zone_id):
        self.current_zone_id = zone_id

    # --- Gazebo spawning ---

    def _spawn_all(self):
        """Spawn platforms, ArUco markers, and objects for every zone."""
        if self._spawned:
            return
        self._spawned = True
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Gazebo spawn service unavailable')
            return

        for zoneID, pos in self._zone_positions.items():
            x, y = pos['x'], pos['y']
            sdf = ZONE_SDF[zoneID]
            markerID = ZONE_OBJECTS[zoneID]['marker_id']
            self._spawn(f'{zoneID}_platform', _platform_sdf(sdf['platform_color']), x, y, 0.01)
            self._spawn(f'{zoneID}_marker',   _marker_sdf(markerID),                     x, y, 0.095)
            self._spawn(f'{zoneID}_object',   _wrap_sdf(sdf['object_sdf']),         x, y, 0.1) # we are simply passing the sdf['object_sdf'] to the 
            # _wrap_sdf function so that a simple cylinder's sdf can be generated.
        self.get_logger().info('All pickup zones spawned')

    def _spawn(self, name, sdf_xml, x, y, z):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf_xml
        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = float(z)
        request.reference_frame = 'world' # we want the _spawn to happen relative to the world's (0,0,0) not relative to the robot's base frame
        # The response is retrieved from the future object...
        # async call: it doesn't block the execution of the program as in
        # simple call, so we do call_async, or else all the thread that we are in 
        # currently will be blocked.


        # "future object in python" contains an object that may come in future...
        # result() function of future object has been used...
        future = self._spawn_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn(f'Timeout spawning {name}')
                return
            # we don't want the cpu to run for whole iteration of this loop we run it only for 
            # 50-50ms timestamps...
            time.sleep(0.05)
        res = future.result()
        if not res or not res.success:
            self.get_logger().warn(f'Failed to spawn {name}')


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    node = PickupSiteNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()