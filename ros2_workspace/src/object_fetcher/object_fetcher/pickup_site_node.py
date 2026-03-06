"""
pickup_site_node.py
-------------------
WHAT IT DOES:
  1. On startup, generates 3 RANDOM positions inside the room
  2. Spawns the pickup objects (platform + item) in Gazebo at those positions
  3. Exposes get_zone_positions() so main_controller can read the locations
  4. Advertises /confirm_pickup service to confirm object identity

Every time the simulation restarts, the 3 pickup boxes appear in NEW locations.
Walls, barrels, and other static obstacles never move.
"""

import random
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity


# ── Object manifest ──
ZONE_OBJECTS = {
    'zone_1': {'name': 'Red Box',        'marker_id': 0, 'weight': 1.2},
    'zone_2': {'name': 'Blue Cube',      'marker_id': 1, 'weight': 0.8},
    'zone_3': {'name': 'Green Cylinder', 'marker_id': 2, 'weight': 1.5},
}

# ── Zone visual definitions (SDF snippets) ──
ZONE_SDF = {
    'zone_1': {
        'platform_color': '0.0 0.8 0.0 1',   # green platform
        'object_sdf': '''
          <collision name="c"><geometry><box><size>0.15 0.15 0.15</size></box></geometry></collision>
          <visual name="v"><geometry><box><size>0.15 0.15 0.15</size></box></geometry>
            <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>''',
        'object_z': 0.1,
    },
    'zone_2': {
        'platform_color': '0.0 0.3 0.9 1',   # blue platform
        'object_sdf': '''
          <collision name="c"><geometry><box><size>0.15 0.15 0.15</size></box></geometry></collision>
          <visual name="v"><geometry><box><size>0.15 0.15 0.15</size></box></geometry>
            <material><ambient>0.1 0.3 0.9 1</ambient><diffuse>0.1 0.3 0.9 1</diffuse></material></visual>''',
        'object_z': 0.1,
    },
    'zone_3': {
        'platform_color': '0.9 0.1 0.1 1',   # red platform
        'object_sdf': '''
          <collision name="c"><geometry><cylinder><radius>0.07</radius><length>0.2</length></cylinder></geometry></collision>
          <visual name="v"><geometry><cylinder><radius>0.07</radius><length>0.2</length></cylinder></geometry>
            <material><ambient>0.1 0.8 0.1 1</ambient><diffuse>0.1 0.8 0.1 1</diffuse></material></visual>''',
        'object_z': 0.1,
    },
}

# ── Exclusion zones: rectangles (cx, cy, half_w, half_h) the sampler must avoid ──
# IMPORTANT: Use generous padding so zones don't end up in narrow corridors
# where the robot gets stuck between a wall and the zone's spawned objects.
EXCLUSION_RECTS = [
    # Outer walls (with margin)
    (0, 5.0, 5.2, 0.6),     # north
    (0, -5.0, 5.2, 0.6),    # south
    (-5.0, 0, 0.6, 5.2),    # west
    (5.0, 0, 0.6, 5.2),     # east
    # Interior partitions — WIDE exclusion (1.0 m each side of the wall)
    # so zones never spawn in narrow corridors next to partitions
    (-3.0, -1.5, 2.8, 1.0),  # lower horizontal partition (x=-5 to x=-1, y=-1.5)
    (-3.0,  1.5, 2.8, 1.0),  # upper horizontal partition (x=-5 to x=-1, y=+1.5)
    (1.5, -1.5, 1.0, 4.0),   # right vertical partition (x=1.5, y=-5 to y=+2)
    (1.5,  4.25, 1.0, 1.3),  # right vertical upper (x=1.5, y=3.5 to y=5)
    # Static obstacles (generous padding)
    (0.0, -3.0, 0.9, 0.7),   # table
    (3.5, 0.0, 0.7, 0.7),    # barrel_1
    (-0.5, 3.5, 0.7, 0.7),   # crate_1
    (3.5, -3.0, 0.7, 0.7),   # barrel_2
    (-1.8, 0.0, 0.7, 0.7),   # crate_2
    (-0.5, -4.0, 0.8, 0.6),  # box_south
    (-1.3, -2.8, 0.6, 0.7),  # barrel_3
    # Home base / drop-off (0, 0)
    (0.0, 0.0, 1.0, 1.0),
]


def _point_in_rect(px, py, cx, cy, hw, hh):
    return abs(px - cx) < hw and abs(py - cy) < hh


def _is_valid_position(x, y, existing_positions, min_sep=1.5):
    for (cx, cy, hw, hh) in EXCLUSION_RECTS:
        if _point_in_rect(x, y, cx, cy, hw, hh):
            return False
    if abs(x) > 4.3 or abs(y) > 4.3:
        return False
    for (ex, ey) in existing_positions:
        if math.sqrt((x - ex)**2 + (y - ey)**2) < min_sep:
            return False
    return True


def _generate_random_positions(count=3, max_attempts=500):
    positions = []
    for _ in range(count):
        for _ in range(max_attempts):
            x = random.uniform(-4.3, 4.3)
            y = random.uniform(-4.3, 4.3)
            if _is_valid_position(x, y, positions):
                positions.append((x, y))
                break
        else:
            fallback = [(-2.5, -3.0), (-2.5, 3.0), (3.0, 3.0)]
            positions.append(fallback[len(positions) % len(fallback)])
    return positions


def _make_platform_sdf(color_rgba):
    return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="platform"><static>true</static>
    <link name="link">
      <visual name="v"><geometry><box><size>0.6 0.6 0.02</size></box></geometry>
        <material><ambient>{color_rgba}</ambient><diffuse>{color_rgba}</diffuse></material></visual>
    </link>
  </model>
</sdf>'''


def _make_object_sdf(inner_link_sdf):
    return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="obj"><static>true</static>
    <link name="link">
      {inner_link_sdf}
    </link>
  </model>
</sdf>'''


def _make_marker_sdf():
    return '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="marker"><static>true</static>
    <link name="link">
      <visual name="v"><geometry><box><size>0.2 0.2 0.01</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
    </link>
  </model>
</sdf>'''


class PickupSiteNode(Node):
    def __init__(self):
        super().__init__('mc_pickup_site')

        # ── Generate random positions for this run ──
        self._zone_positions = {}
        positions = _generate_random_positions(3)
        zone_ids = ['zone_1', 'zone_2', 'zone_3']
        zone_names = ['Zone Alpha', 'Zone Beta', 'Zone Gamma']
        for i, zid in enumerate(zone_ids):
            x, y = positions[i]
            self._zone_positions[zid] = {
                'x': round(x, 2),
                'y': round(y, 2),
                'name': zone_names[i],
            }
            self.get_logger().info(
                f'🎲 {zone_names[i]} ({zid}) → random position ({x:.2f}, {y:.2f})'
            )

        # ── Gazebo spawn client ──
        # Must use ReentrantCallbackGroup so the service response can be
        # processed while the timer callback (that calls _spawn_model) is running
        self._spawn_cb_group = ReentrantCallbackGroup()
        self._spawn_client = self.create_client(
            SpawnEntity, '/spawn_entity',
            callback_group=self._spawn_cb_group
        )
        self._spawn_done = False

        # Service: /confirm_pickup
        self._confirm_srv = self.create_service(
            Trigger, '/confirm_pickup', self._confirm_pickup_callback
        )

        # Publisher: current object at pickup site
        self._object_pub = self.create_publisher(
            String, '/pickup_site/current_object', 10
        )

        self.current_zone_id = None
        self.create_timer(1.0, self._broadcast_current_object)

        # Spawn objects after a short delay (Gazebo needs time)
        self.create_timer(2.0, self._spawn_all_zones_once,
                          callback_group=self._spawn_cb_group)

        self.get_logger().info(
            'PickupSiteNode ready. '
            'Service: /confirm_pickup | '
            f'Known zones: {list(ZONE_OBJECTS.keys())}'
        )

    # ─── Public API (called by main_controller) ───

    def get_zone_positions(self):
        """Return dict of zone_id → {x, y, name} for the randomly generated positions."""
        return self._zone_positions

    def set_active_zone(self, zone_id):
        self.current_zone_id = zone_id
        obj = ZONE_OBJECTS.get(zone_id, {})
        self.get_logger().info(
            f'Active zone set to: {zone_id} '
            f'(object: {obj.get("name", "Unknown")})'
        )

    # ─── Gazebo Spawning ───

    def _spawn_all_zones_once(self):
        if self._spawn_done:
            return
        self._spawn_done = True

        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'Gazebo /spawn_entity service not available — '
                'objects will NOT appear in simulation.'
            )
            return

        for zid, pos in self._zone_positions.items():
            x, y = pos['x'], pos['y']
            sdf_info = ZONE_SDF[zid]

            self._spawn_model(
                f'{zid}_platform', _make_platform_sdf(sdf_info['platform_color']),
                x, y, 0.01
            )
            self._spawn_model(
                f'{zid}_marker', _make_marker_sdf(),
                x, y, 0.35
            )
            self._spawn_model(
                f'{zid}_object', _make_object_sdf(sdf_info['object_sdf']),
                x, y, sdf_info['object_z']
            )

        self.get_logger().info('✅ All 3 pickup zones spawned in Gazebo!')

    def _spawn_model(self, name, sdf_xml, x, y, z):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf_xml
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)
        req.reference_frame = 'world'

        future = self._spawn_client.call_async(req)
        deadline = time.monotonic() + 10.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn(f'Timeout spawning {name}')
                return
            time.sleep(0.05)

        result = future.result()
        if result and result.success:
            self.get_logger().info(f'Spawned {name} at ({x:.2f}, {y:.2f}, {z:.2f})')
        else:
            msg = result.status_message if result else 'no response'
            self.get_logger().warn(f'Failed to spawn {name}: {msg}')

    # ─── Pickup Confirmation ───

    def _confirm_pickup_callback(self, request, response):
        if self.current_zone_id is None:
            response.success = False
            response.message = 'No active zone set.'
            return response

        obj = ZONE_OBJECTS.get(self.current_zone_id)
        if obj is None:
            response.success = False
            response.message = f'Unknown zone: {self.current_zone_id}'
            return response

        response.success = True
        response.message = (
            f'Object confirmed: {obj["name"]} | '
            f'Marker ID: {obj["marker_id"]} | '
            f'Weight: {obj["weight"]}kg'
        )
        self.get_logger().info(f'✅ Pickup confirmed: {response.message}')
        return response

    def _broadcast_current_object(self):
        if self.current_zone_id:
            obj = ZONE_OBJECTS.get(self.current_zone_id, {})
            msg = String()
            msg.data = f"{self.current_zone_id}: {obj.get('name', 'Unknown')}"
            self._object_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor
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


if __name__ == '__main__':
    main()
