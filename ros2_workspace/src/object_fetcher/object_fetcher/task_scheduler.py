"""
task_scheduler.py
-----------------
WHAT IT DOES: Decides which pickup zone the robot should visit next.

TWO STRATEGIES:
  1. "distance" - always go to the NEAREST unvisited zone (BONUS MARKS!)
  2. "priority"  - go in order of priority number (1 = most important)

THINK OF IT LIKE: A delivery driver who always picks the closest package first.
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os


class TaskScheduler(Node):
    def __init__(self):
        super().__init__('mc_task_scheduler')

        # Parameter: which strategy to use ('distance' or 'priority')
        if not self.has_parameter('scheduling_strategy'):
            self.declare_parameter('scheduling_strategy', 'distance')
        self.strategy = self.get_parameter('scheduling_strategy').value

        # Load waypoints from YAML config file
        pkg_share = get_package_share_directory('object_fetcher')
        waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
        self.waypoints = self._load_waypoints(waypoints_file)

        # Track which zones we've already completed
        self.completed_zones = set()

        # Robot's current position (updated externally or estimated)
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.get_logger().info(
            f'TaskScheduler ready. Strategy: {self.strategy}. '
            f'Zones loaded: {[z["id"] for z in self.waypoints["pickup_zones"]]}'
        )

    def _load_waypoints(self, filepath):
        """Load the waypoints YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            self.get_logger().info(f'Loaded waypoints from {filepath}')
            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            # Return a minimal fallback so the node doesn't crash
            return {
                'pickup_zones': [],
                'dropoff_zone': {'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}}
            }

    def _distance(self, zone):
        """Calculate Euclidean distance from robot to a zone."""
        dx = zone['position']['x'] - self.robot_x
        dy = zone['position']['y'] - self.robot_y
        return math.sqrt(dx * dx + dy * dy)

    def get_next_task(self, robot_x=None, robot_y=None):
        """
        Returns the next pickup zone to visit, or None if all done.

        Args:
            robot_x, robot_y: Current robot position (for distance strategy)

        Returns:
            dict with zone info, or None if mission complete
        """
        if robot_x is not None:
            self.robot_x = robot_x
        if robot_y is not None:
            self.robot_y = robot_y

        # Filter out zones we've already completed
        remaining = [
            z for z in self.waypoints['pickup_zones']
            if z['id'] not in self.completed_zones
        ]

        if not remaining:
            self.get_logger().info('All zones completed! Mission done.')
            return None

        # Apply scheduling strategy
        if self.strategy == 'distance':
            # BONUS: Sort by distance, pick the closest one
            remaining.sort(key=self._distance)
            chosen = remaining[0]
            dist = self._distance(chosen)
            self.get_logger().info(
                f'[DISTANCE] Next zone: {chosen["id"]} '
                f'(distance: {dist:.2f}m from robot)'
            )
        else:
            # Priority: sort by priority number (lower = higher priority)
            remaining.sort(key=lambda z: z.get('priority', 99))
            chosen = remaining[0]
            self.get_logger().info(
                f'[PRIORITY] Next zone: {chosen["id"]} '
                f'(priority: {chosen.get("priority", "?")})'
            )

        return chosen

    def mark_completed(self, zone_id):
        """Call this after successfully delivering from a zone."""
        self.completed_zones.add(zone_id)
        self.get_logger().info(
            f'Zone {zone_id} marked complete. '
            f'Remaining: {len(self.waypoints["pickup_zones"]) - len(self.completed_zones)}'
        )

    def get_dropoff_zone(self):
        """Returns the home base / drop-off coordinates."""
        return self.waypoints.get('dropoff_zone', {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        })

    def reset(self):
        """Reset all progress (used when restarting the mission)."""
        self.completed_zones.clear()
        self.get_logger().info('Task scheduler reset.')


def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
