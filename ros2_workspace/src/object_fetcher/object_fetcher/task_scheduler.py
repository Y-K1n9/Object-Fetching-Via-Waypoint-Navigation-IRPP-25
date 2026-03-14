"""Decides which pickup zone to visit next (nearest-first or by priority)."""

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class TaskScheduler(Node):
    def __init__(self):
        super().__init__('mc_task_scheduler')
        if not self.has_parameter('scheduling_strategy'):
            # this is a way to declare parameter scheduling_strategy
            # the default value is set to distance....
            # we have declared it as a parameter as declaring
            # it as a parameter allows us to be able to change the
            # task scheduling strategy at runtime....
            # :::BONUS:::
            # here is the point where we set the task 
            # scheduling strategy to be bastd on distance...
            self.declare_parameter('scheduling_strategy', 'distance')
        # this is the way to get the parameter's value from the parameter 
        # object returned by self.get_parameter('parameter_name')
        self.strategy = self.get_parameter('scheduling_strategy').value

        pkg = get_package_share_directory('object_fetcher')
        path = os.path.join(pkg, 'config', 'waypoints.yaml')
        try:
            with open(path) as f:
                self.waypoints = yaml.safe_load(f)
        except Exception:
            self.waypoints = {
                'pickup_zones': [],
                'dropoff_zone': {'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}}}

        self.completed_zones = set()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.get_logger().info(f'TaskScheduler ready ({self.strategy})')

    def _dist(self, z):
        """Euclidean distance from robot to zone."""
        return math.hypot(z['position']['x'] - self.robot_x,
                          z['position']['y'] - self.robot_y)

    def get_next_task(self, robot_x=None, robot_y=None):
        """Return the next zone dict to visit, or None if all done."""
        if robot_x is not None:
            self.robot_x = robot_x
        if robot_y is not None:
            self.robot_y = robot_y

        remaining = [z for z in self.waypoints['pickup_zones']
                     if z['id'] not in self.completed_zones]
        if not remaining:
            return None

        if self.strategy == 'distance':
            remaining.sort(key=self._dist)
            self.get_logger().info(
                f'Nearest: {remaining[0]["id"]} ({self._dist(remaining[0]):.1f}m)')
        else:
            remaining.sort(key=lambda z: z.get('priority', 99))
        return remaining[0]

    def mark_completed(self, zone_id):
        self.completed_zones.add(zone_id)
        left = len(self.waypoints['pickup_zones']) - len(self.completed_zones)
        self.get_logger().info(f'{zone_id} done — {left} remaining')

    def get_dropoff_zone(self):
        return self.waypoints.get('dropoff_zone',
                                  {'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}})

    def reset(self):
        self.completed_zones.clear()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TaskScheduler())
    rclpy.shutdown()