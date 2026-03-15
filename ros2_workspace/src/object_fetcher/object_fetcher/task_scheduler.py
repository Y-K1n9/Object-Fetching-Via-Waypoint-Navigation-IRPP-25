"""Decides which pickup zone to visit next (nearest-first or by priority)."""

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class TaskScheduler(Node):
    def __init__(self):
        super().__init__('mc_task_scheduler') # main controller task scheduler
        if not self.has_parameter('scheduling_strategy'):
            # this is a way to declare parameter scheduling_strategy
            # the default value is set to distance....
            # we have declared it as a parameter as declaring
            # it as a parameter allows us to be able to change the
            # task scheduling strategy at runtime....
            # :::BONUS:::
            # here is the point where we set the task 
            # scheduling strategy to be based on distance...
            self.declare_parameter('scheduling_strategy', 'distance')
            
        # this is the way to get the parameter's value from the parameter 
        # object returned by self.get_parameter('parameter_name')
        self.strategy = self.get_parameter('scheduling_strategy').value

        pkg = get_package_share_directory('object_fetcher') # we locate the path of the share directory i.e.: ./install/object_fetcher/share
                                                            # then inside that, in the ./install/object_fetcher/share/object_fetcher/config, 
                                                            # the yaml files are sourced
        path = os.path.join(pkg, 'config', 'waypoints.yaml')
        try:
            with open(path) as f:
                self.waypoints = yaml.safe_load(f) # we open the yaml file as python dict...
        except Exception:
        # in case of an exception, if waypoints.yaml threw exception while opening, 
        # the program hits this block and creates a minimalist map with no pickup zones
        # but one drop-off zone... So than instead of a full crash, atleast we get to 
        # see if there is an error
            self.waypoints = {
                'pickup_zones': [],
                'dropoff_zone': {
                    'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}
                }
            }

        self.completed_zones = set()
        self.robot_x = 0.0 # initializes robot's coordinates...
        self.robot_y = 0.0
        self.get_logger().info(f'TaskScheduler ready ({self.strategy})')

    def _dist(self, z):
        """Euclidean distance from robot to zone."""
        return math.hypot(
            z['position']['x'] - self.robot_x,
            z['position']['y'] - self.robot_y
        )
# :::BONUS:::
    def get_next_task(self, robot_x=None, robot_y=None):
        """Return the next zone dict to visit, or None if all done."""
        # the main node will be passing the robot's current 
        # position_x and position_y and if it passed something in the planning
        # phase, we use it for getting next task...
        if robot_x is not None:
            self.robot_x = robot_x
        if robot_y is not None:
            self.robot_y = robot_y
        # we maintain a list of zones that are in waypoints['pickup_zones'] but not in
        # completed_zones set...
        remaining = [z for z in self.waypoints['pickup_zones']
                     if z['id'] not in self.completed_zones]
        # if no remaining zone, we return that next task is none and
        # Mission is complete transition will be done by the main_controller
        if not remaining:
            return None

        if self.strategy == 'distance':
            remaining.sort(key=self._dist) # we sort the list of remaining zones(list of dictionaries)
            # based on the zone dictionary's distance with (robot_x, robot_y)

            # the nearest node after sorting will be the zeroth element of the remaining list...
            self.get_logger().info(
                f'Nearest: {remaining[0]["id"]} ({self._dist(remaining[0]):.1f}m)'
            )
        else: 
            # Priority based...
            # we used a lambda function to get the key in the case strategy was not
            # distance from (robot_x, robot_y), this lambda function finds the 
            # value of the priority of all the zones, if there is no priority of any specific zone,
            # then it assigns priority = 99 (a very low priority...), by default, waypoints.yaml 
            # has the priority for each zone... (b->r->g, but not used...)
            remaining.sort(key=lambda z: z.get('priority', 99))
        return remaining[0]

    def mark_completed(self, zone_id):
        # since this node is solely responsible for the planning, this node 
        # has to maintain the completed_zones list and respond correctly for 
        # each call it gets, so whenever 
        # 1. any delivery is done,
        # 2. navigation to pickup zone failed all 4 tries,
        # 3. navigation to dropoff zone failed all 4 tries,
        # The main_controller calls this mark_completed function so 
        # that the task_scheduler node is up-to-date with all the 
        # happenings in each run... 
        self.completed_zones.add(zone_id)
        left = len(self.waypoints['pickup_zones']) - len(self.completed_zones)
        self.get_logger().info(f'{zone_id} done - {left} remaining')

    def get_dropoff_zone(self):
    # this returns the dropoff_zone dictionary of dictionary,
    # by default (0,0,0)
        return self.waypoints.get('dropoff_zone',
                                  {'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}})

    def reset(self):
        # reset is asserted just before IDLE->PLANNING
        # transition, just at start of simulation...
        self.completed_zones.clear()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TaskScheduler())
    rclpy.shutdown()