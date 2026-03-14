"""Mission state machine — orchestrates the pickup / detect / dropoff cycle."""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
# we import Trigger to create std_srvs.srv/Trigger type of service
from std_srvs.srv import Trigger
from std_msgs.msg import Int32MultiArray, String, ColorRGBA
from visualization_msgs.msg import Marker

from object_fetcher.task_scheduler import TaskScheduler
from object_fetcher.waypoint_navigator import WaypointNavigator
from object_fetcher.pickup_site_node import PickupSiteNode


class MissionState:
    IDLE                  = 'IDLE'
    PLANNING              = 'PLANNING'
    NAVIGATING_TO_PICKUP  = 'NAVIGATING_TO_PICKUP'
    DETECTING_MARKER      = 'DETECTING_MARKER'
    CONFIRMING_PICKUP     = 'CONFIRMING_PICKUP'
    NAVIGATING_TO_DROPOFF = 'NAVIGATING_TO_DROPOFF'
    DELIVERING            = 'DELIVERING'
    COMPLETED             = 'COMPLETED'
    ERROR                 = 'ERROR'


# Approach directions tried on successive retries (front, left, right, rear)
_ANGLES = [0.0, math.pi / 2, -math.pi / 2, math.pi]


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Parameters
        # this is a way to declare parameter scheduling_strategy
        # the default value is set to distance....
        # we have declared it as a parameter as declaring
        # it as a parameter allows us to be able to change the
        # task scheduling strategy at runtime....
        # :::BONUS:::
        # here is the point where we set the task 
        # scheduling strategy to be bastd on distance...
        if not self.has_parameter('scheduling_strategy'):
            self.declare_parameter('scheduling_strategy', 'distance')
        # default amt of time we wait near the pickup zone before
        # giving up is 15 sec.
        if not self.has_parameter('marker_timeout_sec'):
            self.declare_parameter('marker_timeout_sec', 15.0)
        # we by default use simulation time
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        # we use multiple -p in command line command for setting
        # multiple parameters:
        # ros2 run object_fetcher --ros-args -p marker_timeout_sec:=10 -p scheduling_strategy:=priority

        self.scheduling_strategy = self.get_parameter('scheduling_strategy').value
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value

        # Sub-nodes (shared process — added to the same executor in main())
        self.scheduler = TaskScheduler()
        self.scheduler.strategy = self.scheduling_strategy
        self.navigator = WaypointNavigator()
        self.pickup_site = PickupSiteNode()

        # Inject the random spawn positions into the scheduler's waypoints
        self._apply_random_zone_positions()

        # State machine variables
        self.state = MissionState.IDLE
        self.current_task = None
        self.detected_markers = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self._detect_start_time = None
        self._nav_in_progress = False
        self._pickup_retries = 0
        self._dropoff_retries = 0

        # ROS interfaces
        ##################################
        # self.create_service(
        #       Type of service,
        #       name of service,
        #       callback function  (here self._start_cb and _stop_cb)
        # )
        self.create_service(Trigger, '/start_mission', self._start_cb)
        self.create_service(Trigger, '/stop_mission', self._stop_cb)
        ###################################
        #################################
        # self.create_subscription(
        #       Datatype,
        #       topic,
        #       callback function       (here self._marker_cb)
        #       queue size
        # )
        self.create_subscription(
            Int32MultiArray, '/aruco/marker_ids', self._marker_cb, 10)
        #################################
        self._state_pub = self.create_publisher(String, '/mission_state', 10)
        self._marker_pub = self.create_publisher(
            Marker, '/mission_status_marker', 10
        )

        # Mission loop at 2 Hz
        # time = 0.5 sec between two successive calls of
        # required self.mission_loop() function...
        self.create_timer(
            0.5,
            self._mission_loop
        )
        self.get_logger().info(
            f'MainController ready (strategy={self.scheduling_strategy})'
        )

    # ── Inject random spawn positions into scheduler ──

    def _apply_random_zone_positions(self):
        positions = self.pickup_site.get_zone_positions()
        if not positions:
            return
        for zone in self.scheduler.waypoints['pickup_zones']:
            zid = zone['id']
            if zid in positions:
                zone['position']['x'] = positions[zid]['x']
                zone['position']['y'] = positions[zid]['y']
                zone['name'] = positions[zid]['name']
                self.get_logger().info(
                    f'{zid} -> ({positions[zid]["x"]}, {positions[zid]["y"]})')

    # ── Service callbacks ──

    def _start_cb(self, request, response):
        if self.state not in (MissionState.IDLE, MissionState.COMPLETED,
                              MissionState.ERROR):
            response.success = False
            response.message = f'Already running ({self.state})'
            return response
        self.scheduler.reset()
        self._pickup_retries = self._dropoff_retries = 0
        self._transition_to(MissionState.PLANNING)
        response.success = True
        response.message = 'Mission started'
        return response

    def _stop_cb(self, request, response):
        self.navigator.cancel_navigation()
        self._transition_to(MissionState.IDLE)
        response.success = True
        response.message = 'Mission stopped'
        return response

    def _marker_cb(self, msg):
    # callback needs msg as argument
        self.detected_markers = msg.data

    # ── Main loop (2 Hz) ──

    def _mission_loop(self):
        # every 0.5 sec, this function is called
        # from line:
        # self.create_timer(0.5, self._mission_loop)
        # and it does the following, every 0.5 sec:
        # It publishes it's current state
        # as a String
        state_msg = String()
        state_msg.data = self.state
        self._state_pub.publish(state_msg)
        # and calls the function corresp.
        # to the state or it calls the state
        # handler function....
        handlers = {
            MissionState.PLANNING:              self._do_planning,
            MissionState.NAVIGATING_TO_PICKUP:  self._do_nav_pickup,
            MissionState.DETECTING_MARKER:      self._do_detect,
            MissionState.CONFIRMING_PICKUP:     self._do_confirm,
            MissionState.NAVIGATING_TO_DROPOFF: self._do_nav_dropoff,
            MissionState.DELIVERING:            self._do_deliver,
        }
        h = handlers.get(self.state)
        if h:
            h()

    # ── State handlers ──

    def _do_planning(self):
        task = self.scheduler.get_next_task(self.robot_x, self.robot_y)
        if task is None:
            self.get_logger().info('ALL ZONES COMPLETED — mission successful!')
            self._transition_to(MissionState.COMPLETED)
            return
        self.current_task = task
        self.pickup_site.set_active_zone(task['id'])
        self.get_logger().info(f'Next target: {task.get("name", task["id"])}')
        self._transition_to(MissionState.NAVIGATING_TO_PICKUP)

    def _do_nav_pickup(self):
        if self._nav_in_progress or not self.current_task:
            return
        pos = self.current_task['position']
        angle = _ANGLES[self._pickup_retries % len(_ANGLES)]
        nav_x, nav_y = self._approach_point(pos['x'], pos['y'], 0.4, angle)
        facing = math.atan2(pos['y'] - nav_y, pos['x'] - nav_x)

        self._nav_in_progress = True
        ok = self.navigator.navigate_to(nav_x, nav_y, facing)
        self._nav_in_progress = False

        if ok:
            self.robot_x, self.robot_y = nav_x, nav_y
            self._pickup_retries = 0
            self._transition_to(MissionState.DETECTING_MARKER)
        else:
            self._pickup_retries += 1
            if self._pickup_retries < len(_ANGLES):
                self.get_logger().warn(
                    f'Pickup nav failed — retry {self._pickup_retries}')
                self._transition_to(MissionState.NAVIGATING_TO_PICKUP)
            else:
                self.get_logger().error('All pickup approaches failed — skipping')
                self._pickup_retries = 0
                self.scheduler.mark_completed(self.current_task['id'])
                self._transition_to(MissionState.PLANNING)

    def _do_detect(self):
        expected = self.current_task.get('marker_id')
        if self._detect_start_time is None:
            self._detect_start_time = time.time()
            self.get_logger().info(f'Looking for marker {expected}...')

        if expected in self.detected_markers:
            self.get_logger().info(f'Marker {expected} detected!')
            self._detect_start_time = None
            self._transition_to(MissionState.CONFIRMING_PICKUP)
            return

        if time.time() - self._detect_start_time >= self.marker_timeout:
            self.get_logger().warn('Marker timeout — proceeding anyway')
            self._detect_start_time = None
            self._transition_to(MissionState.CONFIRMING_PICKUP)

    def _do_confirm(self):
        name = self.current_task.get('name', self.current_task['id'])
        self.get_logger().info(f'Pickup confirmed at {name}')
        self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)

    def _do_nav_dropoff(self):
        if self._nav_in_progress:
            return
        dropoff = self.scheduler.get_dropoff_zone()['position']
        angle = _ANGLES[self._dropoff_retries % len(_ANGLES)]
        nav_x, nav_y = self._approach_point(
            dropoff['x'], dropoff['y'], 0.7, angle)
        heading = math.atan2(nav_y - self.robot_y, nav_x - self.robot_x)

        self._nav_in_progress = True
        ok = self.navigator.navigate_to(nav_x, nav_y, heading)
        self._nav_in_progress = False

        if ok:
            self.robot_x, self.robot_y = nav_x, nav_y
            self._dropoff_retries = 0
            self._transition_to(MissionState.DELIVERING)
        else:
            self._dropoff_retries += 1
            if self._dropoff_retries < len(_ANGLES):
                self.get_logger().warn(
                    f'Dropoff nav failed — retry {self._dropoff_retries}')
                self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)
            else:
                self.get_logger().error('All dropoff approaches failed — skipping')
                self._dropoff_retries = 0
                self.scheduler.mark_completed(self.current_task['id'])
                self.current_task = None
                self._transition_to(MissionState.PLANNING)

    def _do_deliver(self):
        if self._nav_in_progress:
            return
        self._nav_in_progress = True
        name = self.current_task.get('name', self.current_task['id'])
        done = len(self.scheduler.completed_zones) + 1
        self.get_logger().info(f'Delivered from {name} ({done} done)')
        self.scheduler.mark_completed(self.current_task['id'])
        self.current_task = None
        time.sleep(1.0)
        self._transition_to(MissionState.PLANNING)
        self._nav_in_progress = False

    # ── Helpers ──

    def _approach_point(self, gx, gy, dist, angle_offset=0.0):
        """Point `dist` metres from goal, rotated by `angle_offset` around it."""
        dx, dy = gx - self.robot_x, gy - self.robot_y
        d = math.hypot(dx, dy)
        if d <= dist:
            return max(-4.3, min(4.3, gx)), max(-4.3, min(4.3, gy))
        bx, by = -dx / d, -dy / d                       # goal -> robot
        c, s = math.cos(angle_offset), math.sin(angle_offset)
        px = gx + (bx * c - by * s) * dist
        py = gy + (bx * s + by * c) * dist
        return max(-4.3, min(4.3, px)), max(-4.3, min(4.3, py))

    def _transition_to(self, new_state):
        old = self.state
        self.state = new_state
        self.get_logger().info(f'{old} -> {new_state}')
        self._publish_overlay()

    def _publish_overlay(self):
        """RViz text marker above the robot showing mission progress."""
        total = len(self.scheduler.waypoints.get('pickup_zones', []))
        done = len(self.scheduler.completed_zones)
        labels = {
            MissionState.IDLE: 'IDLE',
            MissionState.PLANNING: 'PLANNING',
            MissionState.NAVIGATING_TO_PICKUP: 'TO PICKUP',
            MissionState.DETECTING_MARKER: 'DETECTING',
            MissionState.CONFIRMING_PICKUP: 'CONFIRMING',
            MissionState.NAVIGATING_TO_DROPOFF: 'TO DROPOFF',
            MissionState.DELIVERING: 'DELIVERING',
            MissionState.COMPLETED: 'MISSION COMPLETE',
            MissionState.ERROR: 'ERROR',
        }
        label = labels.get(self.state, self.state)
        zone = f"  [{self.current_task.get('name', '')}]" if self.current_task else ''

        # State-based colour
        color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        if self.state == MissionState.COMPLETED:
            color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)
        elif self.state == MissionState.ERROR:
            color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        elif 'NAVIGATING' in self.state:
            color = ColorRGBA(r=0.3, g=0.6, b=1.0, a=1.0)
        elif self.state in (MissionState.DETECTING_MARKER,
                            MissionState.CONFIRMING_PICKUP):
            color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)

        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mission_status'
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 0.8
        m.pose.orientation.w = 1.0
        m.scale.z = 0.25
        m.color = color
        m.text = f'{label}{zone}\n{done}/{total} zones done'
        self._marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    ctrl = MainController()
    for node in [ctrl, ctrl.navigator, ctrl.scheduler, ctrl.pickup_site]:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in [ctrl.navigator, ctrl.scheduler, ctrl.pickup_site, ctrl]:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
