"""Mission state machine - orchestrates the pickup / detect / dropoff cycle."""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
# we import Trigger to create std_srvs.srv/Trigger type of service
from std_srvs.srv import Trigger
# we import Int32MultiArray, String, ColorRGBA types of messages from std_msgs.msg
from std_msgs.msg import Int32MultiArray, String, ColorRGBA
from visualization_msgs.msg import Marker
# we als
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


# Approach directions tried on successive retries (front(0deg), left(90deg), right(-90deg), rear(180deg))
_ANGLES = [0.0, math.pi / 2, -math.pi / 2, math.pi]


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller') # name of this node is main_controller

        # Parameters
        # this is a way to declare parameter scheduling_strategy
        # the default value is set to distance....
        # we have declared it as a parameter as declaring
        # it as a parameter allows us to be able to change the
        # task scheduling strategy at runtime....
        # :::BONUS:::
        # here is the point where we set the task 
        # scheduling strategy to be based on distance...
        if not self.has_parameter('scheduling_strategy'):
            self.declare_parameter('scheduling_strategy', 'distance')
        # default amt of time we wait near the pickup zone before
        # giving up is 15 sec.
        if not self.has_parameter('marker_timeout_sec'):
            self.declare_parameter('marker_timeout_sec', 15.0)
        # we by default use simulation time (use_sim_time = True)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        # we use multiple -p in command line command for setting
        # multiple parameters:
        # ros2 run object_fetcher --ros-args -p marker_timeout_sec:=10 -p scheduling_strategy:=priority

        # finally, we check what actual value of the scheduling strategy or marker_timeout will be used for the run
        # (default one or user input) by checking it's actual value...
        self.scheduling_strategy = self.get_parameter('scheduling_strategy').value
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value

        # Sub-nodes (shared process - added to the same executor in main())
        self.scheduler = TaskScheduler() # self.scheduler is a dictionary that is returned by TaskScheduler()
        self.scheduler.strategy = self.scheduling_strategy
        self.navigator = WaypointNavigator()
        self.pickup_site = PickupSiteNode()

        # Inject the random spawn positions into the scheduler's waypoints
        self._apply_random_zone_positions()

        # State machine variables
        self.state = MissionState.IDLE # the robot starts in the IDLE 
        # state until the trigger service signals it

        self.current_task = None
        self.detected_markers = [] # <- list of detected markers...

        # kind of state variables
        # variables robot_x and robot_y update only when the robot finishes
        # any navigation goal...(in _do_nav_pickup or _do_nav_dropoff ) While the 30-sec 
        # navigation is hapenning Nav2 takes care of the localization etc. 
        # and we only need these coordinates for planning (distance based, even if off by 20cm not a big deal)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Whhen robo reaches DETECTING_MARKER state, the _detect_start_time is populated
        # with the time.time(), and then for checking of failed perception, we check if 
        # current_time-self._detect_start_time against marker_timeout(15s), if marker isn't seen in this
        # time window, we proceed the mission regardless...
        self._detect_start_time = None

        # status flag to prevent overlapping navigation commands...
        # set to true just before calling self.navigator.navigate_to(...)
        # and set back to False after navigation completes.
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

        # start mission Trigger type service
        self.create_service(
            Trigger,
            '/start_mission',
            self._start_cb
        )

        # stop mission Trigger type service
        self.create_service(
            Trigger, 
            '/stop_mission', 
            self._stop_cb
        )
        ###################################
        #################################
        # self.create_subscription(
        #       Datatype,
        #       topic,
        #       callback function       (here self._marker_cb)
        #       queue size
        # )
        self.create_subscription(
            Int32MultiArray,
            '/aruco/marker_ids',
            self._marker_cb,
            10
        )
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
            f'MainController ready (strategy={self.scheduling_strategy})'   # this makes us aware by 
            # the terminal what the current strategy is...
        )

    # ── Inject random spawn positions into scheduler ──

    def _apply_random_zone_positions(self):

        positions = self.pickup_site.get_zone_positions() # get_zone_positions function in pickup_site_node.py is used
        
        if not positions: # if for some reason, self.get_zone_positions() returns an empty dict 
            # which will happen only when _zone_positions dict inside pickup_site_node.py remains empty, 
            # then in that case, we simply don't apply any random zones
            return
        
        for zone in self.scheduler.waypoints['pickup_zones']:
            zoneID = zone['id']     # zone corresponding to 'id'
            if zoneID in positions:
                zone['position']['x'] = positions[zoneID]['x']  # the x, y and name of 
                # zone in self.scheduler.waypoints['pickup_zones'] in 
                # PickupSiteNode's returned positions dictionary is updated with real world coordinates... 
                # other fields of zone in self.scheduler.waypoints are left unchanges... (like arucoID, is_visited, etc. are left unchanged)...
                zone['position']['y'] = positions[zoneID]['y']
                zone['name'] = positions[zoneID]['name']
                self.get_logger().info(
                    f'{zoneID} -> ({positions[zoneID]["x"]}, {positions[zoneID]["y"]})'
                )

    # ── Service callbacks ──

    def _start_cb(self, request, response):
        if self.state not in (MissionState.IDLE, MissionState.COMPLETED,
                              MissionState.ERROR):
            # if the state is neither of IDLE, COMPLETED or ERROR then _start_cb shouldn't be called back
            # so the response.message is flashed on the terminal to show that....
            response.success = False
            response.message = f'Already running ({self.state})'
            return response
        
        # self.scheduler is reset on the start...
        self.scheduler.reset()
        # we initialize the _pickup_retries and _dropoff_retries to 0 so that we accurately reflect that 
        # there has been zero retries for dropoff and pickup till now at the start...
        self._pickup_retries = 0
        self._dropoff_retries = 0


        # at the start, the contoller transitions from state: IDLE -> PLANNING..., at the end of this transition, 
        # response success asserted for this Trigger type service and message is also returned...
        self._transition_to(MissionState.PLANNING)
        response.success = True
        response.message = 'Mission started'
        return response

    def _stop_cb(self, request, response):
        # we cancel all navigation and transition current state to IDLE state 
        # in the case of Stop type trigger being asserted
        self.navigator.cancel_navigation()
        self._transition_to(MissionState.IDLE)
        response.success = True
        response.message = 'Mission stopped'
        return response

    def _marker_cb(self, msg):
    # callback needs msg as argument
        # _marker_cb is a callback of a subscription, so the main motive of the callback 
        # function is to strip off the requisite data from the message of the topic or service... 
        self.detected_markers = msg.data

    # ── Main loop (2 Hz) ──

    def _mission_loop(self):
        # every 0.5 sec, this function is called
        # from line:
        # self.create_timer(
        #    0.5,
        #    self._mission_loop
        # )
        # and it does the following, every 0.5 sec:
        # It publishes it's current state
        # as a String
        state_msg = String()
        state_msg.data = self.state
        self._state_pub.publish(state_msg) # it is a publisher on the topic /mission_state 
        # and this node publishes the current state every 0.5 sec...


        # It calls the function corresp.
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
        h = handlers.get(self.state) # we retrive the handler function corresponding to the
        # current state of the robo and if the state is not IDLE/ERROR or COMPLETED, we call 
        # the handler corresponding to the current state...
        if h:
            h()

    # ── State handlers ──

    def _do_planning(self):
        task = self.scheduler.get_next_task(self.robot_x, self.robot_y) # this function based on the current value of the robot_x and robot_y
        # (which got updated only when the navigation to last waypoint was successful)

        # if the get_next_task returns nothing, then we get to know that ALL ZONES have been completed... and we transition to COMPLETED state...
        if task is None:
            self.get_logger().info('ALL ZONES COMPLETED - mission successful!')
            self._transition_to(MissionState.COMPLETED)
            return
        
        # if there was a task returned by get_next_task, the code updates the current_task variable to the next task (only to be updated when 
        # next time in PLANNING state...)
        self.current_task = task
        #set_active_node form pickup_site_node is updated in this state...
        self.pickup_site.set_active_zone(task['id'])
        self.get_logger().info(f'Next target: {task.get("name", task["id"])}')

        # after the planning and the code knowing that atleast one more zone is left, state transition: PLANNING -> NAVIGATING_TO_PICKUP
        # happens...
        self._transition_to(MissionState.NAVIGATING_TO_PICKUP)

    def _do_nav_pickup(self):
        if self._nav_in_progress or not self.current_task: # navigation won't start again if it's already in progress
            # or if for some reason the robo forgot what it's destination zone was...,(IN THAT CASE PLANNING state would help) 
            return

        # position to go to is the current_task's position
        pos = self.current_task['position']
        # angle of appreoach if _pickup_retries == 0 == _ANGLES[0]
        # angle of appreoach if _pickup_retries == 1 == _ANGLES[1]
        # ... and so on...
        angle = _ANGLES[self._pickup_retries % len(_ANGLES)]

        # the nav_x and nav_y is a point 0.4 meters away from approach_zone in an angle (angle)
        nav_x, nav_y = self._approach_point(pos['x'], pos['y'], 0.4, angle)
        
        # the facing variable defines the angle(relative to the origin) the robo should be facing when it reaches the 
        # pickup zone...
        facing = math.atan2(pos['y'] - nav_y, pos['x'] - nav_x)

        self._nav_in_progress = True
        ok = self.navigator.navigate_to(nav_x, nav_y, facing) # navigates to nav_x and nav_y ******(Nav2 used here...)*******
        # with angle = facing... and then again sets _nav_in_progress = False...
        self._nav_in_progress = False

        if ok:
            # if the navigation "self.navigator.navigate_to(nav_x, nav_y, facing)" was successful,
            # then we re-update robot_x and robot_y to this nav_x and nav_y because we actually are sure that 
            # robot is very near to nav_x and nad_y (we trust Nav2)
            self.robot_x, self.robot_y = nav_x, nav_y
            self._pickup_retries = 0
            self._transition_to(MissionState.DETECTING_MARKER)
        else:
            # if naviagation was unsuccessful, we retry with another _ANGLRE
            self._pickup_retries += 1
            if self._pickup_retries < len(_ANGLES):
                self.get_logger().warn(
                    f'Pickup nav failed - retry {self._pickup_retries}'
                )
                self._transition_to(MissionState.NAVIGATING_TO_PICKUP)
                # if navigation failed, then agan call NAVIGATING_TO_PICKUP state again... keep on tring till 3 extra attempts...
            else:
                self.get_logger().error('All pickup approaches failed - skipping')
                self._pickup_retries = 0
                self.scheduler.mark_completed(self.current_task['id'])
                self._transition_to(MissionState.PLANNING)

    def _do_detect(self):
        expected = self.current_task.get('marker_id') # the expected marker is loaded into a variable...

        if self._detect_start_time is None: # let's say that we just began detecting, 
            # at that instant, we update detection start time instant by time.time() and tell the user that 
            # detection has began -> by Looking for marker log...
            self._detect_start_time = time.time()
            self.get_logger().info(f'Looking for marker {expected}...')

        if expected in self.detected_markers: # let's say that that the expected marker is actually
            # found in the list of detected markers, in this case we know that detection has been successful....
            self.get_logger().info(f'Marker {expected} detected!')
            self._detect_start_time = None
            self._transition_to(MissionState.CONFIRMING_PICKUP)
            return

        if time.time() - self._detect_start_time >= self.marker_timeout:
            # let's say that the control flow was not able to reach the second if statement(if expected in self.detected_markers:)
            # even once in 15 seconds then we conclude that the robo's angle of approach was wrong, instead of backing off and reapproaching 
            # the pickup zone, we proceed to Dropoff anyway... assuming that pickup zone was reached anyway...
            # update the state to CONFIRMING_PICKUP state...
            self.get_logger().warn('Marker timeout - proceeding anyway')
            self._detect_start_time = None
            self._transition_to(MissionState.CONFIRMING_PICKUP)

    def _do_confirm(self):
        # confirmation of pickup is just us Logging the terminal to tell the user that pickup was confirmed...
        name = self.current_task.get('name', self.current_task['id'])
        self.get_logger().info(f'Pickup confirmed at {name}')
        self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)

    def _do_nav_dropoff(self):
        if self._nav_in_progress: # if navigation already in process don't give navigation command again..., similar logic to _do_nav_pickup()
            return
        dropoff = self.scheduler.get_dropoff_zone()['position']
        angle = _ANGLES[self._dropoff_retries % len(_ANGLES)]

        nav_x, nav_y = self._approach_point(
            dropoff['x'],
            dropoff['y'], 
            0.7, 
            angle
        )

        heading = math.atan2(nav_y - self.robot_y, nav_x - self.robot_x)

        self._nav_in_progress = True
        ok = self.navigator.navigate_to(nav_x, nav_y, heading) 
        self._nav_in_progress = False

        if ok:
            self.robot_x, self.robot_y = nav_x, nav_y
            self._dropoff_retries = 0
            self._transition_to(MissionState.DELIVERING)
            # on success NAV_TO_DROPOFF -> DELIVERING
        else:
            self._dropoff_retries += 1
            if self._dropoff_retries < len(_ANGLES):
                self.get_logger().warn(
                    f'Dropoff nav failed - retry {self._dropoff_retries}')
                self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)
                # on approach angle being stupid, NAV_TO_DROPOFF -> NAV_TO_DROPOFF
            else:
                self.get_logger().error('All dropoff approaches failed - skipping')
                self._dropoff_retries = 0
                self.scheduler.mark_completed(self.current_task['id'])
                self.current_task = None
                self._transition_to(MissionState.PLANNING)
                # on all approaches failing, NAV_TO_DROPOFF -> PLANNING

    def _do_deliver(self):
        if self._nav_in_progress: # self._nav_in_progress acts as a flag to check if robo is busy delivering...
            # it is not the same as navigation in progress same flag has been reused..., but it will not trigger any unwanted function it only 
            # blocks the handler function.... 
            return
        self._nav_in_progress = True
        # name of zone delivered is stored in name variable...
        name = self.current_task.get('name', self.current_task['id'])

        # the recently completed node is marked completed...
        done = len(self.scheduler.completed_zones) + 1
        self.get_logger().info(f'Delivered from {name} ({done} done)')
        self.scheduler.mark_completed(self.current_task['id'])
        self.current_task = None
        time.sleep(1.0)
        self._transition_to(MissionState.PLANNING)
        # deliver -> PLANNING if everything goes smoothly
        self._nav_in_progress = False

    # ── Helpers ──

    def _approach_point(self, gx, gy, dist, angle_offset=0.0):
        """Point `dist` metres from goal, rotated by `angle_offset` around it."""
        dx, dy = gx - self.robot_x, gy - self.robot_y # at the start of the navigation (self.robot_x, self.robot_y) 
        # is the coordinate of the robot i.e. it is the coordinate of the previous completed navigation goal.
        # (gx, gy) is the coordinate of the goal point...
        d = math.hypot(dx, dy)
        if d <= dist:
            return max(-4.3, min(4.3, gx)), max(-4.3, min(4.3, gy)) # if the robot's current coordinate is itself nearer
            # to goal than dist it's better to go to goal only and not some other point...
        bx, by = -dx / d, -dy / d                       
        c, s = math.cos(angle_offset), math.sin(angle_offset)
        # bx, by is rotated in angle c and s and correct destination location is found by vector rotation plus translation...
        px = gx + (bx * c - by * s) * dist
        py = gy + (bx * s + by * c) * dist

        return max(-4.3, min(4.3, px)), max(-4.3, min(4.3, py))

    def _transition_to(self, new_state):
        old = self.state
        self.state = new_state
        self.get_logger().info(f'{old} -> {new_state}')
        # overlay on Rviz2 the state change..., and change state as well...
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
        label = labels.get(self.state, self.state) # gets the state's label for overlaying in rviz...
        zone = f"  [{self.current_task.get('name', '')}]" if self.current_task else ''

        # State-based colour
        color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # IDLE state -> white
        if self.state == MissionState.COMPLETED:
            color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0) # green
        elif self.state == MissionState.ERROR:
            color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0) # red
        elif 'NAVIGATING' in self.state:
            color = ColorRGBA(r=0.3, g=0.6, b=1.0, a=1.0) # blue
        elif self.state in (MissionState.DETECTING_MARKER,
                            MissionState.CONFIRMING_PICKUP):
            color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0) # yellow

        m = Marker()
        m.header.frame_id = 'base_link'                     # whenever the header is there, it's spawned on robo's base link frame
        m.header.stamp = self.get_clock().now().to_msg()    # timestamp of overlay
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

    # making the multithreaded executor node (4 threads)
    executor = MultiThreadedExecutor(num_threads=4)
    # making the parent node(ctrl)
    ctrl = MainController()
    for node in [ctrl, ctrl.navigator, ctrl.scheduler, ctrl.pickup_site]:
        # order of adding -> parent then children...
        executor.add_node(node)

    # spin executor till keyboard interrupt
    try:
        # this will spin the executer parent node(with 4 sub-nodes) till it encounters a keyboard interrupt, when
        # it'll go into finally block and destroy all sub nodes and shut rclpy down
        executor.spin()
    except KeyboardInterrupt:
        pass


    # cleanup and shut-down after keyboard interrupt...
    finally:
        for n in [ctrl.navigator, ctrl.scheduler, ctrl.pickup_site, ctrl]:
            # order of destroying, children then parents (like pointer freeing)
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
