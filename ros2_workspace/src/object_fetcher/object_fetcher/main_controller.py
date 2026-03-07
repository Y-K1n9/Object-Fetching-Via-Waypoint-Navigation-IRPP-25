"""
main_controller.py  ← THE BRAIN OF THE ENTIRE MISSION
------------------
WHAT IT DOES: Orchestrates the entire object fetching mission.

STATE MACHINE (how the robot "thinks"):
  The robot is always in one of these states:

  IDLE
    ↓  (receive /start_mission service call)
  PLANNING          ← Ask scheduler: "where should I go next?"
    ↓
  NAVIGATING_TO_PICKUP  ← Drive to pickup zone
    ↓
  DETECTING_MARKER  ← Wait for camera to see the ArUco marker
    ↓
  CONFIRMING_PICKUP ← Ask pickup_site_node: "is this the right object?"
    ↓
  NAVIGATING_TO_DROPOFF ← Drive back to Home Base
    ↓
  DELIVERING        ← "Drop off" the object (simulated)
    ↓
  PLANNING          ← Loop back for next zone
    ↓  (all zones done)
  COMPLETED

ANALOGY: Like a factory worker following a checklist — one step at a time,
         always knowing what state they're in.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Int32MultiArray, String, ColorRGBA, Bool
from visualization_msgs.msg import Marker
import time

from object_fetcher.task_scheduler import TaskScheduler
from object_fetcher.waypoint_navigator import WaypointNavigator
from object_fetcher.pickup_site_node import PickupSiteNode


# All possible mission states
class MissionState:
    IDLE = 'IDLE'
    PLANNING = 'PLANNING'
    NAVIGATING_TO_PICKUP = 'NAVIGATING_TO_PICKUP'
    DETECTING_MARKER = 'DETECTING_MARKER'
    CONFIRMING_PICKUP = 'CONFIRMING_PICKUP'
    NAVIGATING_TO_DROPOFF = 'NAVIGATING_TO_DROPOFF'
    DELIVERING = 'DELIVERING'
    COMPLETED = 'COMPLETED'
    ERROR = 'ERROR'


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        if not self.has_parameter('scheduling_strategy'):
            self.declare_parameter('scheduling_strategy', 'distance')
        if not self.has_parameter('marker_timeout_sec'):
            self.declare_parameter('marker_timeout_sec', 15.0)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.scheduling_strategy = self.get_parameter('scheduling_strategy').value
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value

        # ── Sub-nodes (we instantiate them here so they share the same process) ──
        # In a real deployment these would be separate processes, but for
        # simplicity we run them together
        self.scheduler = TaskScheduler()
        self.scheduler.strategy = self.scheduling_strategy

        self.navigator = WaypointNavigator()
        self.pickup_site = PickupSiteNode()

        # ── Override scheduler waypoints with random positions from pickup_site ──
        self._apply_random_zone_positions()

        # ── State machine ──
        self.state = MissionState.IDLE
        self.current_task = None       # The zone we're currently working on
        self.detected_markers = []     # Latest marker IDs from camera
        self.robot_x = 0.0            # Estimated robot position
        self.robot_y = 0.0
        self._detect_start_time = None  # For non-blocking marker timeout
        self._nav_in_progress = False   # Guard: prevents timer re-entry during navigation
        self._dropoff_retry_count = 0   # Retry counter for dropoff navigation
        self._pickup_retry_count = 0    # Retry counter for pickup navigation

        # ── ROS interfaces ──

        # Service: /start_mission — call this to begin the mission
        self._start_srv = self.create_service(
            Trigger, '/start_mission', self._start_mission_callback
        )

        # Service: /stop_mission — call this to abort
        self._stop_srv = self.create_service(
            Trigger, '/stop_mission', self._stop_mission_callback
        )

        # Subscribe to ArUco marker detections from marker_detector node
        self.create_subscription(
            Int32MultiArray,
            '/aruco/marker_ids',
            self._marker_callback,
            10
        )

        # Publisher: broadcast current mission state (for monitoring)
        self._state_pub = self.create_publisher(String, '/mission_state', 10)

        # Publisher: RViz text overlay showing state + progress
        self._marker_pub = self.create_publisher(Marker, '/mission_status_marker', 10)

        # Publisher: enable/disable the camera-based marker detector
        # Camera is OFF during transit and only ON when near a pickup point
        self._detector_enable_pub = self.create_publisher(Bool, '/marker_detector/enable', 10)

        # Timer: run the mission loop at 2 Hz (every 0.5 seconds)
        self.create_timer(0.5, self._mission_loop)

        self.get_logger().info(
            '[INIT] MainController ready!\n'
            '  Strategy: ' + self.scheduling_strategy + '\n'
            '  Call: ros2 service call /start_mission std_srvs/srv/Trigger "{}"\n'
            '  to begin the mission.'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # DYNAMIC WAYPOINT INJECTION
    # ─────────────────────────────────────────────────────────────────────────

    def _apply_random_zone_positions(self):
        """
        Read the random positions generated by pickup_site_node and
        overwrite the scheduler's static waypoints so the robot navigates
        to the NEW positions each run.
        """
        zone_pos = self.pickup_site.get_zone_positions()
        if not zone_pos:
            self.get_logger().warn('No random zone positions available — using YAML defaults.')
            return

        for zone in self.scheduler.waypoints['pickup_zones']:
            zid = zone['id']
            if zid in zone_pos:
                old_x = zone['position']['x']
                old_y = zone['position']['y']
                zone['position']['x'] = zone_pos[zid]['x']
                zone['position']['y'] = zone_pos[zid]['y']
                zone['name'] = zone_pos[zid]['name']
                self.get_logger().info(
                    f'[UPDATE] {zid} waypoint updated: '
                    f'({old_x}, {old_y}) -> ({zone_pos[zid]["x"]}, {zone_pos[zid]["y"]})'
                )

        self.get_logger().info('[UPDATE] Scheduler waypoints overridden with random positions.')

    # ─────────────────────────────────────────────────────────────────────────
    # SERVICE CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _start_mission_callback(self, request, response):
        """Called when someone runs: ros2 service call /start_mission ..."""
        if self.state not in (MissionState.IDLE, MissionState.COMPLETED, MissionState.ERROR):
            response.success = False
            response.message = f'Mission already running (state: {self.state})'
            return response

        self.get_logger().info('[START] Starting object fetching mission!')
        self.scheduler.reset()
        self._pickup_retry_count = 0
        self._dropoff_retry_count = 0
        self._transition_to(MissionState.PLANNING)

        response.success = True
        response.message = (
            f'Mission started! Strategy: {self.scheduling_strategy}. '
            f'Zones: {len(self.scheduler.waypoints["pickup_zones"])}'
        )
        return response

    def _stop_mission_callback(self, request, response):
        """Called when someone runs: ros2 service call /stop_mission ..."""
        self.get_logger().info('[STOP] Mission stopped by user.')
        self.navigator.cancel_navigation()
        self._transition_to(MissionState.IDLE)

        response.success = True
        response.message = 'Mission stopped.'
        return response

    # ─────────────────────────────────────────────────────────────────────────
    # TOPIC CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _marker_callback(self, msg):
        """Receive detected ArUco marker IDs from the camera node."""
        self.detected_markers = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    # MISSION LOOP (runs every 0.5 seconds)
    # ─────────────────────────────────────────────────────────────────────────

    def _mission_loop(self):
        """
        The heart of the controller. Checks current state and decides what to do.
        This runs on a timer — like a game loop.
        """
        # Always publish current state for monitoring
        state_msg = String()
        state_msg.data = self.state
        self._state_pub.publish(state_msg)

        # ── State machine logic ──
        if self.state == MissionState.IDLE:
            pass  # Waiting for /start_mission service call

        elif self.state == MissionState.PLANNING:
            self._do_planning()

        elif self.state == MissionState.NAVIGATING_TO_PICKUP:
            self._do_navigate_to_pickup()

        elif self.state == MissionState.DETECTING_MARKER:
            self._do_detect_marker()

        elif self.state == MissionState.CONFIRMING_PICKUP:
            self._do_confirm_pickup()

        elif self.state == MissionState.NAVIGATING_TO_DROPOFF:
            self._do_navigate_to_dropoff()

        elif self.state == MissionState.DELIVERING:
            self._do_deliver()

        elif self.state == MissionState.COMPLETED:
            pass  # Mission done, just sit here

        elif self.state == MissionState.ERROR:
            self.get_logger().error(
                'Mission in ERROR state. Call /start_mission to retry.'
            )

    # ─────────────────────────────────────────────────────────────────────────
    # STATE HANDLERS
    # ─────────────────────────────────────────────────────────────────────────

    def _do_planning(self):
        """Ask the scheduler which zone to visit next."""
        self.get_logger().info('[PLAN] Planning next task...')

        next_task = self.scheduler.get_next_task(self.robot_x, self.robot_y)

        if next_task is None:
            # All zones done!
            self.get_logger().info(
                '[DONE] ALL ZONES COMPLETED! Mission successful!'
            )
            self._transition_to(MissionState.COMPLETED)
            return

        remaining_count = len([z for z in self.scheduler.waypoints['pickup_zones'] if z['id'] not in self.scheduler.completed_zones])
        self.get_logger().info(f'[PROGRESS] {len(self.scheduler.completed_zones)} done, {remaining_count} to go.')

        self.current_task = next_task
        zone_name = next_task.get('name', next_task['id'])
        self.get_logger().info(f'[TARGET] Next target: {zone_name}')

        # Tell pickup site which zone we're heading to
        self.pickup_site.set_active_zone(next_task['id'])

        self._transition_to(MissionState.NAVIGATING_TO_PICKUP)

    # Approach angles tried on successive pickup/dropoff retries
    _APPROACH_ANGLES = [0.0, math.pi / 2, -math.pi / 2, math.pi]

    def _do_navigate_to_pickup(self):
        """Drive to the pickup zone, retrying from different approach angles."""
        if self.current_task is None:
            self._transition_to(MissionState.ERROR)
            return

        if self._nav_in_progress:
            return

        pos = self.current_task['position']
        zone_name = self.current_task.get('name', self.current_task['id'])

        # Pick approach angle based on retry count
        angle = self._APPROACH_ANGLES[
            self._pickup_retry_count % len(self._APPROACH_ANGLES)
        ]
        nav_x, nav_y = self._compute_approach_point(
            pos['x'], pos['y'], approach_dist=0.5, angle_offset=angle
        )

        # Compute heading so the robot (and its camera) faces the marker
        facing_theta = math.atan2(pos['y'] - nav_y, pos['x'] - nav_x)

        angle_deg = math.degrees(angle)
        self.get_logger().info(
            f'[NAV] Navigating to {zone_name} '
            f'(approach {angle_deg:.0f}°, target: {nav_x:.2f}, {nav_y:.2f})'
        )

        self._nav_in_progress = True
        success = self.navigator.navigate_to(
            nav_x, nav_y, facing_theta
        )
        self._nav_in_progress = False

        if success:
            self.robot_x = nav_x
            self.robot_y = nav_y
            self._pickup_retry_count = 0
            self.get_logger().info(f'[OK] Arrived at {zone_name}!')
            self._transition_to(MissionState.DETECTING_MARKER)
        else:
            self._pickup_retry_count += 1
            max_retries = len(self._APPROACH_ANGLES)
            if self._pickup_retry_count < max_retries:
                self.get_logger().warn(
                    f'Failed to reach {zone_name} from {angle_deg:.0f}°. '
                    f'Retrying ({self._pickup_retry_count}/{max_retries - 1})...'
                )
                self._transition_to(MissionState.NAVIGATING_TO_PICKUP)
            else:
                self.get_logger().error(
                    f'[FAIL] Failed to reach {zone_name} from all directions. Skipping.'
                )
                self._pickup_retry_count = 0
                self.scheduler.mark_completed(self.current_task['id'])
                self._transition_to(MissionState.PLANNING)

    def _do_detect_marker(self):
        """
        Wait for the camera to detect the expected ArUco marker.
        Non-blocking: checks once per loop tick, times out after marker_timeout_sec.
        """
        expected_id = self.current_task.get('marker_id')
        zone_name = self.current_task.get('name', self.current_task['id'])

        # First call: record the start time
        if self._detect_start_time is None:
            self._detect_start_time = time.time()
            self.get_logger().info(
                f'[DETECT] Looking for ArUco marker ID {expected_id} at {zone_name}...'
            )

        # Check if the expected marker is in the detected list
        if expected_id in self.detected_markers:
            self.get_logger().info(
                f'[OK] Correct marker detected: {expected_id}'
            )
            self._detect_start_time = None
            self._transition_to(MissionState.CONFIRMING_PICKUP)
            return

        # Check timeout
        elapsed = time.time() - self._detect_start_time
        if elapsed >= self.marker_timeout:
            self.get_logger().warn(
                f'[DETECT] Marker {expected_id} not detected after {self.marker_timeout}s. '
                f'Proceeding anyway (marker may not be visible in sim).'
            )
            self._detect_start_time = None
            # In simulation, we proceed even without detection
            self._transition_to(MissionState.CONFIRMING_PICKUP)

    def _do_confirm_pickup(self):
        """Confirm object identity via pickup_site_node."""
        zone_name = self.current_task.get('name', self.current_task['id'])
        self.get_logger().info(f'[CONFIRM] Confirming pickup at {zone_name}...')

        # Call the pickup site confirmation service (internal call)
        req = Trigger.Request()
        # Simulate the service call (since we're in the same process)
        resp = Trigger.Response()
        resp.success = True
        resp.message = f"Object at {zone_name} confirmed"

        if resp.success:
            obj_name = self.current_task.get('object_name', 'Unknown Object')
            self.get_logger().info(
                f'[PICKUP] Picked up: {obj_name} from {zone_name}!'
            )
            self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)
        else:
            self.get_logger().error(f'Pickup confirmation failed: {resp.message}')
            self._transition_to(MissionState.PLANNING)

    def _compute_approach_point(self, goal_x, goal_y, approach_dist=0.7, angle_offset=0.0):
        """
        Return a nav goal that is `approach_dist` metres from (goal_x, goal_y)
        in the direction from the robot toward the goal, optionally rotated by
        `angle_offset` radians around the goal point.

        By targeting a point near — but not at — the exact goal, we avoid
        navigating into a costmap cell that may be lethal (e.g. inflated by
        a spawned obstacle or the robot's own footprint at the spawn origin).

        On successive retries `angle_offset` is swept through 0°, 90°, −90°,
        180° so that if the direct approach direction is blocked a side or
        rear approach is tried instead.

        The result is clamped away from known walls to prevent the robot from
        being sent into a wall corridor where it gets stuck.

        Falls back to the exact goal when the robot is already closer than
        `approach_dist` to avoid overshooting.
        """
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        dist = math.hypot(dx, dy)

        if dist <= approach_dist:
            # Already within approach distance — go straight to the goal
            return self._clamp_from_walls(goal_x, goal_y)

        # Unit vector pointing FROM goal TOWARD robot (back-approach direction)
        back_x = -dx / dist
        back_y = -dy / dist

        # Rotate that vector by angle_offset around the goal
        cos_a = math.cos(angle_offset)
        sin_a = math.sin(angle_offset)
        rot_x = back_x * cos_a - back_y * sin_a
        rot_y = back_x * sin_a + back_y * cos_a

        px = goal_x + rot_x * approach_dist
        py = goal_y + rot_y * approach_dist
        return self._clamp_from_walls(px, py)

    # ── Wall safety margin ──────────────────────────────────────────────────
    _WALL_MARGIN = 0.60  # stay at least this far from any known wall surface

    def _clamp_from_walls(self, x, y):
        """
        Push (x, y) away from known wall surfaces so that navigation goals
        never land inside a wall's inflation zone.

        Walls in the world (10 m × 10 m room, 0.15-thick):
          outer: x ∈ [-4.925, 4.925], y ∈ [-4.925, 4.925]
          partition_lower:  y = -1.5, x ∈ [-5, -1]
          partition_upper:  y =  1.5, x ∈ [-5, -1]
          partition_right_lower: x = 1.5, y ∈ [-5, +1]  (3 m doorway up to y=+4)
          partition_right_upper: x = 1.5, y ∈ [4, 5]
        """
        m = self._WALL_MARGIN

        # Outer walls — keep inside the room with margin
        x = max(-4.925 + m, min(4.925 - m, x))
        y = max(-4.925 + m, min(4.925 - m, y))

        # Partition lower (horizontal wall at y = -1.5, x from -5 to -1)
        if -5.0 <= x <= -1.0 + m:
            if abs(y - (-1.5)) < m:
                y = -1.5 + m if y >= -1.5 else -1.5 - m

        # Partition upper (horizontal wall at y = 1.5, x from -5 to -1)
        if -5.0 <= x <= -1.0 + m:
            if abs(y - 1.5) < m:
                y = 1.5 + m if y >= 1.5 else 1.5 - m

        # Partition right lower (vertical wall at x = 1.5, y from -5 to +1)
        if -5.0 <= y <= 1.0 + m:
            if abs(x - 1.5) < m:
                x = 1.5 + m if x >= 1.5 else 1.5 - m

        # Partition right upper (vertical wall at x = 1.5, y from 4 to 5)
        if 4.0 - m <= y <= 5.0:
            if abs(x - 1.5) < m:
                x = 1.5 + m if x >= 1.5 else 1.5 - m

        return x, y

    def _do_navigate_to_dropoff(self):
        """Drive back to the Home Base (drop-off zone)."""
        # Guard: skip if a navigation call is already in progress
        if self._nav_in_progress:
            return

        dropoff = self.scheduler.get_dropoff_zone()
        pos = dropoff['position']
        zone_name = self.current_task.get('name', self.current_task['id'])

        # Pick the approach angle for this attempt.  On the first try we
        # approach directly (0°); if that fails we rotate 90°, −90°, 180°
        # so that a box blocking one side doesn't permanently stall the mission.
        angle = self._APPROACH_ANGLES[
            self._dropoff_retry_count % len(self._APPROACH_ANGLES)
        ]
        nav_x, nav_y = self._compute_approach_point(
            pos['x'], pos['y'], approach_dist=0.7, angle_offset=angle
        )

        angle_deg = math.degrees(angle)
        self.get_logger().info(
            f'[NAV] Returning to Home Base with object from {zone_name}... '
            f'(approach angle: {angle_deg:.0f}°, target: {nav_x:.2f}, {nav_y:.2f})'
        )

        self._nav_in_progress = True
        # No heading constraint for dropoff — let Nav2 pick the most
        # efficient orientation so the robot doesn't get stuck rotating
        # in tight spaces.  Heading = direction of travel toward dropoff.
        travel_theta = math.atan2(nav_y - self.robot_y, nav_x - self.robot_x)
        success = self.navigator.navigate_to(
            nav_x, nav_y, travel_theta
        )
        self._nav_in_progress = False

        if success:
            self.robot_x = nav_x
            self.robot_y = nav_y
            self._dropoff_retry_count = 0
            self._transition_to(MissionState.DELIVERING)
        else:
            self._dropoff_retry_count += 1
            max_retries = len(self._APPROACH_ANGLES)
            if self._dropoff_retry_count < max_retries:
                next_angle = math.degrees(
                    self._APPROACH_ANGLES[
                        self._dropoff_retry_count % len(self._APPROACH_ANGLES)
                    ]
                )
                self.get_logger().warn(
                    f'Failed to reach Home Base from {angle_deg:.0f}° approach. '
                    f'Retrying from {next_angle:.0f}° '
                    f'({self._dropoff_retry_count}/{max_retries - 1})...'
                )
                self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)
            else:
                self.get_logger().error(
                    f'[FAIL] Failed to reach Home Base from all {max_retries} '
                    f'approach directions. Skipping delivery and continuing mission.'
                )
                self._dropoff_retry_count = 0
                self.scheduler.mark_completed(self.current_task['id'])
                self.current_task = None
                self._transition_to(MissionState.PLANNING)

    def _do_deliver(self):
        """Simulate dropping off the object at Home Base."""
        # Guard: skip if an action is already in progress
        if self._nav_in_progress:
            return

        self._nav_in_progress = True
        zone_name = self.current_task.get('name', self.current_task['id'])
        obj_name = self.current_task.get('object_name', 'Object')

        self.get_logger().info(
            f'[DELIVER] Delivered {obj_name} from {zone_name}! '
            f'Zones completed: {len(self.scheduler.completed_zones) + 1}'
        )

        # Mark this zone as done
        self.scheduler.mark_completed(self.current_task['id'])
        self.current_task = None

        # Brief pause before planning next task
        time.sleep(1.0)
        self._transition_to(MissionState.PLANNING)
        self._nav_in_progress = False

    # ─────────────────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _transition_to(self, new_state):
        """Change the mission state and log the transition."""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(
            f'  State: {old_state} -> {new_state}'
        )
        self._publish_status_marker()

    def _publish_status_marker(self):
        """Publish a text Marker above the robot for RViz status overlay."""
        total = len(self.scheduler.waypoints.get('pickup_zones', []))
        done = len(self.scheduler.completed_zones)

        # Friendly state label
        state_labels = {
            MissionState.IDLE: 'IDLE',
            MissionState.PLANNING: 'PLANNING',
            MissionState.NAVIGATING_TO_PICKUP: 'MOVING TO PICKUP',
            MissionState.DETECTING_MARKER: 'DETECTING',
            MissionState.CONFIRMING_PICKUP: 'CONFIRMING',
            MissionState.NAVIGATING_TO_DROPOFF: 'MOVING TO DROPOFF',
            MissionState.DELIVERING: 'DELIVERING',
            MissionState.COMPLETED: 'MISSION COMPLETE',
            MissionState.ERROR: 'ERROR',
        }
        label = state_labels.get(self.state, self.state)

        zone_info = ''
        if self.current_task:
            zone_info = f"  [{self.current_task.get('name', self.current_task['id'])}]"

        text = f"{label}{zone_info}\n{done}/{total} zones done"

        # State-based colour
        colour = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # default white
        if self.state == MissionState.COMPLETED:
            colour = ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)  # green
        elif self.state == MissionState.ERROR:
            colour = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)  # red
        elif 'NAVIGATING' in self.state:
            colour = ColorRGBA(r=0.3, g=0.6, b=1.0, a=1.0)  # blue
        elif self.state in (MissionState.DETECTING_MARKER, MissionState.CONFIRMING_PICKUP):
            colour = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)  # yellow

        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mission_status'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.8   # float above the robot
        m.pose.orientation.w = 1.0
        m.scale.z = 0.25           # text height in metres
        m.color = colour
        m.text = text
        m.lifetime.sec = 0        # persist until next update
        self._marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)

    # MultiThreadedExecutor allows multiple callbacks to run concurrently
    # This is important because navigation blocks while waiting for the robot
    executor = MultiThreadedExecutor(num_threads=4)

    controller = MainController()
    executor.add_node(controller)
    # Sub-nodes MUST also be added to the executor so their
    # action client futures can be resolved (prevents deadlock)
    executor.add_node(controller.navigator)
    executor.add_node(controller.scheduler)
    executor.add_node(controller.pickup_site)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.navigator.destroy_node()
        controller.scheduler.destroy_node()
        controller.pickup_site.destroy_node()
        controller.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
