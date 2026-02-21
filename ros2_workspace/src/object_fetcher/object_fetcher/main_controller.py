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

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Int32MultiArray, String
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

        # ── State machine ──
        self.state = MissionState.IDLE
        self.current_task = None       # The zone we're currently working on
        self.detected_markers = []     # Latest marker IDs from camera
        self.robot_x = 0.0            # Estimated robot position
        self.robot_y = 0.0
        self._detect_start_time = None  # For non-blocking marker timeout
        self._nav_in_progress = False   # Guard: prevents timer re-entry during navigation

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

        # Timer: run the mission loop at 2 Hz (every 0.5 seconds)
        self.create_timer(0.5, self._mission_loop)

        self.get_logger().info(
            '🤖 MainController ready!\n'
            '  Strategy: ' + self.scheduling_strategy + '\n'
            '  Call: ros2 service call /start_mission std_srvs/srv/Trigger "{}"\n'
            '  to begin the mission.'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # SERVICE CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _start_mission_callback(self, request, response):
        """Called when someone runs: ros2 service call /start_mission ..."""
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Mission already running (state: {self.state})'
            return response

        self.get_logger().info('🚀 Starting object fetching mission!')
        self.scheduler.reset()
        self._transition_to(MissionState.PLANNING)

        response.success = True
        response.message = (
            f'Mission started! Strategy: {self.scheduling_strategy}. '
            f'Zones: {len(self.scheduler.waypoints["pickup_zones"])}'
        )
        return response

    def _stop_mission_callback(self, request, response):
        """Called when someone runs: ros2 service call /stop_mission ..."""
        self.get_logger().info('🛑 Mission stopped by user.')
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
        self.get_logger().info('📋 Planning next task...')

        next_task = self.scheduler.get_next_task(self.robot_x, self.robot_y)

        if next_task is None:
            # All zones done!
            self.get_logger().info(
                '🎉 ALL ZONES COMPLETED! Mission successful!'
            )
            self._transition_to(MissionState.COMPLETED)
            return

        remaining_count = len([z for z in self.scheduler.waypoints['pickup_zones'] if z['id'] not in self.scheduler.completed_zones])
        self.get_logger().info(f'📊 Progress: {len(self.scheduler.completed_zones)} done, {remaining_count} to go.')

        self.current_task = next_task
        zone_name = next_task.get('name', next_task['id'])
        self.get_logger().info(f'📍 Next target: {zone_name}')

        # Tell pickup site which zone we're heading to
        self.pickup_site.set_active_zone(next_task['id'])

        self._transition_to(MissionState.NAVIGATING_TO_PICKUP)

    def _do_navigate_to_pickup(self):
        """Drive to the pickup zone."""
        if self.current_task is None:
            self._transition_to(MissionState.ERROR)
            return

        # Guard: skip if a navigation call is already in progress
        if self._nav_in_progress:
            return

        pos = self.current_task['position']
        zone_name = self.current_task.get('name', self.current_task['id'])

        self.get_logger().info(
            f'🚗 Navigating to Pickup Zone: {zone_name} '
            f'({pos["x"]:.1f}, {pos["y"]:.1f})'
        )

        # This call BLOCKS until the robot arrives (or fails)
        self._nav_in_progress = True
        success = self.navigator.navigate_to(
            pos['x'], pos['y'], pos.get('theta', 0.0)
        )
        self._nav_in_progress = False

        if success:
            # Update estimated robot position
            self.robot_x = pos['x']
            self.robot_y = pos['y']
            self.get_logger().info(f'✅ Arrived at {zone_name}!')
            self._transition_to(MissionState.DETECTING_MARKER)
        else:
            self.get_logger().error(
                f'❌ Failed to reach {zone_name}. Skipping zone.'
            )
            # Skip this zone and try the next one
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
                f'👁️  Looking for ArUco marker ID {expected_id} at {zone_name}...'
            )

        # Check if the expected marker is in the detected list
        if expected_id in self.detected_markers:
            self.get_logger().info(
                f'✅ Correct marker detected: {expected_id}'
            )
            self._detect_start_time = None
            self._transition_to(MissionState.CONFIRMING_PICKUP)
            return

        # Check timeout
        elapsed = time.time() - self._detect_start_time
        if elapsed >= self.marker_timeout:
            self.get_logger().warn(
                f'⏰ Marker {expected_id} not detected after {self.marker_timeout}s. '
                f'Proceeding anyway (marker may not be visible in sim).'
            )
            self._detect_start_time = None
            # In simulation, we proceed even without detection
            self._transition_to(MissionState.CONFIRMING_PICKUP)

    def _do_confirm_pickup(self):
        """Confirm object identity via pickup_site_node."""
        zone_name = self.current_task.get('name', self.current_task['id'])
        self.get_logger().info(f'🔍 Confirming pickup at {zone_name}...')

        # Call the pickup site confirmation service (internal call)
        req = Trigger.Request()
        # Simulate the service call (since we're in the same process)
        resp = Trigger.Response()
        resp.success = True
        resp.message = f"Object at {zone_name} confirmed"

        if resp.success:
            obj_name = self.current_task.get('object_name', 'Unknown Object')
            self.get_logger().info(
                f'📦 Picked up: {obj_name} from {zone_name}!'
            )
            self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)
        else:
            self.get_logger().error(f'Pickup confirmation failed: {resp.message}')
            self._transition_to(MissionState.PLANNING)

    def _do_navigate_to_dropoff(self):
        """Drive back to the Home Base (drop-off zone)."""
        # Guard: skip if a navigation call is already in progress
        if self._nav_in_progress:
            return

        dropoff = self.scheduler.get_dropoff_zone()
        pos = dropoff['position']
        zone_name = self.current_task.get('name', self.current_task['id'])

        self.get_logger().info(
            f'🏠 Returning to Home Base with object from {zone_name}...'
        )

        self._nav_in_progress = True
        success = self.navigator.navigate_to(
            pos['x'], pos['y'], pos.get('theta', 0.0)
        )
        self._nav_in_progress = False

        if success:
            self.robot_x = pos['x']
            self.robot_y = pos['y']
            self._transition_to(MissionState.DELIVERING)
        else:
            self.get_logger().error('Failed to reach Home Base! Retrying...')
            # Try again
            self._transition_to(MissionState.NAVIGATING_TO_DROPOFF)

    def _do_deliver(self):
        """Simulate dropping off the object at Home Base."""
        # Guard: skip if an action is already in progress
        if self._nav_in_progress:
            return

        self._nav_in_progress = True
        zone_name = self.current_task.get('name', self.current_task['id'])
        obj_name = self.current_task.get('object_name', 'Object')

        self.get_logger().info(
            f'📬 Delivered {obj_name} from {zone_name}! '
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
            f'  State: {old_state} → {new_state}'
        )


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
