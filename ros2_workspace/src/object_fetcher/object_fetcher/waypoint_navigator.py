"""Nav2 action-client wrapper - sends the robot to (x, y, theta) goals."""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('mc_waypoint_navigator') # main controller's waypoint navigator
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient( # uses Actions -> suited for long-running tasks
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self._cb_group
        )
        self._goal_handle = None
        self.get_logger().info('WaypointNavigator ready.')

    def navigate_to(self, x, y, theta=0.0, timeout_sec=120.0):
        """Blocking: drive to (x, y, theta). Returns True on success."""
        self.get_logger().info(f'Nav goal ({x:.2f}, {y:.2f}, θ={theta:.2f})')

        # for ensuring that Nav2 is available we wait 10 sec for it
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 not available!')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, theta)

        # we send the work order to NAv2 server.. 
        # async means that function immediately gices confirmation if goal was sent...
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )

        # Wait for goal acceptance
        deadline = time.monotonic() + 15.0
        while not future.done(): # if future ka not done hona 15 sec le le, toh, 
            # we log an error and return....
            if time.monotonic() > deadline:
                self.get_logger().error('Goal acceptance timeout')
                return False
            time.sleep(0.05)

        self._goal_handle = future.result()
        # if the goal is not accepted by nav2, log the user, and return False..(we couldn't pass goal to nav2)
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return False

        # Wait for navigation result
        result_future = self._goal_handle.get_result_async()
        # when the goal has been passed to Nav2, it gives us a _goal_handle
        # This line populates future_result the moment robo either completes goal/
        # crashes or gives up...


        # deadline 2 mins for any navigation task put since we assume if
        # > 2mins taken, it surely is an error... 
        deadline = time.monotonic() + timeout_sec
        while not result_future.done(): # while the Nav2 doesn't return any 
            # result message(done/error/give_up), if it took 2 mins, we
            # log user, cancel goal and return False
            if time.monotonic() > deadline:
                self.get_logger().warn('Navigation timeout - cancelling')
                self._goal_handle.cancel_goal_async()
                return False
            # we make sure cpu rests for 100ms between any successive checks...
            time.sleep(0.1)

        # final confirmation flag is following::
        ok = result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f'Arrived at ({x:.2f}, {y:.2f})' if ok else 'Navigation failed')
        return ok

    def cancel_navigation(self):
        """Cancel the current navigation goal."""
        # using _goal_handle's cancel_goal_async function...
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()

    def _make_pose(self, x, y, theta):
        """Build a PoseStamped in the map frame from (x, y, yaw)."""
        # PoseStamped is a type of msg from geometry_msgs.msg
        # it stores Pose of robo with reference coorrdinte frame and timestamp...
        # Header header <- ref frame and timestamp
        # Pose pose <- pose quaternion
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(theta / 2.0)
        p.pose.orientation.w = math.cos(theta / 2.0)
        return p

    def _feedback_cb(self, msg): # this is the feedback
        # callback function of the navigation action client
        d = msg.feedback.distance_remaining
        if d > 0.1:
            self.get_logger().info(
                f'  Distance remaining: {d:.2f}m', throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    rclpy.shutdown()
