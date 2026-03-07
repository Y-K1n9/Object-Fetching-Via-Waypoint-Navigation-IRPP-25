"""Nav2 action-client wrapper — sends the robot to (x, y, theta) goals."""

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
        super().__init__('mc_waypoint_navigator')
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)
        self._goal_handle = None
        self.get_logger().info('WaypointNavigator ready.')

    def navigate_to(self, x, y, theta=0.0, timeout_sec=120.0):
        """Blocking: drive to (x, y, theta). Returns True on success."""
        self.get_logger().info(f'Nav goal ({x:.2f}, {y:.2f}, θ={theta:.2f})')

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 not available!')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, theta)

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)

        # Wait for goal acceptance
        deadline = time.monotonic() + 15.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error('Goal acceptance timeout')
                return False
            time.sleep(0.05)

        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return False

        # Wait for navigation result
        result_future = self._goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_sec
        while not result_future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn('Navigation timeout — cancelling')
                self._goal_handle.cancel_goal_async()
                return False
            time.sleep(0.1)

        ok = result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f'Arrived at ({x:.2f}, {y:.2f})' if ok else 'Navigation failed')
        return ok

    def cancel_navigation(self):
        """Cancel the current navigation goal."""
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()

    def _make_pose(self, x, y, theta):
        """Build a PoseStamped in the map frame from (x, y, yaw)."""
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(theta / 2.0)
        p.pose.orientation.w = math.cos(theta / 2.0)
        return p

    def _feedback_cb(self, msg):
        d = msg.feedback.distance_remaining
        if d > 0.1:
            self.get_logger().info(
                f'  Distance remaining: {d:.2f}m', throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointNavigator())
    rclpy.shutdown()
