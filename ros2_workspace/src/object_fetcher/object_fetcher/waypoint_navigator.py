"""
waypoint_navigator.py
---------------------
WHAT IT DOES: Sends the robot to a specific (x, y) location using Nav2.

HOW IT WORKS:
  Nav2 is like an autopilot. We give it a destination (x, y coordinates),
  and it figures out the path, avoids obstacles, and drives there.
  We communicate with Nav2 using an "Action" — like a long-running task
  where we can check progress and get a result.

ANALOGY: Like calling an Uber — you give a destination, it drives there,
         and tells you when it arrives (or if it failed).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('mc_waypoint_navigator')

        # Use ReentrantCallbackGroup so callbacks can run concurrently
        self.callback_group = ReentrantCallbackGroup()

        # Create the Nav2 action client
        # NavigateToPose is the action type — it takes a goal pose and
        # navigates the robot there autonomously
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Publish navigation status so other nodes know what's happening
        self._status_pub = self.create_publisher(
            String, '/navigation_status', 10
        )

        # Track current goal handle (so we can cancel if needed)
        self._goal_handle = None
        self._nav_result = None

        self.get_logger().info('WaypointNavigator ready. Waiting for Nav2...')

    def navigate_to(self, x, y, theta=0.0):
        """
        Navigate to position (x, y) with orientation theta (in radians).

        This is a BLOCKING call — it waits until the robot arrives or fails.

        Args:
            x, y: Target coordinates in the map frame
            theta: Target heading (0 = facing right/east)

        Returns:
            True if navigation succeeded, False otherwise
        """
        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f}, θ={theta:.2f})')
        self._publish_status('NAVIGATING')

        # Wait for Nav2 to be ready (up to 10 seconds)
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            self._publish_status('FAILED')
            return False

        # Build the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(x, y, theta)

        # Send the goal to Nav2
        self.get_logger().info('Sending goal to Nav2...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        self._goal_handle = send_goal_future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED by Nav2!')
            self._publish_status('FAILED')
            return False

        self.get_logger().info('Goal accepted! Robot is navigating...')

        # Wait for the navigation to finish
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ Arrived at ({x:.2f}, {y:.2f})!')
            self._publish_status('SUCCESS')
            return True
        else:
            self.get_logger().warn(f'❌ Navigation failed. Status code: {status}')
            self._publish_status('FAILED')
            return False

    def cancel_navigation(self):
        """Cancel the current navigation goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling navigation...')
            self._goal_handle.cancel_goal_async()
            self._publish_status('CANCELLED')

    def _create_pose_stamped(self, x, y, theta):
        """
        Create a PoseStamped message from (x, y, theta).

        PoseStamped = position + orientation + which coordinate frame.
        We use the 'map' frame (the global map Nav2 uses for navigation).

        Theta (yaw angle) is converted to a quaternion — that's just how
        ROS represents 3D rotations. For 2D navigation, only z and w matter.
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # Orientation: convert yaw angle to quaternion
        # For 2D: q = (0, 0, sin(θ/2), cos(θ/2))
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)

        return pose

    def _feedback_callback(self, feedback_msg):
        """Called periodically while navigating — shows distance remaining."""
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        if dist > 0.1:  # Only log if more than 10cm remaining
            self.get_logger().info(
                f'  📍 Distance remaining: {dist:.2f}m',
                throttle_duration_sec=2.0  # Log at most every 2 seconds
            )

    def _publish_status(self, status):
        """Publish current navigation status to a topic."""
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
