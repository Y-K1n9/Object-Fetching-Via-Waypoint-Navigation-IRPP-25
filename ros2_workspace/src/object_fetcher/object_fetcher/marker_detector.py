"""ArUco marker detector — subscribes to camera, publishes detected IDs."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import numpy as np
from cv_bridge import CvBridge
import cv2


class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('camera_image_topic').value

        self.bridge = CvBridge()

        # ArUco setup — DICT_4X4_50: 4x4 grid, up to 50 unique IDs
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params)
            self._new_api = True
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self._new_api = False

        # Tune detection for Gazebo software-rendered images
        self.aruco_params.minOtsuStdDev = 3.0
        self.aruco_params.adaptiveThreshWinSizeMax = 30
        self.aruco_params.minMarkerPerimeterRate = 0.02
        self.aruco_params.polygonalApproxAccuracyRate = 0.08

        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self._ids_pub = self.create_publisher(
            Int32MultiArray, '/aruco/marker_ids', 10)
        self.get_logger().info(f'MarkerDetector ready on {image_topic}')

    def _image_cb(self, msg):
        """Convert camera image to grayscale, detect ArUco markers, publish IDs."""
        try:
            gray = cv2.cvtColor(
                self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().warn(
                f'Image conversion failed: {e}', throttle_duration_sec=5.0)
            return

        if self._new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

        out = Int32MultiArray()
        if ids is not None and len(ids) > 0:
            out.data = sorted(set(ids.flatten().tolist()))
            self.get_logger().info(
                f'Detected markers: {out.data}', throttle_duration_sec=1.0)
        self._ids_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MarkerDetector())
    rclpy.shutdown()
