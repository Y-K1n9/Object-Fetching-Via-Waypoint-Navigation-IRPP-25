"""
marker_detector.py  (PERCEPTION NODE — your teammate's part, but we include it)
------------------
WHAT IT DOES: Uses the robot's camera to detect ArUco markers.

WHAT IS AN ArUco MARKER?
  Like a QR code but simpler — a black-and-white square pattern.
  Each marker has a unique ID number. We place marker ID=0 at Zone Alpha,
  ID=1 at Zone Beta, ID=2 at Zone Gamma.

HOW IT WORKS:
  1. Subscribe to /camera/image_raw (robot's camera feed)
  2. Convert ROS image → OpenCV image (cv_bridge does this)
  3. Run ArUco detection algorithm
  4. Publish detected marker IDs to /aruco/marker_ids

ANALOGY: Like a barcode scanner at a supermarket checkout.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

# cv_bridge converts between ROS Image messages and OpenCV images
try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except Exception as e:
    # Catch ALL exceptions (ImportError, AttributeError, etc) to be safe
    # This ensures we fall back to simulation mode even if NumPy ABI is broken
    print(f"⚠️  MarkerDetector: OpenCV/cv_bridge import failed: {e}")
    print("⚠️  MarkerDetector: Falling back to SIMULATION MODE (no vision)")
    CV_AVAILABLE = False


class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')

        # Parameters (can be overridden in marker_config.yaml)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.15)  # meters
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('publish_debug_image', True)

        self.marker_size = self.get_parameter('marker_size').value
        image_topic = self.get_parameter('camera_image_topic').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        if not CV_AVAILABLE:
            self.get_logger().error(
                'OpenCV or cv_bridge not available! '
                'Marker detection will be simulated.'
            )
            self._setup_simulation_mode()
            return

        # Set up cv_bridge and ArUco detector
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Set up ArUco dictionary
        # DICT_4X4_50 means: 4x4 grid markers, up to 50 unique IDs
        dict_name = self.get_parameter('aruco_dict').value
        aruco_dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

        # Handle OpenCV version differences (API changed in 4.7)
        try:
            # New API (OpenCV 4.7+)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params
            )
            self.use_new_api = True
            self.get_logger().info('Using OpenCV 4.7+ ArUco API')
        except AttributeError:
            # Legacy API (OpenCV < 4.7)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False
            self.get_logger().info('Using legacy OpenCV ArUco API')

        # Subscribe to camera topics
        self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_callback, 10
        )

        # Publishers
        # /aruco/marker_ids — list of detected marker IDs (main_controller reads this)
        self._ids_pub = self.create_publisher(
            Int32MultiArray, '/aruco/marker_ids', 10
        )
        # /aruco/marker_poses — 3D positions of markers (for distance estimation)
        self._poses_pub = self.create_publisher(
            PoseArray, '/aruco/marker_poses', 10
        )
        # /aruco/debug_image — camera feed with markers highlighted (for visualization)
        if self.publish_debug:
            self._debug_pub = self.create_publisher(
                Image, '/aruco/debug_image', 10
            )

        self.get_logger().info(
            f'MarkerDetector ready. Listening on {image_topic}'
        )

    def _setup_simulation_mode(self):
        """
        Fallback: simulate marker detection without real OpenCV.
        Publishes a timer-based simulated detection for testing.
        """
        self._ids_pub = self.create_publisher(
            Int32MultiArray, '/aruco/marker_ids', 10
        )
        self._sim_marker_id = -1  # No marker detected by default
        self.get_logger().warn(
            'Running in SIMULATION mode — no real camera detection!'
        )

    def set_expected_marker(self, marker_id):
        """
        Tell the detector which marker ID to look for.
        Called by main_controller before arriving at a zone.
        """
        self._sim_marker_id = marker_id
        self.get_logger().info(f'Now looking for marker ID: {marker_id}')

    def _camera_info_callback(self, msg):
        """Store camera calibration data for pose estimation."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration data received.')

    def _image_callback(self, msg):
        """Process incoming camera image and detect ArUco markers."""
        if not CV_AVAILABLE:
            return
        try:
            # Step 1: Convert ROS Image → OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(
                f'cv_bridge failed (NumPy ABI mismatch?): {e}. '
                'Switching to simulation mode.',
                throttle_duration_sec=5.0
            )
            self._setup_simulation_mode()
            return

        try:
            # Step 2: Convert to grayscale (ArUco detection works on grayscale)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Step 3: Detect ArUco markers
            if self.use_new_api:
                corners, ids, rejected = self.detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )

            # Step 4: Process and publish results
            if ids is not None and len(ids) > 0:
                detected_ids = ids.flatten().tolist()
                self.get_logger().info(
                    f'🎯 Detected ArUco markers: {detected_ids}',
                    throttle_duration_sec=1.0
                )

                # Publish the detected IDs
                ids_msg = Int32MultiArray()
                ids_msg.data = detected_ids
                self._ids_pub.publish(ids_msg)

                # Step 5: Estimate 3D pose (if camera calibrated)
                if self.camera_matrix is not None:
                    self._estimate_and_publish_poses(corners, ids, cv_image)

                # Step 6: Draw markers on debug image
                if self.publish_debug:
                    debug_img = cv2.aruco.drawDetectedMarkers(
                        cv_image.copy(), corners, ids
                    )
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
                    self._debug_pub.publish(debug_msg)
            else:
                # No markers found — publish empty list
                ids_msg = Int32MultiArray()
                ids_msg.data = []
                self._ids_pub.publish(ids_msg)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def _estimate_and_publish_poses(self, corners, ids, image):
        """Estimate 3D position of each detected marker."""
        try:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size,
                self.camera_matrix, self.dist_coeffs
            )

            pose_array = PoseArray()
            pose_array.header.frame_id = 'camera_link'
            pose_array.header.stamp = self.get_clock().now().to_msg()

            for i, tvec in enumerate(tvecs):
                pose = Pose()
                pose.position.x = float(tvec[0][0])
                pose.position.y = float(tvec[0][1])
                pose.position.z = float(tvec[0][2])
                pose_array.poses.append(pose)

                dist = float(np.linalg.norm(tvec))
                self.get_logger().info(
                    f'  Marker {ids[i][0]}: distance = {dist:.2f}m',
                    throttle_duration_sec=1.0
                )

            self._poses_pub.publish(pose_array)
        except Exception as e:
            self.get_logger().warn(f'Pose estimation failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
