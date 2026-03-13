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
        # name of the detection node is merker_detector...
        super().__init__('marker_detector')
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('camera_image_topic').value

        self.bridge = CvBridge()

        # ArUco setup — DICT_4X4_50: 4x4 grid, up to 50 unique IDs
        # self.aruco_dict is storing a DICT_4X4_50 name dictionary of predefined
        # standard tested markers that OpenCV provides... One of these is only used...
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            # we try to instantiate the detector model as:
            # cv2.aruco.ArucDetector(self.aruco_dict, self.aruco_params)
            # if it is possible we set a boolean flag self._new_api = True
            # This is done as we'll try to use the new api because it's actually a better 
            # thing to use this new api
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, 
                self.aruco_params
            )
            self._new_api = True
        except AttributeError:
            # if loading this new model on the dictionary aruco_dict, with the aruco_params 
            # while trying to load is not able to load or throws an exception while trying to load,
            # we set self._new_api = True telling that the new_api couldn't be used and we'd need 
            # to fall back to the old api only..., that we'll do with new aruco_params since it's 
            # parameters are a bit different...
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self._new_api = False

        # Tune detection for Gazebo software-rendered images
        # We change minimum standard deviation for Otsu Thresholding that's used for foreground-background
        # classification of the image seen, this will ensure that the minimum admissible standard deviation is better
        # (the lower, the better) than the threshold of 5
        # this was found by us here: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
        self.aruco_params.minOtsuStdDev = 3.0
        # we increase the maximum window size for finding contours to 30 from default of 23, so that the 
        # camera image detection logic gets a wider perspective(checks window sizes 3, 5, 7, ..., 29) as opposed to till 23 
        # only while deciding whether a bit is actually black or white if too small of a window is used, it might not be able 
        # to distinguish the background with the black or white (let's say whole window was black only)
        self.aruco_params.adaptiveThreshWinSizeMax = 30
        # this parameter tells the model to consider bounding boxes in the image seen of perimeter >= 0.02 only...
        self.aruco_params.minMarkerPerimeterRate = 0.02
        # minimum accuracy while doing polygonal approximation to determine whether a contour found is a square
        # this was increased from the default = 0.03 to 0.08...
        self.aruco_params.polygonalApproxAccuracyRate = 0.08
        # self.create_subscription(
        #       Datatype,
        #       topic, <- user specified or default = '/camera/image_raw' ...
        #       callback function       (here self._image_cb)
        #       queue size
        # )
        self.create_subscription(
            Image, 
            image_topic, 
            self._image_cb, 
            10
        )
        # self.create_publisher(Message Type, 
        #                       topic this will publish to, 
        #                       queue size) 
        # keeping messages before they are processed,
        # they are kept in a queue,
        # ten messages kept older ones lost
        # best value = 10
        self._ids_pub = self.create_publisher(
            Int32MultiArray,
            '/aruco/marker_ids', 
            10
        )
        
        self.get_logger().info(f'MarkerDetector ready on {image_topic}')

    def _image_cb(self, msg):
        """Convert camera image to grayscale, detect ArUco markers, publish IDs."""
        try:
            # try to convert the msg from bgr8 to grayscale for easy processing...
            # gray will get populated with a cv2 grayscale image if there was no 
            # exception while doing so....
            gray = cv2.cvtColor(
                # ros2 sends images as a specific message
                # type of mesage (sensor_msgs/Image). Thiss is basically
                # a long flat stream of bytes that includes meta-data 
                # like timestamps. However openCV can't read this it requires
                # a numpy array. The imgmsg_to_cv2 takes the ROS2 message msg and decodes
                # it into a standard BGR image that openCV can understand.
                self.bridge.imgmsg_to_cv2(msg, 'bgr8'),
                # converting to grayscale removes the color noise that arises
                # from uneven lighting...
                cv2.COLOR_BGR2GRAY
            )
        except Exception as e:
            # if while conversion an exception ocurred, we log console 
            # and return...
            self.get_logger().warn(
                f'Image conversion failed: {e}', throttle_duration_sec=5.0
            )
            return

        if self._new_api:
            # if the new_api flag is True, i.e. the new api could be initialized,
            # we find ids using following approach( new approach )
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            # if the new_api couldn't be initialized, we use the old style of api
            # we pass in grayscale image, self.aruco_dict and other parameters...
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        # out stores 32 bit Integers(we put the detected id's into the out container)
        # in a flexible MultiArray container.
        out = Int32MultiArray()
        if ids is not None and len(ids) > 0:
            # if there has been a detection, we log the console and tell
            # user that yes there has been a detection... and we then publish 
            # what markers were detected...
            out.data = sorted(set(ids.flatten().tolist()))
            self.get_logger().info(
                f'Detected markers: {list(out.data)}', throttle_duration_sec=1.0
                # without list(out.data) : array('i', [2])
                # with list(out.data) : [2]
            )
        self._ids_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    rclpy.spin(node)
    rclpy.shutdown()
