import os
import rclpy
import message_filters
import time
from interfaces.msg import EstimatedPoses
from .pose_estimator import PoseEstimator
from .keypoint_detector import KeypointDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from .config import Config
from ament_index_python.packages import get_package_share_directory


class PoseEstimationNode(Node):
    """
    ROS 2 node for detecting keypoints and estimating 6DoF object poses from synchronized RGB-D images.

    Subscribes to RGB and aligned depth image topics, performs inference using a keypoint detector
    and pose estimator, and publishes the resulting poses as a custom message type.
    """
    def __init__(self):
        """
        Initializes the pose estimation node with configuration, model loading, and ROS topic subscriptions.

        Sets up synchronized message filters for RGB and depth images, loads the model and configuration files,
        and initializes the publisher for estimated poses.
        """
        super().__init__('pose_estimation_node')
        share_directory = get_package_share_directory('pose_estimation')
        config_directory = get_package_share_directory('resources')
        resources_path = os.path.join(config_directory, 'config/config.yaml')
        self.config = Config.from_yaml(resources_path)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.keypoint_detector = KeypointDetector(config=self.config, share_directory=share_directory)
        self.estimator = PoseEstimator(config=self.config, share_directory=share_directory)
        self.subscribers = [
            message_filters.Subscriber(self, Image, '/camera/d435i/color/image_raw'),
            message_filters.Subscriber(self, Image, '/camera/d435i/aligned_depth_to_color/image_raw')]
        self.synchronizer = message_filters.TimeSynchronizer(self.subscribers, 10)
        self.synchronizer.registerCallback(self.pose_estimation_callback)
        self.estimation_publisher = self.create_publisher(EstimatedPoses, 'poses', 10)
        self.logger.info('========== Pose estimation node started ==========')

    def pose_estimation_callback(self, raw_rgb_image: Image, raw_depth_image: Image) -> None:
        """
        Callback for processing synchronized RGB and depth images to estimate object poses.

        Converts image messages to OpenCV format, runs keypoint detection and pose estimation,
        and publishes the results with the appropriate timestamp header.
        """
        if self.debug:
            start_time = time.time()
        rgb_image = self.bridge.imgmsg_to_cv2(raw_rgb_image, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(raw_depth_image, '16UC1')
        detections = self.keypoint_detector.detect(rgb_image)
        estimations = self.estimator.estimate(detections, rgb_image, depth_image)
        estimations.header = raw_rgb_image.header
        self.estimation_publisher.publish(estimations)
        if self.debug:
            elapsed_ms = (time.time() - start_time) * 1000
            print(f"Execution time: {elapsed_ms:.2f} ms")


def main(args=None) -> None:
    """
    Initializes and runs the PoseEstimationNode within a ROS 2 environment.

    Sets up the node, begins spinning to process synchronized image streams,
    and shuts down gracefully when complete.
    """
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
