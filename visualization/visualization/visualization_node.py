import rclpy
import cv2
import os
import message_filters
from sensor_msgs.msg import Image
from interfaces.msg import EstimatedPoses
from .visualizer import Visualizer
from .config import Config
from rclpy.node import Node
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class VisualizationNode(Node):
    """
    ROS 2 node that visualizes keypoints, pose estimations, and feasibility information from a vision pipeline.

    Subscribes to synchronized RGB, depth, and pose messages, and renders visual overlays to help interpret
    detection results and grasp feasibility in real time.
    """
    def __init__(self):
        """
        Initializes the visualization node with configuration parameters and subscriber filters.

        Sets up synchronization for RGB image, depth image, and pose estimation messages.
        Loads the visualization configuration and prepares the visualizer for rendering.
        """
        super().__init__('visualization_node')
        config_directory = get_package_share_directory('resources')
        resources_path = os.path.join(config_directory, 'config/config.yaml')
        self.config = Config.from_yaml(resources_path)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.declare_parameter('keypoints', True)
        self.declare_parameter('estimations', True)
        self.declare_parameter('feasible', True)
        keypoints = self.get_parameter('keypoints').get_parameter_value().bool_value
        estimations = self.get_parameter('estimations').get_parameter_value().bool_value
        feasible = self.get_parameter('feasible').get_parameter_value().bool_value
        self.visualizer = Visualizer(config=self.config, keypoints=keypoints, estimations=estimations, feasible=feasible)
        self.subscribers = [
            message_filters.Subscriber(self, Image, '/camera/d435i/color/image_raw'),
            message_filters.Subscriber(self, Image, '/camera/d435i/aligned_depth_to_color/image_raw'),
            message_filters.Subscriber(self, EstimatedPoses, '/vision/poses'), ]
        self.synchronizer = message_filters.TimeSynchronizer(self.subscribers, 100)
        self.synchronizer.registerCallback(self.visualizer_callback)
        self.logger.info('========== Visualization node started ==========')

    def visualizer_callback(self, raw_rgb_image: Image, raw_depth_image: Image, estimations: EstimatedPoses) -> None:
        """
        Callback function that processes synchronized image and pose messages.

        Converts ROS image messages to OpenCV format and passes them to the visualizer for rendering.
        Applies optional depth colormapping for visualization clarity.
        """
        rgb_image = self.bridge.imgmsg_to_cv2(raw_rgb_image, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(raw_depth_image, '16UC1')
        cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        self.visualizer.draw(rgb_image, depth_image, estimations)


def main(args=None) -> None:
    """
    Initializes and runs the VisualizationNode in a ROS 2 environment.

    Spins the node to process synchronized visual and pose data streams and shuts down gracefully on exit.
    """
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
