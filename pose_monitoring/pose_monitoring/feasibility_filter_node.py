import rclpy
import os
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from interfaces.msg import EstimatedPoses
from .feasibility_filter import FeasibilityFilter
from ament_index_python.packages import get_package_share_directory
from .config import FeasibilityFilterConfig


class PoseFeasibilityFilter(Node):
    """
    ROS 2 node that filters estimated object poses based on feasibility constraints using depth data.

    Subscribes to synchronized pose and depth topics, evaluates whether each pose has sufficient clearance,
    and publishes only the feasible poses for downstream motion execution.
    """
    def __init__(self):
        """
        Initializes the feasibility filtering node with configuration parameters and synchronized ROS subscribers.

        Loads camera and depth filter parameters from configuration, sets up message filtering, and prepares
        a publisher for broadcasting filtered, pickable poses.
        """
        super().__init__('feasibility_filter_node')
        self.logger = self.get_logger()
        self.bridge = CvBridge()
        config_directory = get_package_share_directory('resources')
        resources_path = os.path.join(config_directory, 'config/config.yaml')
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.config = FeasibilityFilterConfig.from_yaml(resources_path)
        self.feasibility_filter = FeasibilityFilter(depth_threshold=self.config.depth_threshold,
                                                    clearance_region=self.config.clearance_region,
                                                    intrinsics=self.config.intrinsics,
                                                    distortion_coefficients=self.config.distortion_coefficients,
                                                    logger=self.logger, debug=self.debug)
        self.subscribers = [
            message_filters.Subscriber(self, EstimatedPoses, '/vision/poses'),
            message_filters.Subscriber(self, Image, '/camera/d435i/aligned_depth_to_color/image_raw')]
        self.synchronizer = message_filters.TimeSynchronizer(self.subscribers, 100)
        self.synchronizer.registerCallback(self.feasibility_callback)
        self.feasible_pose_publisher = self.create_publisher(EstimatedPoses, 'feasible_poses', 10)
        self.logger.info('========== Feasibility filter node started ==========')

    def feasibility_callback(self, estimations: EstimatedPoses, depth_image: Image) -> None:
        """
        Callback function for evaluating the feasibility of received poses using the current depth image.

        Applies geometric clearance checks to the input poses, and publishes only those that meet
        the depth and space requirements defined by the filter.
        """
        feasible_poses = self.feasibility_filter.get_feasible_poses(estimations,
                                                                    self.bridge.imgmsg_to_cv2(depth_image, '16UC1'))
        if feasible_poses.poses:
            if self.debug:
                self.logger.info(f'Found {len(feasible_poses.poses)} feasible poses. Sending to motion executor.')
        elif self.debug:
            self.logger.info('No feasible poses found.')
        self.feasible_pose_publisher.publish(feasible_poses)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFeasibilityFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
