import rclpy
import os
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from interfaces.msg import EstimatedPoses
from .pose_stability_tracker import PoseStabilityTracker
from ament_index_python.packages import get_package_share_directory
from .config import PoseStabilityTrackerConfig
from interfaces.srv import TriggerPickSequence


class PoseStabilityTrackerNode(Node):
    """
    ROS 2 node that monitors pose estimations over time and triggers pick actions when stability is detected.

    Subscribes to object pose estimates and evaluates their consistency using a stability tracker.
    If objects are stationary for a sufficient duration, it invokes a service to trigger the robot's pick sequence.
    """

    def __init__(self):
        """
        Initializes the node with pose stability tracking logic and ROS interfaces.

        Loads stability thresholds from configuration, sets up a pose subscriber,
        and prepares a service client for triggering robot actions.
        """
        super().__init__('pose_stability_tracker_node')
        self.logger = self.get_logger()
        config_directory = get_package_share_directory('resources')
        resources_path = os.path.join(config_directory, 'config/config.yaml')
        self.declare_parameter('debug', False)
        self.declare_parameter('auto', True)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.callback_group = ReentrantCallbackGroup()
        self.config = PoseStabilityTrackerConfig.from_yaml(resources_path)
        self.pose_tracker = PoseStabilityTracker(max_age=self.config.max_pose_age,
                                                 time_threshold=self.config.time_threshold,
                                                 distance_threshold=self.config.distance_threshold, logger=self.logger)
        self.pose_subscription = self.create_subscription(EstimatedPoses, '/vision/poses', self.stability_callback, 10, callback_group=self.callback_group)
        self.trigger_service = self.create_client(TriggerPickSequence, '/robot/trigger', callback_group=self.callback_group)
        self.robot_running = False
        self.logger.info('========== Pose stability tracker node started ==========')

    def stability_callback(self, estimations: EstimatedPoses) -> None:
        """
        Callback function that receives pose estimates and updates the stability tracker.

        If poses have remained still for long enough, it triggers the robot to begin the pick sequence via a service call.
        """
        if not self.robot_running and self.get_parameter('auto').get_parameter_value().bool_value:
            timestamp = estimations.header.stamp.sec + estimations.header.stamp.nanosec * 1e-9
            if not estimations.poses:
                self.logger.info('No poses detected.')
                return
            self.pose_tracker.update(estimations, timestamp)
            if self.pose_tracker.has_been_still():
                self.robot_running = True
                self.logger.info('Pose stability detected.')
                future = self.trigger_service.call_async(TriggerPickSequence.Request())
                self.logger.info('Pick sequence triggered.')
                while not future.done():
                    time.sleep(0.1)
                self.logger.info('Pick sequence done.')
                self.robot_running = False
        else:
            if self.debug:
                self.logger.info('Robot running.')


def main(args=None) -> None:
    """
    Initializes and runs the PoseStabilityTrackerNode in a ROS 2 environment.

    Spins the node to process incoming pose data and evaluate motion stability,
    and shuts down gracefully upon exit.
    """
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = PoseStabilityTrackerNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
