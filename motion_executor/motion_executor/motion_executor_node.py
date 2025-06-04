import os
import cv2
import numpy as np
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import EstimatedPoses
from interfaces.srv import TriggerPickSequence, GetTcpPose, MoveRobot, MoveGripper
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from .config import Config
from .motion_executor import MotionExecutor


class MotionExecutorNode(Node):
    """
    ROS 2 node responsible for coordinating robot motions based on detected object poses.

    Interfaces with robot, gripper, and vision systems to align the tool, pick caps,
    verify successful removal, and return to home position as needed.
    """

    def __init__(self):
        """
        Initializes the MotionExecutorNode by loading configuration, setting up services, and establishing clients.

        Sets up the transformation chain between camera and tool, loads configuration parameters, and connects to
        ROS 2 services needed for robot and gripper control, as well as TCP pose retrieval.
        """
        super().__init__('motion_executor_node')
        config_directory = get_package_share_directory('resources')
        resources_path = os.path.join(config_directory, 'config/config.yaml')
        self.config = Config.from_yaml(resources_path)
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()
        tool_tcp = self.config.tool_tcp
        camera_tcp = self.config.camera_to_tool
        tool_to_end_effector = self.build_transformation_matrix(tool_tcp[3:], tool_tcp[:3])
        camera_to_end_effector = self.build_transformation_matrix(camera_tcp[3:], camera_tcp[:3])
        camera_to_tool = np.linalg.inv(tool_to_end_effector) @ camera_to_end_effector
        camera_offset = np.eye(4)
        camera_offset[0, 3] = -camera_tcp[0]
        camera_offset[1, 3] = -camera_tcp[1]
        camera_offset[2, 3] = -0.25
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.debug = True
        self.motion_executor_service = self.create_service(TriggerPickSequence, 'trigger',
                                                           self.motion_executor_callback,
                                                           callback_group=self.callback_group)
        self.get_pose_client = self.create_client(GetTcpPose, '/robot/get_tcp_pose', callback_group=self.callback_group)
        while not self.get_pose_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn('Waiting for service /get_tcp_pose to respond...')
        self.robot_client = self.create_client(MoveRobot, '/robot/move_robot', callback_group=self.callback_group)
        while not self.robot_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn('Waiting for service /move_robot to respond...')
        self.gripper_client = self.create_client(MoveGripper, '/robot/move_gripper', callback_group=self.callback_group)
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn('Waiting for service /move_gripper to respond...')
        self.motion_executor = MotionExecutor(self.robot_client, self.gripper_client, camera_to_tool, camera_offset,
                                              self.logger)
        self.get_logger().info('========== Motion Executor Node initialized ==========')

    def motion_executor_callback(self, request: TriggerPickSequence.Request,
                                 response: TriggerPickSequence.Response) -> TriggerPickSequence.Response:
        """
        Callback for executing the full pick sequence when the trigger service is called.

        Aligns the robot based on initial pose estimates, refines the target pose, and repeatedly attempts to pick
        caps until none remain. Includes logic for verifying cap removal and handling errors gracefully.
        """
        self.motion_executor.home()
        initial_estimation = self.get_poses('/vision/poses')
        initial_tcp_pose = self.get_tcp_pose()
        align_pose = self.motion_executor.get_align_pose(initial_estimation, initial_tcp_pose)
        self.motion_executor.align(align_pose)
        time.sleep(0.5)
        while True:
            refined_estimations = self.get_poses('/monitoring/feasible_poses')
            refined_tcp_pose = self.get_tcp_pose()
            number_of_caps_to_pick = len(refined_estimations.poses)
            if not refined_estimations.poses:
                self.motion_executor.home()
                break
            pick_sequence = self.motion_executor.get_pick_sequence(refined_estimations, refined_tcp_pose)
            cap_picked = False
            while not cap_picked:
                print('number of caps to pick: {}'.format(number_of_caps_to_pick))
                self.motion_executor.execute_pick_sequence(pick_sequence)
                self.motion_executor.align(align_pose)
                time.sleep(0.5)
                if self.verify_cap_removal(number_of_caps_to_pick,
                                           len(self.get_poses('/monitoring/feasible_poses').poses)):
                    number_of_caps_to_pick -= 1
                    cap_picked = True
                    print('Cap picked successfully, caps left: {}'.format(number_of_caps_to_pick))
                else:
                    continue
                self.motion_executor.leave_cap()
                self.motion_executor.align(align_pose)
        response = TriggerPickSequence.Response()
        return response

    def verify_cap_removal(self, pre_removal_count: int, post_removal_count: int) -> bool:
        """
        Verifies whether a cap has been successfully removed by comparing the number of detected caps before and after.

        Returns True if exactly one cap has been removed; otherwise, returns False.
        """
        if post_removal_count != pre_removal_count - 1:
            return False
        else:
            return True

    def get_tcp_pose(self) -> np.ndarray:
        """
        Requests and returns the current TCP pose of the robot.

        Waits synchronously for the /get_tcp_pose service to return the pose data.
        """
        future = self.get_pose_client.call_async(GetTcpPose.Request())
        while not future.done():
            time.sleep(0.1)
        return future.result().tcp_pose

    def get_poses(self, topic: str) -> EstimatedPoses:
        """
        Subscribes once to the given topic and retrieves the latest EstimatedPoses message.

        Spins a temporary ROS node until a single message is received and then shuts it down.
        """
        one_shot_node = rclpy.create_node('one_shot_node')
        future = Future()

        def one_shot_callback(estimations):
            subscription.destroy()
            one_shot_node.destroy_subscription(subscription)
            future.set_result(estimations)

        subscription = one_shot_node.create_subscription(EstimatedPoses, topic, one_shot_callback, 10)
        rclpy.spin_until_future_complete(one_shot_node, future)
        one_shot_node.destroy_node()
        return future.result()

    def build_transformation_matrix(self, rotation_vector: np.ndarray, transformation_vector: np.ndarray) -> np.ndarray:
        """
        Constructs a 4x4 transformation matrix from a rotation vector and a translation vector.

        Converts the rotation vector to a matrix using the Rodrigues formula and combines it with the translation.
        """
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = transformation_vector
        return transformation_matrix


def main(args=None) -> None:
    """
    Initializes and spins the MotionExecutorNode using a multithreaded executor.

    Serves as the entry point for running the node as a standalone ROS 2 application.
    """
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = MotionExecutorNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
