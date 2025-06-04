import time
from logging import Logger
from typing import List
import numpy as np
import cv2
import math
from scipy.spatial.transform import Rotation
from interfaces.msg import EstimatedPoses
from interfaces.srv import MoveRobot, MoveGripper
from rclpy.client import Client


class MotionExecutor:
    """
    Executes motion sequences for a robot system including gripper control and pose transformations.

    This class handles the full manipulation pipeline, from transforming estimated poses to robot commands,
    computing grip sequences, and controlling both the robot and the gripper.
    """

    def __init__(self, robot: Client, gripper: Client, camera_to_tool: np.ndarray, camera_offset: np.ndarray, logger: Logger):
        """
        Initializes the MotionExecutor with robot interfaces and transformation data.
        """
        self.robot = robot
        self.gripper = gripper
        self.camera_to_tool = camera_to_tool
        self.camera_offset = camera_offset
        self.logger = logger
        self.home_position = [-math.pi / 2, -math.pi / 2, -math.pi / 2, -math.pi / 2, math.pi / 2, 0]

    def execute_pick_sequence(self, sequence: List[np.ndarray]):
        """
        Executes a full pick sequence consisting of approach, grip, and pull poses.

        The gripper is opened, the robot is moved to each pose in the sequence, the object is gripped,
        and finally pulled upwards.
        """
        self.command_gripper('open')
        approach = self.get_command(sequence[0], 'moveJ_IK', 0.4, 0.5)
        self.command_robot(approach)
        grip = self.get_command(sequence[1], 'moveL', 0.05, 0.2)
        self.command_robot(grip)
        self.command_gripper('close')
        pull = self.get_command(sequence[2], 'moveL', 0.05, 0.5)
        self.command_robot(pull)

    def home(self) -> None:
        """
        Sends the robot to a predefined home joint configuration.

        This is useful for resetting the robot to a safe position before or after task execution.
        """
        home = self.get_command(np.array(self.home_position), 'moveJ', 0.6, 0.5)
        self.command_robot(home)

    def leave_cap(self) -> None:
        """
        Moves the robot to a predefined position away from the cap and opens the gripper.

        Used to discard or release the cap after successful pick-up.
        """
        leave_position = self.get_command(np.array([0.410, -0.462, 0.2, 1.774, -1.870, 0.700]), 'moveJ_IK', 0.6, 0.5)
        self.command_robot(leave_position)
        self.command_gripper('open')

    def align(self, pose: np.ndarray) -> None:
        """
        Aligns the robot TCP with a given pose using inverse kinematics motion.

        Typically used before initiating a grip sequence to ensure correct orientation.
        """
        command = self.get_command(pose, 'moveJ_IK', 0.6, 0.5)
        self.command_robot(command)

    def get_align_pose(self, initial_estimations: EstimatedPoses, current_tcp_pose: np.ndarray) -> np.ndarray:
        """
        Computes an averaged pose from a set of initial estimations and the current TCP pose.

        The result is used to guide the robot toward a more stable alignment position.
        """
        first_transformations = self.transform_poses_to_base(initial_estimations,
                                                             self.build_transformation_matrix(current_tcp_pose[3:],
                                                                                              current_tcp_pose[:3]))
        mean_pose = self.calculate_mean_pose(first_transformations, current_tcp_pose)
        return self.matrix_to_vector(mean_pose)

    def get_pick_sequence(self, estimated_poses: EstimatedPoses, tcp_pose: np.ndarray) -> List[np.ndarray]:
        """
        Generates a full pick sequence consisting of approach, grip, and pull poses based on estimated poses.

        Uses the current TCP pose to transform the estimates into the robot's base frame.
        """
        poses = self.transform_poses_to_base(estimated_poses,
                                             self.build_transformation_matrix(tcp_pose[3:], tcp_pose[:3]))
        pick_sequence = self.get_grip_sequence(poses[0])
        return pick_sequence

    def transform_poses_to_base(self, poses: EstimatedPoses, current_pose: np.ndarray) -> List[np.ndarray]:
        """
        Transforms all estimated poses from the camera frame to the robot base frame.

        The transformation chain includes camera-to-tool and current TCP pose.
        """
        return [current_pose @ self.camera_to_tool @ pose.cap_to_camera.reshape(4, 4) for pose in
                poses.poses]

    def matrix_to_vector(self, matrix: np.ndarray) -> np.ndarray:
        """
        Converts a 4x4 transformation matrix into a 6D pose vector.

        The resulting vector contains 3D translation and Rodrigues rotation.
        """
        translation_vector = matrix[:3, 3]
        rotation_vector, _ = cv2.Rodrigues(matrix[:3, :3])
        return np.concatenate((translation_vector, rotation_vector.flatten()), axis=0)

    def build_transformation_matrix(self, rotation_vector: np.ndarray, transformation_vector: np.ndarray) -> np.ndarray:
        """
        Builds a 4x4 homogeneous transformation matrix from rotation and translation vectors.

        Uses OpenCV’s Rodrigues transformation to convert rotation vector into a matrix.
        """
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = transformation_vector
        return transformation_matrix

    def calculate_mean_pose(self, poses: List[np.ndarray], tcp_pose: np.ndarray) -> np.ndarray:
        """
        Computes a mean transformation matrix from a list of transformation matrices.

        Averages the translation and rotation (only X and Y) across all poses, then applies a fixed rotation
        and camera offset.
        """
        calculate_poses = poses.copy()
        rotation_vectors = np.array(
            [Rotation.from_matrix(rotation[:3, :3]).as_rotvec() for rotation in calculate_poses])
        mean_rotation = Rotation.from_rotvec(rotation_vectors[0]).as_matrix()
        mean_rotation = mean_rotation @ Rotation.from_euler('y', math.pi, degrees=False).as_matrix()
        mean_translation = np.mean([translation[:3, 3] for translation in calculate_poses], axis=0)
        mean_transformation = np.eye(4)
        mean_transformation[:3, :3] = mean_rotation
        mean_transformation[:3, 3] = mean_translation
        mean_transformation = mean_transformation @ self.camera_offset
        return mean_transformation

    def get_grip_sequence(self, pose: np.ndarray) -> List[np.ndarray]:
        """
        Constructs a list of poses for approach, grip, and pull actions based on a single target pose.

        Each step in the sequence applies specific spatial and rotational offsets to guide the robot
        through a safe and effective picking motion.
        """
        target = pose.copy()
        rotation_offset = np.eye(4)
        rotation_offset[:3, :3] = Rotation.from_euler('yz', [180, 90], degrees=True).as_matrix()
        approach_offset = np.eye(4)
        approach_offset[1, 3] = 0.016
        approach_offset[2, 3] = 0.01
        approach_pose = target @ approach_offset @ rotation_offset
        approach_pose = self.matrix_to_vector(approach_pose)
        grip_offset = np.eye(4)
        grip_offset[1, 3] = 0.016
        grip_offset[2, 3] = 0.001
        grip_offset[:3, :3] = Rotation.from_euler('x', -5, degrees=True).as_matrix()
        grip_pose = target @ grip_offset @ rotation_offset
        grip_pose = self.matrix_to_vector(grip_pose)
        pull_offset = np.eye(4)
        pull_offset[1, 3] = 0.016
        pull_offset[2, 3] = 0.07
        pull_offset[:3, :3] = Rotation.from_euler('x', 5, degrees=True).as_matrix()
        pull_pose = target @ pull_offset @ rotation_offset
        pull_pose = self.matrix_to_vector(pull_pose)
        return [approach_pose, grip_pose, pull_pose]

    def get_command(self, pose: np.ndarray, move_command: str, speed: float, acceleration: float) -> MoveRobot.Request:
        """
        Packages a pose and motion parameters into a robot movement request.

        Used to encapsulate pose, movement type, speed, and acceleration for robot execution.
        """
        command = MoveRobot.Request()
        command.pose = pose
        command.type = move_command
        command.speed = speed
        command.acceleration = acceleration
        return command

    def command_robot(self, command: MoveRobot.Request) -> MoveRobot.Response:
        """
        Sends a robot command asynchronously and waits for the result.

        Blocks execution until the robot finishes the commanded movement.
        """
        future = self.robot.call_async(command)
        while not future.done():
            time.sleep(0.1)
        return future.result()

    def command_gripper(self, command: str, position: int = None) -> MoveGripper.Response:
        """
        Sends a gripper command asynchronously and waits for the result.

        Supports both simple open/close commands and position-specific commands.
        """
        gripper_command = MoveGripper.Request()
        gripper_command.type = command
        if position is not None:
            gripper_command.position = position
        future = self.gripper.call_async(gripper_command)
        while not future.done():
            time.sleep(0.1)
        return future.result()
