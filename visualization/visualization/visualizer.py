import cv2
import os
import open3d as o3d
import numpy as np
from sympy.vector import matrix_to_vector
from ultralytics.engine.results import Keypoints

from .config import Config
from cv_bridge import CvBridge
from typing import List, Dict, Tuple
from interfaces.msg import EstimatedPoses, EstimatedPose, Detection
from sensor_msgs.msg import Image


class Visualizer:
    """
    Provides visualization tools for keypoints, pose estimations, and feasibility assessment using OpenCV overlays.

    Displays annotated RGB and depth images to help interpret detection accuracy, pose alignment,
    and physical constraints for robotic grasping tasks.
    """
    def __init__(self, config: Config, keypoints: bool, estimations: bool, feasible: bool):
        """
        Initializes the Visualizer with camera parameters and display settings.
        """
        self.storage: Dict[str, np.ndarray] = {}
        self.cap_axis = np.float32([[0.01, 0, 0],
                                    [0, 0.01, 0],
                                    [0, 0, 0.01]]).reshape(-1, 3)
        self.intrinsics = config.intrinsics
        self.distortion_coefficients = config.distortion_coefficients
        self.clearance_region = config.clearance_region
        self.depth_threshold = config.depth_threshold
        self.keypoints = keypoints
        self.estimations = estimations
        self.feasible = feasible

    def show_images(self):
        """
        Displays all currently stored image frames in individual OpenCV windows.

        Each frame is labeled and automatically updated with a short delay.
        """
        for frame, image in self.storage.items():
            cv2.imshow(frame, image)
        cv2.waitKey(1)

    def draw(self, rgb_image: np.ndarray, depth_image: np.ndarray, estimations: EstimatedPoses) -> None:
        """
        Renders multiple visualization overlays onto the input RGB and depth images.

        Applies keypoint markers, 3D axes projections, and feasibility polygons depending on user configuration.
        Stores each rendered frame internally for later display.
        """
        keypoints = [np.array(detection.keypoints).reshape(3, 2) for detection in estimations.detections]
        bounding_boxes = [np.array(detection.bounding_box).reshape(2, 2) for detection in estimations.detections]
        poses = [np.array(estimations.cap_to_camera).reshape(4, 4) for estimations in estimations.poses]
        rotation_vectors = []
        translation_vectors = []
        for pose in poses:
            rotation_vector, translation_vector = self.matrix_to_vectors(pose)
            rotation_vectors.append(rotation_vector)
            translation_vectors.append(translation_vector)
        if self.keypoints:
            self.storage["Keypoint frame"] = self.draw_object_detection(rgb_image, keypoints, bounding_boxes)
        if self.estimations:
            self.storage["axes frame"] = self.draw_pose_estimations(rgb_image, keypoints, rotation_vectors,
                                                                    translation_vectors)
        if self.feasible:
            self.storage["Feasible frame"] = self.draw_feasible_poses(depth_image, rotation_vectors,
                                                                      translation_vectors)
        self.show_images()

    def draw_object_detection(self, image: np.ndarray, keypoints_list: List[np.ndarray],
                              bounding_boxes: List[np.ndarray]) -> np.ndarray:
        """
        Draws bounding boxes and keypoints on an RGB image for visualizing object detection results.
        """
        draw_image = image.copy()
        for box, keypoints in zip(bounding_boxes, keypoints_list):
            cv2.rectangle(draw_image, [int(box[0][0]), int(box[0][1])], [int(box[1][0]), int(box[1][1])], (255, 0, 0),
                          2)
            cv2.circle(draw_image, [int(keypoints[0][0]), int(keypoints[0][1])], 4, (0, 255, 255), -1)
            cv2.circle(draw_image, [int(keypoints[1][0]), int(keypoints[1][1])], 4, (255, 255, 255), -1)
            cv2.circle(draw_image, [int(keypoints[2][0]), int(keypoints[2][1])], 4, (255, 255, 255), -1)
        return draw_image

    def draw_pose_estimations(self, image: np.ndarray, keypoints_list: List[np.ndarray],
                              rotation_vectors: List[List[float]],
                              transformation_vectors: List[List[float]]) -> np.ndarray:
        """
        Overlays feasibility regions onto a depth image and colors them based on pickability.

        Projects a clearance region for each pose and colors it green if the object is pickable, red otherwise.
        Also highlights nearby obstacles that could interfere with grasping.
        """
        draw_image = image.copy()
        for keypoints, rotation_vector, transformation_vector in zip(keypoints_list, rotation_vectors,
                                                                     transformation_vectors):
            image_points, _ = cv2.projectPoints(self.cap_axis, rotation_vector, transformation_vector, self.intrinsics,
                                                self.distortion_coefficients)
            cap_center = tuple(keypoints[0].astype(int))
            draw_image = cv2.line(draw_image, cap_center, tuple(image_points[0].ravel().astype(int)), (0, 0, 255), 2)
            draw_image = cv2.line(draw_image, cap_center, tuple(image_points[1].ravel().astype(int)), (0, 255, 0), 2)
            draw_image = cv2.line(draw_image, cap_center, tuple(image_points[2].ravel().astype(int)), (255, 0, 0), 2)
        return draw_image

    def draw_feasible_poses(self, depth_image: np.ndarray, rotation_vectors: List[np.ndarray],
                            translation_vectors: List[np.ndarray]) -> np.ndarray:
        """
        Overlays feasibility regions onto a depth image and colors them based on pickability.

        Projects a clearance region for each pose and colors it green if the object is pickable, red otherwise.
        Also highlights nearby obstacles that could interfere with grasping.
        """
        draw_image = cv2.cvtColor(depth_image.copy().astype(np.uint8), cv2.COLOR_GRAY2BGR)
        for rotation_vector, translation_vector in zip(rotation_vectors, translation_vectors):
            cap_depth = translation_vector[2]
            clearance_region, _ = cv2.projectPoints(self.clearance_region, rotation_vector, translation_vector,
                                                    self.intrinsics, self.distortion_coefficients)
            depth_threshold = self.depth_threshold + cap_depth
            if cap_depth is None:
                return draw_image
            if cap_depth >= depth_threshold:
                return draw_image
            mask = np.zeros(depth_image.shape, dtype=np.uint8)
            polygon = np.round(clearance_region).astype(np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(mask, [polygon], 1)
            masked_depth = depth_image.copy()
            masked_depth[mask == 0] = 0
            valid_depths = masked_depth[masked_depth > 0]
            if valid_depths.size == 0:
                return draw_image
            min_depth = np.min(valid_depths) * 0.001
            feasible = min_depth > depth_threshold
            cv2.polylines(draw_image, [polygon], True, (0, 255, 0) if feasible else (0, 0, 255), 2)
            exceed_mask = (depth_image > 0) & (depth_image < depth_threshold * 1000)
            draw_image[exceed_mask & (cv2.cvtColor(
                cv2.merge([mask := cv2.fillPoly(np.zeros_like(depth_image, dtype=np.uint8), [polygon], 1)], 0),
                cv2.COLOR_GRAY2BGR)[:, :, 0] == 1)] = [0, 0, 255]
        return draw_image

    def matrix_to_vectors(self, matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Converts a 4x4 transformation matrix into separate rotation and translation vectors.

        Uses OpenCV’s Rodrigues method to extract the rotation vector from the rotation matrix.
        """
        rotation_matrix = matrix[:3, :3]
        translation_vector = matrix[:3, 3].reshape(3, 1)
        rotation_vector, _ = cv2.Rodrigues(rotation_matrix)
        return rotation_vector, translation_vector
