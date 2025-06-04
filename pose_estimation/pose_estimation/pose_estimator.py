import cv2
import numpy as np
import open3d as o3d
from pathlib import Path
from interfaces.msg import EstimatedPose, EstimatedPoses, Detection
from .config import Config
from typing import List


class PoseEstimator:
    """
    Estimates 6DoF poses of detected objects using PnP and ICP refinement.

    Combines keypoint-based pose estimation with point cloud alignment using a CAD model
    to improve accuracy and robustness in real-world scenes.
    """
    def __init__(self, config: Config, share_directory: str):
        """
        Initializes the pose estimator with camera intrinsics, distortion coefficients, PnP method, and CAD model.

        Loads and samples the CAD model to prepare for point cloud registration in later refinement steps.
        """
        self.cap_object_points = config.cap_object_points
        self.pnp_method = config.pnp_method
        self.intrinsics = config.intrinsics
        self.distortion_coefficients = config.distortion_coefficients
        mesh = o3d.io.read_triangle_mesh(Path(share_directory) / config.small_cad_model_path)
        mesh.scale(0.001, center=(0, 0, 0))
        self.cad_point_cloud = mesh.sample_points_uniformly(number_of_points=config.image_samples)

    def estimate(self, detections: List, rgb_image: np.ndarray, depth_image: np.ndarray) -> EstimatedPoses:
        """
        Estimates the 6DoF poses of all detected objects in a given scene.

        For each detection, performs PnP using 2D-3D correspondences, filters depth, builds a transformation,
        and optionally refines it using ICP. Returns a message containing all refined poses and associated detections.
        """
        results: EstimatedPoses = EstimatedPoses()
        try:
            for cap_id, detection in enumerate(detections):
                keypoints = detection.keypoints
                ret, rotation_vector, translation_vector = cv2.solvePnP(self.cap_object_points, keypoints,
                                                                        self.intrinsics, self.distortion_coefficients,
                                                                        flags=self.pnp_method)
                if not ret:
                    continue
                rotation_vector = self.assure_upward_rotation(rotation_vector)
                depth = self.get_filtered_depth(depth_image, keypoints[0][0], keypoints[0][1])
                cap_to_camera = self.build_transformation_matrix(rotation_vector,
                                                                 translation_vector.reshape(-1).tolist())
                cap_to_camera[2, 3] = depth
                point_cloud = self.extract_masked_point_cloud(detection.bounding_box, rgb_image, depth_image)
                refined_cap_to_camera = self.refine(initial_guess=cap_to_camera, scene_point_cloud=point_cloud)
                refined_rotation_vector, _ = cv2.Rodrigues(refined_cap_to_camera[:3, :3])
                refined_translation_vector = refined_cap_to_camera[:3, 3]
                results.detections.append(
                    Detection(keypoints=detection.keypoints.flatten(), bounding_box=detection.bounding_box, ))
                results.poses.append(EstimatedPose(cap_id=cap_id, rotation_vector=refined_rotation_vector.flatten(),
                                                   translation_vector=refined_translation_vector.flatten(),
                                                   cap_to_camera=refined_cap_to_camera.flatten(), depth=depth))
        finally:
            return results

    def refine(self, initial_guess: np.ndarray, scene_point_cloud: o3d.geometry.PointCloud) -> np.ndarray:
        """
        Refines an initial transformation using ICP registration between the CAD model and the scene point cloud.

        Uses point-to-point registration and a given initial guess to align the model to the observed data.
        """
        result = o3d.pipelines.registration.registration_icp(source=self.cad_point_cloud, target=scene_point_cloud,
                                                             max_correspondence_distance=0.1, init=initial_guess,
                                                             estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return result.transformation

    def extract_masked_point_cloud(self, bounding_box: np.ndarray, rgb_image: np.ndarray,
                                   depth_image: np.ndarray) -> o3d.geometry.PointCloud:
        """
        Extracts a masked and cropped point cloud from a depth and RGB image based on a bounding box.

        Applies color quantization to mask out background, constructs an RGBD image,
        and creates a point cloud using camera intrinsics. Also removes statistical outliers.
        """
        x_min, y_min, x_max, y_max = bounding_box.ravel().astype(np.int32)
        cropped_rgb = rgb_image[y_min:y_max, x_min:x_max].copy()
        cropped_depth = depth_image[y_min:y_max, x_min:x_max].copy()
        mask = self.color_quantization_to_mask(cropped_rgb)
        masked_rgb = cv2.bitwise_and(cropped_rgb, cropped_rgb, mask=mask)
        masked_depth = np.where(mask > 0, cropped_depth, 0).astype(np.uint16)
        rgb_image_o3d = o3d.geometry.Image(masked_rgb)
        depth_image_o3d = o3d.geometry.Image(masked_depth)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_image_o3d, depth_image_o3d,
                                                                        depth_scale=1000,
                                                                        convert_rgb_to_intensity=False)
        fx, fy = self.intrinsics[0, 0], self.intrinsics[1, 1]
        cx, cy = self.intrinsics[0, 2], self.intrinsics[1, 2]
        intrinsics = o3d.camera.PinholeCameraIntrinsic()
        intrinsics.set_intrinsics(width=x_max - x_min, height=y_max - y_min, fx=fx, fy=fy, cx=cx - x_min, cy=cy - y_min)
        point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)
        point_cloud = point_cloud.remove_statistical_outlier(nb_neighbors=100, std_ratio=1.0)[0]
        return point_cloud

    def assure_upward_rotation(self, rotation_vector: np.ndarray) -> np.ndarray:
        """
        Ensures that the estimated rotation vector maintains a consistent upward orientation.

        Flips the direction of rotation components if necessary to avoid ambiguity in orientation.
        """
        if rotation_vector[0] < 0:
            rotation_vector[0] *= -1
            rotation_vector[1] *= -1
        return rotation_vector

    def get_filtered_depth(self, depth_image: np.ndarray, x: int, y: int, kernel_size: int = 5,
                           convert_to_m: bool = True) -> float:
        """
        Computes a median depth value around a pixel using a kernel for noise reduction.

        Returns the depth in meters if `convert_to_m` is True, otherwise returns in millimeters.
        """
        half_k = kernel_size // 2
        h, w = depth_image.shape
        x, y = int(x), int(y)
        x_min = max(x - half_k, 0)
        x_max = min(x + half_k + 1, w)
        y_min = max(y - half_k, 0)
        y_max = min(y + half_k + 1, h)
        region = depth_image[y_min:y_max, x_min:x_max]
        region = region[region > 0]
        if region.size == 0:
            return 0.0
        scaler: float = 0.001 if convert_to_m else 1.0
        return np.median(region) * scaler  # Convert mm to meters

    def color_quantization_to_mask(self, image: np.ndarray, clusters: int = 2, color_tolerance: int = 10):
        """
        Generates a binary mask by clustering image colors and selecting the brightest cluster.

        Quantizes image into color clusters using K-means, selects the brightest cluster based on HSV value,
        and generates a binary mask for segmenting foreground from background.
        """
        pixel_vectors = image.reshape((-1, 3)).astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        _, labels, centers = cv2.kmeans(pixel_vectors, clusters, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        centers = np.uint8(centers)
        quantized_img = centers[labels.flatten()].reshape(image.shape)
        hsv_centers = cv2.cvtColor(centers.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
        brightness = hsv_centers[:, 2]  # Value (brightness) channel
        lightest_idx = np.argmax(brightness)
        ref_color = centers[lightest_idx]
        lower_bound = np.clip(ref_color - color_tolerance, 0, 255).astype(np.uint8)
        upper_bound = np.clip(ref_color + color_tolerance, 0, 255).astype(np.uint8)
        mask = cv2.inRange(quantized_img, lower_bound, upper_bound)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_OPEN, kernel)
        return mask_cleaned

    def build_transformation_matrix(self, rotation_vector: np.ndarray, translation_vector: np.ndarray) -> np.ndarray:
        """
        Constructs a 4x4 transformation matrix from a rotation and translation vector.

        Uses Rodrigues formula to compute the rotation matrix and inserts it with translation
        into a homogeneous transformation matrix.
        """
        rotation_matrix, _ = cv2.Rodrigues(np.array(rotation_vector))
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation_vector
        return transformation_matrix
