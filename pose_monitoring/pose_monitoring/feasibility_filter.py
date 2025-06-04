import cv2
import numpy as np
from interfaces.msg import EstimatedPoses
from logging import Logger


class FeasibilityFilter:
    """
    Evaluates the feasibility of picking objects based on depth constraints in a defined clearance region.

    Uses projected 3D geometry and depth image data to ensure sufficient space around objects
    for successful grasping. Supports optional debug visualization.
    """
    def __init__(self, depth_threshold: float, clearance_region: np.ndarray, intrinsics: np.ndarray,
                 distortion_coefficients: np.ndarray, logger: Logger, debug: bool = False):
        """
        Initializes the feasibility filter with camera parameters, depth threshold, and logging options.
        """
        self.depth_threshold = depth_threshold
        self.clearance_region = clearance_region
        self.intrinsics = intrinsics
        self.distortion_coefficients = distortion_coefficients
        self.logger = logger
        self.debug = debug
        self.depth_image = None
        self.draw_image = None

    def get_feasible_poses(self, estimations: EstimatedPoses, depth_image: np.ndarray) -> EstimatedPoses:
        """
        Filters a set of estimated poses based on geometric clearance in the depth image.

        For each pose, checks whether sufficient free space exists around the object by projecting a clearance
        polygon and analyzing depth values. Returns only the poses deemed feasible for picking.
        """
        self.depth_image = depth_image
        self.draw_image = cv2.cvtColor(depth_image.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        feasible_poses = EstimatedPoses()
        for pose in estimations.poses:
            feasible = self.check_feasibility(pose.rotation_vector, pose.translation_vector)
            if feasible:
                feasible_poses.poses.append(pose)
        if self.debug:
            cv2.imshow("feasible poses", self.draw_image)
            cv2.waitKey(1)
        return feasible_poses

    def check_feasibility(self, rotation_vector: np.ndarray, translation_vector: np.ndarray) -> bool:
        """
        Determines whether a single pose is pickable by comparing its depth to surrounding points.

        Projects a 3D clearance region into the image, masks the corresponding area in the depth map,
        and checks if any nearby obstacles violate the required clearance threshold.
        """
        cap_depth = translation_vector[2]
        clearance_region, _ = cv2.projectPoints(self.clearance_region, rotation_vector, translation_vector,
                                                self.intrinsics, self.distortion_coefficients)
        depth_threshold = self.depth_threshold + cap_depth
        if cap_depth is None:
            self.logger.error('Cap depth is None')
            return False
        if cap_depth >= depth_threshold:
            self.logger.error(f'Cap depth ({cap_depth:.3f} m) exceeds threshold ({depth_threshold:.3f} m).')
            return False
        mask = np.zeros(self.depth_image.shape, dtype=np.uint8)
        polygon = np.round(clearance_region).astype(np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(mask, [polygon], 1)
        masked_depth = self.depth_image.copy()
        masked_depth[mask == 0] = 0
        valid_depths = masked_depth[masked_depth > 0]
        if valid_depths.size == 0:
            self.logger.error('No valid depth found in polygon.')
            return False
        min_depth = np.min(valid_depths) * 0.001
        mean_depth = np.mean(valid_depths) * 0.001
        if self.debug:
            self.logger.info(
                f'Cap depth: {cap_depth:.3f} m | Min polygon depth: {min_depth:.3f} m | "f"Mean polygon depth: {mean_depth:.3f} m | Threshold: {depth_threshold:.3f} m')
        feasible = min_depth > depth_threshold
        return feasible
