import yaml
import cv2
import numpy as np
from dataclasses import dataclass


@dataclass
class Config:
    # Paths
    yolo_model_path: str
    small_cad_model_path: str
    large_cad_model_path: str

    # Camera settings
    intrinsics: np.ndarray
    distortion_coefficients: np.ndarray

    # YOLO and Pose estimator
    yolo_confidence: float
    pnp_method: int
    image_samples: int

    # Points and zones
    cap_object_points: np.ndarray

    @classmethod
    def from_yaml(cls, path: str) -> "Config":
        raw = yaml.safe_load(open(path))
        try:
            return cls(
                yolo_model_path=raw["paths"]["yolo_model_path"],
                small_cad_model_path=raw["paths"]["small_cad_model_path"],
                large_cad_model_path=raw["paths"]["large_cad_model_path"],
                intrinsics=np.array(raw["camera"]["intrinsics"]),
                distortion_coefficients=np.array(raw["camera"]["distortion_coefficients"]),
                yolo_confidence=raw["yolo"]["confidence_threshold"],
                pnp_method=getattr(cv2, raw["pose_estimation"]["pnp_method"]),
                image_samples=raw["pose_estimation"]["image_samples"],
                cap_object_points=np.array(raw["points"]["cap_object_points"]),
            )
        except KeyError as e:
            raise ValueError(f'Missing expected key "{e}"')