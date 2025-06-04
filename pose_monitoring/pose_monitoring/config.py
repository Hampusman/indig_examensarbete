import yaml
import numpy as np
from dataclasses import dataclass


@dataclass
class PoseStabilityTrackerConfig:
    time_threshold: float
    distance_threshold: float
    max_pose_age: float

    @classmethod
    def from_yaml(cls, path: str) -> "PoseStabilityTrackerConfig":
        raw = yaml.safe_load(open(path))
        try:
            return cls(
                time_threshold=raw["selector"]["time_threshold"],
                distance_threshold=raw["selector"]["distance_threshold"],
                max_pose_age=raw["selector"]["max_pose_age"],
            )
        except KeyError as e:
            raise ValueError(f'Missing expected key "{e}"')


@dataclass
class FeasibilityFilterConfig:
    orientation_threshold_deg: float
    depth_threshold: float
    clearance_region: np.ndarray
    intrinsics: np.ndarray
    distortion_coefficients: np.ndarray

    @classmethod
    def from_yaml(cls, path: str) -> "FeasibilityFilterConfig":
        raw = yaml.safe_load(open(path))
        try:
            return cls(
                orientation_threshold_deg=raw["selector"]["orientation_threshold_deg"],
                depth_threshold=raw["selector"]["depth_threshold"],
                clearance_region=np.array(raw["selector"]["clearance_region"]),
                intrinsics=np.array(raw["camera"]["intrinsics"]),
                distortion_coefficients=np.array(raw["camera"]["distortion_coefficients"]),
            )
        except KeyError as e:
            raise ValueError(f'Missing expected key "{e}"')
