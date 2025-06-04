import yaml
import numpy as np
from dataclasses import dataclass


@dataclass
class Config:
    # Camera settings
    depth_threshold: float
    clearance_region: np.ndarray
    intrinsics: np.ndarray
    distortion_coefficients: np.ndarray

    @classmethod
    def from_yaml(cls, path: str) -> "Config":
        raw = yaml.safe_load(open(path))
        try:
            return cls(
                depth_threshold=raw["selector"]["depth_threshold"],
                clearance_region=np.array(raw["selector"]["clearance_region"]),
                intrinsics=np.array(raw["camera"]["intrinsics"]),
                distortion_coefficients=np.array(raw["camera"]["distortion_coefficients"]),
            )
        except KeyError as e:
            raise ValueError(f'Missing expected key "{e}"')