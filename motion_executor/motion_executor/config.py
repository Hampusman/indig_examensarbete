import yaml
import numpy as np
from dataclasses import dataclass


@dataclass
class Config:

    tool_tcp: np.ndarray
    camera_to_tool: np.ndarray

    @classmethod
    def from_yaml(cls, path: str) -> "Config":
        raw = yaml.safe_load(open(path))
        try:
            return cls(
                tool_tcp=np.array(raw['tool']['cap_gripper_s']),
                camera_to_tool=np.array(raw['tool']['cam_color_sensor_tcp']),

            )
        except KeyError as e:
            raise ValueError(f'Missing expected key "{e}"')