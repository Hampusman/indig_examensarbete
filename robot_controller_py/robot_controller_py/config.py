import yaml
import numpy as np
from dataclasses import dataclass


@dataclass
class Config:
    tool_tcp: np.ndarray


    @classmethod
    def from_yaml(cls, path: str) -> "Config":
        raw = yaml.safe_load(open(path))
        try:
            return cls(
                tool_tcp=raw['tool']['cap_gripper_s'],
            )
        except KeyError as e:
            raise ValueError(f'Missing expected key "{e}"')