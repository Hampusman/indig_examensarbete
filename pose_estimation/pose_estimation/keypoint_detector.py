import os
import numpy as np
from ultralytics import YOLO
from .config import Config
from typing import List
from dataclasses import dataclass
from interfaces.msg import Detection


@dataclass
class DetectionResult:
    """
    Data structure representing a single detection result.

    Contains the predicted keypoints, bounding box coordinates, and confidence score of the detection.
    """
    keypoints: List[np.ndarray]
    bounding_box: List[float]
    confidence: float


class KeypointDetector:
    """
    Handles keypoint detection using a pretrained YOLO model.

    Loads the model based on configuration parameters and performs inference on RGB images to extract keypoints and bounding boxes.
    """
    def __init__(self, config: Config, share_directory: str):
        """
        Initializes the keypoint detector with a YOLO model and confidence threshold.

        Loads the model from the specified path using GPU acceleration and stores the detection confidence threshold.
        """
        self.model = YOLO(os.path.join(share_directory, config.yolo_model_path)).to('cuda')
        self.confidence = config.yolo_confidence


    def detect(self, image: np.ndarray) -> List[DetectionResult]:
        """
        Performs keypoint detection on a given RGB image using the loaded YOLO model.

        Returns a list of detection results, each containing keypoints, bounding box coordinates, and confidence score.
        """
        detections = []
        results = self.model(image, save=False, conf=self.confidence, verbose=False)[0]
        for keypoints, box, score in zip(results.keypoints.xy, results.boxes.xyxy, results.boxes.conf):
            detections.append(DetectionResult(keypoints.cpu().numpy(), box.cpu().numpy(), float(score.cpu())))
        return detections
