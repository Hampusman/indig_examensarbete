import numpy as np
from logging import Logger
from interfaces.msg import EstimatedPoses
from collections import deque
from typing import Deque, Tuple, List


class PoseStabilityTracker:
    """
    Tracks the temporal stability of detected object poses to determine whether they have remained still.

    Maintains a history of translation vectors over time and checks whether objects have been sufficiently
    stationary to trigger further actions, such as grasping. Handles outlier removal, aging, and consistency checks.
    """

    def __init__(self, max_age: float, time_threshold: float, distance_threshold: float, logger: Logger):
        """
        Initializes the stability tracker with timing and distance thresholds.
        """
        self.max_age = max_age
        self.time_threshold = time_threshold
        self.time_threshold = 1.0
        self.distance_threshold = distance_threshold
        self.history: Deque[Tuple[float, List[np.ndarray]]] = deque()
        self.logger = logger

    def update(self, estimations: EstimatedPoses, timestamp: float) -> None:
        """
        Updates the pose history with the latest frame of pose estimations and associated timestamp.

        Converts translation vectors into NumPy arrays and stores them in a time-ordered buffer.
        Also prunes outdated entries beyond the configured maximum age.
        """
        self.history.append(
            (timestamp, [pose.copy() for pose in [np.array(pose.translation_vector) for pose in estimations.poses]]))
        self.prune_old(timestamp)

    def prune_old(self, current_time: float) -> None:
        """
        Removes historical pose data that exceeds the maximum age threshold.

        Ensures that the internal history buffer only contains recent and relevant data points.
        """
        self.history = deque([(t, ps) for (t, ps) in self.history if current_time - t <= self.max_age])

    def clear(self) -> None:
        """
        Clears the entire history of tracked poses.

        Typically used when inconsistent data or motion is detected that invalidates tracking.
        """
        self.history.clear()

    def has_been_still(self) -> bool:
        """
        Evaluates whether all tracked poses have remained within a distance threshold over time.

        Returns True if the number of tracked poses is consistent and no object has moved significantly,
        for at least the required duration. Otherwise, returns False.
        """
        if not self.history:
            self.logger.info('No pose history yet.')
            return False
        first_time = self.history[0][0]
        last_time = self.history[-1][0]
        stationary_duration = last_time - first_time
        if stationary_duration < self.time_threshold:
            self.logger.info(f'Stationary for {stationary_duration:.2f}s (need {self.time_threshold}s).')
            return False
        n_caps = len(self.history[0][1])
        if any(len(positions) != n_caps for _, positions in self.history):
            self.logger.warning('Inconsistent cap count detected — treating as movement.')
            self.clear()
            return False
        all_positions = [[] for _ in range(n_caps)]
        for _, positions in self.history:
            for i, pos in enumerate(positions):
                all_positions[i].append(pos)
        for cap_idx, cap_positions in enumerate(all_positions):
            reference = cap_positions[-1]
            distances = [np.linalg.norm(p - reference) for p in cap_positions]
            max_distance = max(distances)
            if any(d > self.distance_threshold for d in distances):
                self.logger.info(
                    f'Cap {cap_idx} moved {max_distance:.4f}m (threshold: {self.distance_threshold:.4f}m) — resetting.')
                self.clear()
                return False
        self.logger.info(f'Stationary for {stationary_duration:.2f}s — safe to proceed.')
        return True
