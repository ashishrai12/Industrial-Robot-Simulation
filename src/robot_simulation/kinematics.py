import numpy as np
from typing import Tuple, Optional

class PlanarKinematics2D:
    """
    Handles Forward and Inverse Kinematics for a 2-link planar robot arm.
    """
    def __init__(self, l1: float, l2: float):
        self.l1 = l1
        self.l2 = l2

    def forward(self, theta1: float, theta2: float) -> np.ndarray:
        """
        Calculates the (x, y) position of the end effector.
        """
        x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
        y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
        return np.array([x, y])

    def inverse(self, target_x: float, target_y: float, elbow_up: bool = True) -> Optional[Tuple[float, float]]:
        """
        Calculates theta1 and theta2 required to reach (target_x, target_y).
        Returns None if the target is unreachable.
        """
        d_sq = target_x**2 + target_y**2
        
        # Law of cosines for theta2
        cos_theta2 = (d_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        
        if np.abs(cos_theta2) > 1.0:
            return None # Target out of reach
            
        theta2 = np.arccos(cos_theta2)
        if not elbow_up:
            theta2 = -theta2
            
        theta1 = np.arctan2(target_y, target_x) - np.arctan2(self.l2 * np.sin(theta2), self.l1 + self.l2 * np.cos(theta2))
        
        return theta1, theta2

    def get_joint_positions(self, theta1: float, theta2: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns the positions of the base, joint 1, and end effector.
        Useful for visualization.
        """
        p0 = np.array([0, 0])
        p1 = np.array([self.l1 * np.cos(theta1), self.l1 * np.sin(theta1)])
        p2 = self.forward(theta1, theta2)
        return p0, p1, p2
