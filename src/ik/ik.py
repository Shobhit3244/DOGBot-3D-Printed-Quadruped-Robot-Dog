import os
import numpy as np
import yaml

class LegIK:
    def __init__(self, config_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.abspath(os.path.join(script_dir, config_file))
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Limb lengths config not found: {config_path}")
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        self.shoulder_len = cfg['shoulder_length']
        self.upper_arm_len = cfg['upper_arm_length']
        self.fore_arm_len = cfg['fore_arm_length']

    def solve(self, x, y, z):
        """
        Computes joint angles as offsets relative to midpoint.
        Inputs:
          - x, y, z: target foot position relative to shoulder joint in cm.
        Returns:
          - dict with 'shoulder', 'hip', 'knee' angles in degrees.
        """
        # Shoulder abduction/adduction (servo 1)
        theta1 = np.arctan2(z, x)

        # Projection into YX plane with offset
        r = np.hypot(x, z)
        y1 = y - self.shoulder_len

        dist = np.hypot(r, y1)

        a = self.upper_arm_len
        b = self.fore_arm_len
        cos_knee = (dist**2 - a**2 - b**2) / (2*a*b)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)
        theta3 = np.arccos(cos_knee)

        k1 = a + b * np.cos(theta3)
        k2 = b * np.sin(theta3)
        theta2 = np.arctan2(y1, r) - np.arctan2(k2, k1)

        angles = {
            'shoulder': np.degrees(theta1),
            'hip': np.degrees(theta2),
            'knee': np.degrees(theta3)
        }
        return angles
