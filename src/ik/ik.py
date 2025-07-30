import math
import yaml

class LegIK:
    """
    3-DOF leg IK/FK for robot dog.
    Shoulder = yaw; hip/knee = pitch.
    Lengths in cm loaded from limb_lengths.yaml.
    """

    def __init__(self, limb_cfg='../configs/limb_lengths.yaml'):
        with open(limb_cfg, 'r') as f:
            cfg = yaml.safe_load(f)
        self.L1 = cfg['shoulder_length']
        self.L2 = cfg['upper_arm_length']
        self.L3 = cfg['fore_arm_length']
        self.home_xyz = {}

    def fk(self, th_s, th_h, th_k):
        # Forward kinematics: angles to foot xyz
        s, h, k = map(math.radians, (th_s, th_h, th_k))
        R = self.L2 * math.cos(h) + self.L3 * math.cos(h + k)
        H = self.L2 * math.sin(h) + self.L3 * math.sin(h + k)
        x = R * math.cos(s)
        y = self.L1 - H
        z = R * math.sin(s)
        return x, y, z

    def ik(self, x, y, z):
        # Inverse kinematics: foot xyz to angles
        th_s = math.degrees(math.atan2(z, x))
        r = math.hypot(x, z)
        dy = self.L1 - y
        d = max(min(math.hypot(r, dy), self.L2 + self.L3 - 1e-6), abs(self.L2 - self.L3) + 1e-6)
        c_k = (self.L2**2 + self.L3**2 - d**2) / (2 * self.L2 * self.L3)
        th_k = math.degrees(math.acos(c_k))

        alpha = math.atan2(dy, r)
        c_a = (self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d)
        beta = math.acos(c_a)
        th_h = math.degrees(alpha + beta)
        return {'shoulder': th_s, 'hip': th_h, 'knee': th_k}

    def compute_home_from_offsets(self, offsets, joint_map):
        # Compute home XYZ coords from stand angles for all legs
        for leg, joints in joint_map.items():
            s = offsets[joints[0]]['stand']
            h = offsets[joints[1]]['stand']
            k = offsets[joints[2]]['stand']
            x, y, z = self.fk(s, h, k)
            self.home_xyz[leg] = {'x': x, 'y': y, 'z': z}

    def get_all_home_positions(self):
        return self.home_xyz

    def get_leg_home_position(self, leg):
        return self.home_xyz.get(leg, {'x': 0, 'y': 0, 'z': 0})
