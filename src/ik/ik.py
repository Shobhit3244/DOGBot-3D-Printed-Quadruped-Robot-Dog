import math
import yaml

class LegIK:
    """
    2-DOF leg IK/FK for robot dog (hip and knee only).
    Shoulder angles are fixed at stand positions.
    All calculations use degrees (no radians).
    Lengths in cm loaded from limb_lengths.yaml.
    Legs are numbered: 0=front_left, 1=front_right, 2=back_left, 3=back_right
    """

    def __init__(self, limb_cfg='../configs/limb_lengths.yaml'):
        with open(limb_cfg, 'r') as f:
            cfg = yaml.safe_load(f)
        
        self.L1 = cfg['shoulder_length']     # 6.0 cm
        self.L2 = cfg['upper_arm_length']    # 10.6 cm  
        self.L3 = cfg['fore_arm_length']     # 13.5 cm
        
        self.home_xyz = {}
        self.shoulder_stand_angles = {}  # Store fixed shoulder angles
        
        # Leg number to name mapping
        self.leg_names = {
            0: 'front_left',
            1: 'front_right', 
            2: 'back_left',
            3: 'back_right'
        }
        
        # Reverse mapping
        self.name_to_num = {v: k for k, v in self.leg_names.items()}

    # Degree-based trigonometric functions
    def sind(self, angle_deg):
        return math.sin(math.radians(angle_deg))
    
    def cosd(self, angle_deg):  
        return math.cos(math.radians(angle_deg))
    
    def atan2d(self, y, x):
        return math.degrees(math.atan2(y, x))
    
    def acosd(self, x):
        return math.degrees(math.acos(max(-1, min(1, x))))

    def set_shoulder_stand_angles(self, offsets, joint_map):
        """Store shoulder stand angles so they can be returned by IK"""
        for leg_num in range(4):
            shoulder_joint = joint_map[leg_num][0]  # First joint is shoulder
            self.shoulder_stand_angles[leg_num] = offsets[shoulder_joint]['stand']
        
        print("Shoulder stand angles loaded:")
        for leg_num, angle in self.shoulder_stand_angles.items():
            print(f"  Leg {leg_num} ({self.leg_names[leg_num]}): {angle}°")

    def fk(self, th_h, th_k):
        """
        Forward kinematics: hip and knee angles to foot XY position
        th_h: hip angle (degrees)
        th_k: knee angle (degrees)
        Returns: (x, y) coordinates
        """
        # Calculate reach and height using only hip and knee
        R = self.L2 * self.cosd(th_h) + self.L3 * self.cosd(th_h + th_k)
        H = self.L2 * self.sind(th_h) + self.L3 * self.sind(th_h + th_k)
        
        x = R  # Forward/backward distance
        y = self.L1 - H  # Up/down from shoulder height
        
        return x, y

    def ik(self, x, y, leg_num=0):
        """
        2-DOF Inverse kinematics: foot XY position to hip and knee angles
        x: forward(+)/backward(-) position (cm)
        y: up(+)/down(-) position relative to shoulder (cm) 
        leg_num: leg number for getting correct shoulder angle
        Returns: {'shoulder': fixed_angle, 'hip': hip_angle, 'knee': knee_angle}
        """
        try:
            # Calculate distance from shoulder joint to foot
            dy = self.L1 - y  # Distance below shoulder
            d = math.hypot(x, dy)
            
            # Check reachability  
            max_reach = self.L2 + self.L3
            min_reach = abs(self.L2 - self.L3)
            
            if d > max_reach - 1e-6:
                print(f"Warning: Target ({x:.1f}, {y:.1f}) beyond max reach {max_reach:.1f}, clamping")
                d = max_reach - 1e-6
            elif d < min_reach + 1e-6:
                print(f"Warning: Target ({x:.1f}, {y:.1f}) within min reach {min_reach:.1f}, clamping") 
                d = min_reach + 1e-6
            
            # Calculate knee angle using law of cosines
            c_k = (self.L2**2 + self.L3**2 - d**2) / (2 * self.L2 * self.L3)
            th_k = self.acosd(c_k)
            
            # Calculate hip angle
            alpha = self.atan2d(dy, x)
            c_a = (self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d)
            beta = self.acosd(c_a)
            th_h = alpha + beta
            
            # Get fixed shoulder angle for this leg
            shoulder_angle = self.shoulder_stand_angles.get(leg_num, 90)
            
            return {
                'shoulder': shoulder_angle,  # FIXED at stand angle
                'hip': th_h,                # Calculated
                'knee': th_k                # Calculated  
            }
            
        except Exception as e:
            print(f"IK calculation failed for position ({x:.1f}, {y:.1f}): {e}")
            # Return default safe angles
            shoulder_angle = self.shoulder_stand_angles.get(leg_num, 90)
            return {'shoulder': shoulder_angle, 'hip': 90, 'knee': 90}

    def compute_home_from_offsets(self, offsets, joint_map):
        """
        Compute home XY coordinates from stand angles for all legs using geometry.
        Uses the formulas:
        h = sqrt(ual² + fal² - (cos(knee_angle) * 2 * ual * fal))
        c = arccos((ual² + h² - fal²) / (2 * ual * h))
        theta = 90 - hip_angle + c
        x = h * sin(theta)
        y = h * cos(theta)
        """
        print("Computing home positions from stand angles using leg geometry:")
        
        # First, store shoulder stand angles
        self.set_shoulder_stand_angles(offsets, joint_map)
        
        for leg_num in range(4):
            leg_name = self.leg_names[leg_num]
            joints = joint_map[leg_num]
            
            # Get stand angles for this leg
            hip_angle = offsets[joints[1]]['stand']    # Hip 
            knee_angle = offsets[joints[2]]['stand']   # Knee
            
            print(f"\nLeg {leg_num} ({leg_name}):")
            print(f"  Stand angles: hip={hip_angle}°, knee={knee_angle}°")
            
            try:
                # Calculate using the specified formulas
                ual = self.L2  # Upper arm length  
                fal = self.L3  # Forearm length
                
                # h = sqrt(ual² + fal² - (cos(knee_angle) * 2 * ual * fal))
                cos_knee = self.cosd(knee_angle)
                h_squared = ual**2 + fal**2 - (cos_knee * 2 * ual * fal)
                
                if h_squared < 0:
                    print(f"  Warning: Invalid geometry, using minimum reach")
                    h = abs(ual - fal)
                else:
                    h = math.sqrt(h_squared)
                
                # c = arccos((ual² + h² - fal²) / (2 * ual * h))
                if h == 0:
                    c = 0
                else:
                    c_arg = (ual**2 + h**2 - fal**2) / (2 * ual * h)
                    c = self.acosd(c_arg)
                
                # theta = 90 - hip_angle + c  
                theta = 90 - hip_angle + c
                
                # x = h * sin(theta), y = h * cos(theta)
                x = h * self.sind(theta)
                y = h * self.cosd(theta)
                
                print(f"  Calculations: h={h:.2f}cm, c={c:.2f}°, theta={theta:.2f}°")
                print(f"  Home position: x={x:.2f}cm, y={y:.2f}cm")
                
                self.home_xyz[leg_num] = {'x': x, 'y': y}
                
            except Exception as e:
                print(f"  Error calculating home position: {e}")
                print(f"  Using default position (13.5, 10.6)")
                self.home_xyz[leg_num] = {'x': 13.5, 'y': 10.6}

    def get_all_home_positions(self):
        """Return all home positions as {leg_num: {'x': x, 'y': y}}"""
        return self.home_xyz

    def get_leg_home_position(self, leg_num):
        """Return home position for specific leg"""
        return self.home_xyz.get(leg_num, {'x': 0, 'y': 0})