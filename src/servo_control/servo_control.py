import os
import board
import busio
from adafruit_pca9685 import PCA9685
import yaml

class ServoController:
    def __init__(self, servo_map_file, calib_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))

        servo_map_path = os.path.abspath(os.path.join(script_dir, servo_map_file))
        if not os.path.exists(servo_map_path):
            raise FileNotFoundError(f"Servo map config not found: {servo_map_path}")
        with open(servo_map_path, 'r') as f:
            self.servo_map = yaml.safe_load(f)

        calib_path = os.path.abspath(os.path.join(script_dir, calib_file))
        if not os.path.exists(calib_path):
            raise FileNotFoundError(f"Calibration file not found: {calib_path}")
        with open(calib_path, 'r') as f:
            self.calib = yaml.safe_load(f)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # Standard servo frequency for hobby servos

    def set_servo_angle(self, joint_name, angle_offset):
        """
        Set servo angle with calibration:
        angle_offset: degrees relative to midpoint (calibrated neutral position)
        """
        if joint_name not in self.servo_map:
            raise KeyError(f"Joint '{joint_name}' not in servo map")
        if joint_name not in self.calib:
            raise KeyError(f"Calibration missing for joint '{joint_name}'")

        cal = self.calib[joint_name]
        min_angle = cal['min']
        max_angle = cal['max']
        mid_angle = cal['mid']

        abs_angle = mid_angle + angle_offset
        abs_angle = max(min_angle, min(max_angle, abs_angle))

        channel = self.servo_map[joint_name]

        # Convert absolute angle to microseconds pulse (typical 500-2500 µs)
        pulse_us = int((abs_angle / 180.0) * 2000 + 500)
        pulse_us = max(500, min(2500, pulse_us))

        duty_cycle = int(pulse_us / 20000 * 0xFFFF)  # Normalize 20ms period

        self.pca.channels[channel].duty_cycle = duty_cycle

if __name__ == "__main__":
    ctrl = ServoController('../../configs/servo_map.yaml', '../../configs/servo_calib.yaml')
    print("Moving all servos to their midpoints...")
    for joint in ctrl.servo_map:
        ctrl.set_servo_angle(joint, 0)  # 0 offset means midpoint position
