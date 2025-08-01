import os
import time
import board
import busio
from adafruit_pca9685 import PCA9685
import yaml


class ServoController:
    def __init__(self, servo_map_file, calib_file, offset_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Load servo map
        servo_map_path = os.path.abspath(os.path.join(script_dir, servo_map_file))
        if not os.path.exists(servo_map_path):
            raise FileNotFoundError(f"Servo map config not found: {servo_map_path}")
        with open(servo_map_path, 'r') as f:
            self.servo_map = yaml.safe_load(f)

        # Load calibration (min, mid ignored, max)
        calib_path = os.path.abspath(os.path.join(script_dir, calib_file))
        if not os.path.exists(calib_path):
            raise FileNotFoundError(f"Calibration file not found: {calib_path}")
        with open(calib_path, 'r') as f:
            self.calib = yaml.safe_load(f)

        # Load offsets and stand positions
        offset_path = os.path.abspath(os.path.join(script_dir, offset_file))
        if not os.path.exists(offset_path):
            raise FileNotFoundError(f"Offset file not found: {offset_path}")
        with open(offset_path, 'r') as f:
            self.offsets = yaml.safe_load(f)

        # Initialize PCA9685 controller on I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # 50 Hz for standard hobby servos

    def _angle_to_duty_cycle(self, angle):
        # Convert 0-180 angle to PWM pulse width and duty cycle for PCA9685
        pulse_us = int((angle / 180.0) * 2000 + 500)  # 500us to 2500us typical servo pulse
        pulse_us = max(500, min(2500, pulse_us))
        duty_cycle = int(pulse_us / 20000 * 0xFFFF)  # Normalize for 20ms period
        return duty_cycle

    def set_servo_angle(self, joint_name, input_angle):
        """
        Set servo to absolute input_angle (0-180), applying offset correction first,
        then clamp the corrected angle within min/max calibration limits.

        Parameters:
        - joint_name: servo name string as in YAML configs
        - input_angle: absolute desired angle (degrees)
        """
        if joint_name not in self.servo_map:
            raise KeyError(f"Joint '{joint_name}' not in servo map")
        if joint_name not in self.calib:
            raise KeyError(f"Calibration missing for joint '{joint_name}'")
        if joint_name not in self.offsets:
            raise KeyError(f"Offset info missing for joint '{joint_name}'")

        cal = self.calib[joint_name]
        off = self.offsets[joint_name]
        min_ang = cal['min']
        max_ang = cal['max']
        offset_val = off.get('offset', 0)

        # Apply offset correction
        if offset_val == 0:
            corrected_angle = input_angle
        elif offset_val == 180:
            corrected_angle = 180 - input_angle
            if corrected_angle < 0:
                corrected_angle = -corrected_angle
        else:
            raise ValueError(f"Offset must be 0 or 180 for joint '{joint_name}', got {offset_val}")

        # Clamp within absolute min/max
        if min_ang <= max_ang:
            corrected_angle = max(min_ang, min(corrected_angle, max_ang))
        else:
            # inverted range: clamp carefully
            if not (corrected_angle > min_ang or corrected_angle < max_ang):
                corrected_angle = min_ang if abs(corrected_angle - min_ang) <= abs(corrected_angle - max_ang) else max_ang

        duty_cycle = self._angle_to_duty_cycle(corrected_angle)
        channel = self.servo_map[joint_name]
        self.pca.channels[channel].duty_cycle = duty_cycle

    def set_servo_absolute_no_offset(self, joint_name, angle):
        """
        Set servo to absolute angle directly without offset correction,
        clamped safely to min and max limits.
        Intended mainly for calibration and testing mode.
        """
        if joint_name not in self.servo_map:
            raise KeyError(f"Joint '{joint_name}' not in servo map")
        if joint_name not in self.calib:
            raise KeyError(f"Calibration missing for joint '{joint_name}'")

        cal = self.calib[joint_name]
        min_ang = cal['min']
        max_ang = cal['max']

        if min_ang <= max_ang:
            safe_angle = max(min_ang, min(angle, max_ang))
        else:
            # inverted range clamp
            if not (angle > min_ang or angle < max_ang):
                safe_angle = min_ang if abs(angle - min_ang) <= abs(angle - max_ang) else max_ang
            else:
                safe_angle = angle

        duty_cycle = self._angle_to_duty_cycle(safe_angle)
        channel = self.servo_map[joint_name]
        self.pca.channels[channel].duty_cycle = duty_cycle

    def stand(self):
        """
        Move all servos to their 'stand' positions as per offsets file,
        without applying offset or clamping beyond limits.
        """
        for joint_name in self.servo_map:
            if joint_name not in self.offsets:
                print(f"Stand position missing for servo '{joint_name}', skipping")
                continue

            stand_angle = self.offsets[joint_name].get('stand', None)
            if stand_angle is None:
                print(f"'stand' value missing for servo '{joint_name}', skipping")
                continue

            cal = self.calib[joint_name]
            min_ang = cal['min']
            max_ang = cal['max']

            # Clamp stand_angle within min/max limits
            if min_ang <= max_ang:
                safe_angle = max(min_ang, min(stand_angle, max_ang))
            else:
                if not (stand_angle > min_ang or stand_angle < max_ang):
                    safe_angle = min_ang if abs(stand_angle - min_ang) <= abs(stand_angle - max_ang) else max_ang
                else:
                    safe_angle = stand_angle

            duty_cycle = self._angle_to_duty_cycle(safe_angle)
            channel = self.servo_map[joint_name]
            self.pca.channels[channel].duty_cycle = duty_cycle

    def move_servos_interpolated(self, joint_targets, current_positions, steps_per_second=50):
        """
        Smoothly move multiple servos from current_positions to joint_targets
        with linear interpolation at the specified speed (steps per second).

        Params:
        - joint_targets: {joint_name: target_angle}
        - current_positions: {joint_name: current_angle}
        - steps_per_second: int, number of interpolation steps per second
        """
        joints = list(joint_targets.keys())
        max_delta = max(abs(joint_targets[j] - current_positions.get(j, joint_targets[j])) for j in joints)

        if max_delta == 0:
            # Already at target position, no move needed
            return

        total_steps = int(max_delta * steps_per_second / 60)  # approx steps proportional to max delta
        total_steps = max(1, total_steps)  # at least one step

        for step in range(1, total_steps + 1):
            fraction = step / total_steps
            for j in joints:
                start = current_positions.get(j, joint_targets[j])
                end = joint_targets[j]
                interp_angle = start + fraction * (end - start)
                self.set_servo_angle(j, interp_angle)
            time.sleep(1.0 / steps_per_second)


# ===== CONFIGURATOR FUNCTIONS =====

def load_calib(calib_path, servo_names):
    """Load calibration data, creating defaults for missing servos"""
    if os.path.exists(calib_path):
        with open(calib_path, 'r') as f:
            calib = yaml.safe_load(f)
        for name in servo_names:
            if name not in calib:
                calib[name] = {'min': 0, 'mid': 90, 'max': 180}
    else:
        calib = {name: {'min': 0, 'mid': 90, 'max': 180} for name in servo_names}
    return calib


def save_calib(calib_path, calib):
    """Save calibration data to YAML file"""
    with open(calib_path, 'w') as f:
        yaml.dump(calib, f)
    print("Calibration saved.")


def motor_index_to_joint(servo_map):
    """Convert servo map to channel->joint mapping"""
    return {int(v): k for k, v in servo_map.items()}


def print_menu():
    """Print the configurator main menu"""
    print("\n--- Servo Configurator ---")
    print("Select mode:")
    print("1: Set servo angles (absolute, no offset)")
    print("2: Set servo limits (min, mid, max)")
    print("q: Quit")


def set_angle_mode(ctrl, channel_to_joint, calib):
    """Interactive mode for setting servo angles"""
    print("\nSet Angle Mode: Enter command pairs '<channel> <angle>' (0-180 absolute angles).")
    print("Commands: b = back, q = quit")

    while True:
        line = input("Set angles> ").strip()
        if line.lower() == 'q':
            print("Quitting configurator.")
            exit(0)
        if line.lower() == 'b' or not line:
            print("Returning to main menu.")
            break

        tokens = line.split()
        if len(tokens) % 2 != 0:
            print("Error: Enter pairs in format '<channel> <angle>'.")
            continue

        try:
            for i in range(0, len(tokens), 2):
                idx = int(tokens[i])
                angle = int(tokens[i + 1])

                if idx not in channel_to_joint:
                    print(f"Warning: Motor index {idx} not found, skipping.")
                    continue

                joint = channel_to_joint[idx]
                # Move servo directly with no offset correction
                ctrl.set_servo_absolute_no_offset(joint, angle)
                print(f"Set servo '{joint}' (channel {idx}) to {angle}° (absolute).")
                time.sleep(0.05)
        except Exception as e:
            print(f"Invalid input or error: {e}")


def set_limits_mode(calib, channel_to_joint):
    """Interactive mode for setting servo limits"""
    print("\nSet Limits Mode: Input quadruples '<channel> <min> <mid> <max>'.")
    print("Angles should be absolute values between 0-180 (inverted allowed).")
    print("Commands: b = back, q = quit")

    while True:
        line = input("Set limits> ").strip()
        if line.lower() == 'q':
            print("Quitting configurator.")
            exit(0)
        if line.lower() == 'b' or not line:
            print("Returning to main menu.")
            break

        tokens = line.split()
        if len(tokens) % 4 != 0:
            print("Error: Enter quadruples in format '<channel> <min> <mid> <max>'.")
            continue

        try:
            for i in range(0, len(tokens), 4):
                idx = int(tokens[i])
                min_angle = int(tokens[i + 1])
                mid_angle = int(tokens[i + 2])
                max_angle = int(tokens[i + 3])

                if idx not in channel_to_joint:
                    print(f"Warning: Motor index {idx} not found, skipping.")
                    continue

                joint = channel_to_joint[idx]

                if any(val < 0 or val > 180 for val in [min_angle, mid_angle, max_angle]):
                    print(f"Warning: Angles for '{joint}' must be between 0 and 180. Skipping update.")
                    continue

                calib[joint] = {'min': min_angle, 'mid': mid_angle, 'max': max_angle}
                print(f"Set limits for '{joint}': min={min_angle}, mid={mid_angle}, max={max_angle}")
        except Exception as e:
            print(f"Invalid input or error: {e}")


def run_configurator():
    """Main configurator function"""
    # Configuration file paths (relative to this script)
    MAP_PATH = '../../configs/servo_map.yaml'
    CALIB_PATH = '../../configs/servo_calib.yaml'
    OFFSET_PATH = '../../configs/servo_offset.yaml'

    if not os.path.exists(MAP_PATH):
        print(f"Error: Servo map file not found: {MAP_PATH}")
        return

    with open(MAP_PATH, 'r') as f:
        servo_map = yaml.safe_load(f)

    servo_names = list(servo_map.keys())
    calib = load_calib(CALIB_PATH, servo_names)
    channel_to_joint = motor_index_to_joint(servo_map)

    ctrl = ServoController(MAP_PATH, CALIB_PATH, OFFSET_PATH)

    while True:
        print_menu()
        choice = input("Select mode> ").strip().lower()

        if choice == '1':
            set_angle_mode(ctrl, channel_to_joint, calib)
        elif choice == '2':
            set_limits_mode(calib, channel_to_joint)
            save_calib(CALIB_PATH, calib)
        elif choice == 'q':
            print("Exiting configurator.")
            save_calib(CALIB_PATH, calib)
            break
        else:
            print("Invalid choice. Please enter 1, 2 or q.")


if __name__ == "__main__":
    # When run as standalone script, launch the configurator
    run_configurator()
