import os
import yaml
import time
from servo_control import ServoController

MAP_PATH = '../../configs/servo_map.yaml'
CALIB_PATH = '../../configs/servo_calib.yaml'
OFFSET_PATH = '../../configs/servo_offset.yaml'

def load_calib(calib_path, servo_names):
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
    with open(calib_path, 'w') as f:
        yaml.dump(calib, f)
    print("Calibration saved.")

def motor_index_to_joint(servo_map):
    return {int(v): k for k, v in servo_map.items()}

def print_menu():
    print("\n--- Servo Configurator ---")
    print("Select mode:")
    print("1: Set servo angles (absolute, no offset)")
    print("2: Set servo limits (min, mid, max)")
    print("q: Quit")

def set_angle_mode(ctrl, channel_to_joint, calib):
    print("\nSet Angle Mode: Enter command pairs '<motor_index> <angle>' (0-180 absolute angles).")
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
            print("Error: Enter pairs in format <motor_index> <angle>.")
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
    print("\nSet Limits Mode: Input quadruples '<motor_index> <min> <mid> <max>'.")
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
            print("Error: Enter quadruples in format <motor_index> <min> <mid> <max>.")
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

def main():
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

if __name__ == '__main__':
    main()
