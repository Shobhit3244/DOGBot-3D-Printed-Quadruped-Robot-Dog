import os
import yaml
import re
import time
from servo_control import ServoController  # Import your existing servo_control.py as a module

MAP_PATH = '../../configs/servo_map.yaml'
CALIB_PATH = '../../configs/servo_calib.yaml'

def load_calib(calib_path, servo_names):
    if os.path.exists(calib_path):
        with open(calib_path, 'r') as f:
            calib = yaml.safe_load(f)
    else:
        # Defaults: min=0, mid=90, max=180 for all servos
        calib = {name: {'min': 0, 'mid': 90, 'max': 180} for name in servo_names}
    return calib

def save_calib(calib_path, calib):
    with open(calib_path, 'w') as f:
        yaml.dump(calib, f)
    print("Calibration saved.")

def motor_index_to_joint(servo_map):
    # Map PCA9685 channel indices to joint names
    channel_to_joint = {int(v): k for k, v in servo_map.items()}
    return channel_to_joint

def print_menu():
    print("\n--- Servo Configurator ---")
    print("Select mode:")
    print("1: Set servo angles")
    print("2: Set servo limits (min, mid, max)")
    print("q: Quit")

def set_angle_mode(ctrl, channel_to_joint, calib):
    print("\nSet Angle Mode: Input pairs of '<motor_index> <angle>' separated by spaces.")
    print("Example: 0 90 3 45 5 135")
    print("Commands: b = back to menu, q = quit")
    while True:
        line = input("Set angles> ").strip()
        if line.lower() == 'q':
            print("Quitting...")
            exit(0)
        if line.lower() == 'b':
            print("Returning to main menu...")
            break
        if not line:
            continue

        tokens = line.split()
        if len(tokens) % 2 != 0:
            print("Error: Input must be pairs of '<motor_index> <angle>'")
            continue
        try:
            for i in range(0, len(tokens), 2):
                idx = int(tokens[i])
                angle = int(tokens[i+1])
                if idx not in channel_to_joint:
                    print(f"Warning: Motor index {idx} not found, skipping")
                    continue
                joint = channel_to_joint[idx]
                min_a = calib[joint]['min']
                max_a = calib[joint]['max']
                mid_a = calib[joint]['mid']
                # Convert absolute angle to offset from mid
                angle_offset = angle - mid_a
                # Clamp absolute angle before offset? Actually clamp offset by min/max:
                abs_angle = angle
                if abs_angle < min_a or abs_angle > max_a:
                    print(f"Warning: {joint} angle {abs_angle} outside limits [{min_a},{max_a}]. Clamping.")
                    abs_angle = max(min_a, min(max_a, abs_angle))
                    angle_offset = abs_angle - mid_a
                print(f"Setting servo '{joint}' (channel {idx}) to angle {abs_angle} (offset {angle_offset})")
                ctrl.set_servo_angle(joint, angle_offset)
                time.sleep(0.05)  # small delay for servo to move
        except Exception as e:
            print(f"Invalid input or error: {e}")
            continue

def set_limits_mode(calib, channel_to_joint):
    print("\nSet Limits Mode: Input quadruples '<motor_index> <min> <mid> <max>' separated by spaces.")
    print("Example: 0 10 90 170 3 15 87 160")
    print("Commands: b = back to menu, q = quit")
    while True:
        line = input("Set limits> ").strip()
        if line.lower() == 'q':
            print("Quitting...")
            exit(0)
        if line.lower() == 'b':
            print("Returning to main menu...")
            break
        if not line:
            continue

        tokens = line.split()
        if len(tokens) % 4 != 0:
            print("Error: Input must be quadruples '<motor_index> <min> <mid> <max>'")
            continue
        try:
            for i in range(0, len(tokens), 4):
                idx = int(tokens[i])
                min_angle = int(tokens[i+1])
                mid_angle = int(tokens[i+2])
                max_angle = int(tokens[i+3])

                if idx not in channel_to_joint:
                    print(f"Warning: Motor index {idx} not found, skipping")
                    continue
                joint = channel_to_joint[idx]
                if not (0 <= min_angle <= mid_angle <= max_angle <= 180):
                    print(f"Warning: Invalid angles for {joint}. Must satisfy 0 <= min <= mid <= max <= 180.")
                    continue
                calib[joint] = {'min': min_angle, 'mid': mid_angle, 'max': max_angle}
                print(f"Set limits for '{joint}': min={min_angle}, mid={mid_angle}, max={max_angle}")
        except Exception as e:
            print(f"Invalid input or error: {e}")
            continue

def main():
    # Load servo map and calibration
    if not os.path.exists(MAP_PATH):
        print(f"Error: Servo map file not found: {MAP_PATH}")
        return
    with open(MAP_PATH, 'r') as f:
        servo_map = yaml.safe_load(f)

    servo_names = list(servo_map.keys())
    calib = load_calib(CALIB_PATH, servo_names)
    channel_to_joint = motor_index_to_joint(servo_map)

    ctrl = ServoController(MAP_PATH, CALIB_PATH)

    while True:
        print_menu()
        choice = input("Select mode> ").strip().lower()
        if choice == '1':
            set_angle_mode(ctrl, channel_to_joint, calib)
        elif choice == '2':
            set_limits_mode(calib, channel_to_joint)
        elif choice == 'q':
            print("Exiting configurator.")
            break
        else:
            print("Invalid choice. Please select 1, 2, or q.")

    # Save calibration on exit
    save_calib(CALIB_PATH, calib)

if __name__ == '__main__':
    main()
