import os
import sys
import yaml
import time
import re

# Add your module paths
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, 'servo_control'))
sys.path.append(os.path.join(script_dir, 'ik'))

from servo_control.servo_control import ServoController
from ik.ik import LegIK

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

# Config file paths
CONFIG_DIR = os.path.join(script_dir, '../configs')
SERVO_MAP_PATH = os.path.join(CONFIG_DIR, 'servo_map.yaml')
CALIB_PATH = os.path.join(CONFIG_DIR, 'servo_calib.yaml')
OFFSET_PATH = os.path.join(CONFIG_DIR, 'servo_offset.yaml')  # Using your corrected filename
LIMB_LEN_PATH = os.path.join(CONFIG_DIR, 'limb_lengths.yaml')

# Mapping leg numbers to joints
# 0=front_left, 1=front_right, 2=back_left, 3=back_right
JOINT_MAP = {
    0: ['front_left_shoulder', 'front_left_hip', 'front_left_knee'],
    1: ['front_right_shoulder', 'front_right_hip', 'front_right_knee'],
    2: ['back_left_shoulder', 'back_left_hip', 'back_left_knee'],
    3: ['back_right_shoulder', 'back_right_hip', 'back_right_knee'],
}

LEG_NAMES = {0: 'front_left', 1: 'front_right', 2: 'back_left', 3: 'back_right'}

def parse_command(command_string):
    """
    Parse command string in format: "0 x 2 y 4 z -3"
    Returns tuple (leg_number, coordinates_dict) or (None, None) if invalid
    """
    # Remove extra spaces and convert to lowercase
    command_string = command_string.strip().lower()
    
    # Split into parts
    parts = command_string.split()
    
    if len(parts) < 4:  # Need at least: leg_num x val y val z val
        return None, None
    
    # Extract leg number
    try:
        leg_num = int(parts[0])
        if leg_num not in [0, 1, 2, 3]:
            return None, None
    except ValueError:
        return None, None
    
    # Extract coordinates from remaining parts
    coord_parts = parts[1:]  # Everything after leg number
    
    if len(coord_parts) != 3:
        return None, None
    
    coords = {}
    axis = ['x', 'y', 'z']
    for value in range(3):
        coords[axis[value]] = float(coord_parts[value])
    
    # Check if all three axes are present
    if 'x' in coords and 'y' in coords and 'z' in coords:
        return leg_num, {'x': coords['x'], 'y': coords['y'], 'z': coords['z']}
    else:
        return None, None

def main():
    print("DOGBot Leg Movement Test - XY Plane")
    print("=" * 40)
    
    print("Initializing ServoController and LegIK...")
    servo = ServoController(SERVO_MAP_PATH, CALIB_PATH, OFFSET_PATH)
    ik = LegIK(LIMB_LEN_PATH)

    offsets = load_yaml(OFFSET_PATH)

    # Move to stand (home) position immediately
    print("Moving to stand position (home pose)...")
    servo.stand()
    time.sleep(1.0)  # wait for servos

    # Compute & store home foot xyz positions (all at origin)
    ik.compute_home_from_offsets(offsets, JOINT_MAP)
    home_positions = ik.get_all_home_positions()

    print("\nHome positions of each leg (all set to origin):")
    for leg_num, pos in home_positions.items():
        leg_name = LEG_NAMES[leg_num]
        print(f"  Leg {leg_num} ({leg_name}): x={pos['x']:.2f}  y={pos['y']:.2f}  z={pos['z']:.2f}")

    print("\nMovement occurs in XY plane (ground-based walking):")
    print("  X = forward(+)/backward(-)")
    print("  Y = right(+)/left(-)")
    print("  Z = up(+)/down(-)")
    
    print("\n" + "=" * 50)
    print("COMMAND FORMAT:")
    print("  leg_number x_value y_value z_value")
    print("EXAMPLES:")
    print("  0 2 0 0      - Move leg 0 forward 2cm")
    print("  1 0 3 0      - Move leg 1 right 3cm")
    print("  2 -1 -2 0    - Move leg 2 back 1cm, left 2cm")
    print("  3 0 0 2      - Move leg 3 up 2cm")
    print("SPECIAL COMMANDS:")
    print("  home         - Move all legs to home position")
    print("  q            - Quit")
    print("=" * 50)

    # Main command loop
    while True:
        print(f"\nEnter command:")
        command = input("> ").strip().lower()
        
        if command == 'q':
            print("Exiting test.")
            break
        
        if command == 'home':
            print("Moving all legs to home position...")
            try:
                servo.stand()
                print("All legs moved to home position successfully.")
            except Exception as e:
                print(f"Failed to move to home position: {e}")
            continue
        
        if command == '':
            print("Empty command. Please enter a valid command.")
            continue
        
        # Parse the command
        leg_num, target_pos = parse_command(command)
        
        if leg_num is None or target_pos is None:
            print("Invalid command format!")
            print("Use format: leg_number x_value y_value z_value")
            print("Example: 0 2 -1 0 (move leg 0 forward 2cm, left 1cm)")
            continue
        
        leg_name = LEG_NAMES[leg_num]
        print(f"\nCommand parsed successfully:")
        print(f"  Leg: {leg_num} ({leg_name})")
        print(f"  Target: x={target_pos['x']}, y={target_pos['y']}, z={target_pos['z']}")
        
        # Calculate joint angles for the target
        try:
            angles = ik.ik(target_pos['x'], target_pos['y'], target_pos['z'])
            print(f"  IK Solution:")
            for joint_key, angle_val in angles.items():
                print(f"    {joint_key}: {angle_val:.2f}°")
        except Exception as e:
            print(f"IK calculation failed: {e}")
            print("Target position may be unreachable. Try coordinates closer to origin.")
            continue

        # Prepare target angles dictionary for the leg's joints
        target_servo_angles = {}
        joints = JOINT_MAP[leg_num]
        for j_name, joint_key in zip(joints, ['shoulder', 'hip', 'knee']):
            target_servo_angles[j_name] = angles[joint_key]

        # Move leg smoothly to target angles
        print(f"  Moving leg {leg_num} ({leg_name}) to target position...")
        try:
            servo.move_servos_interpolated(target_servo_angles, target_servo_angles, steps_per_second=40)
            print("  ✓ Move completed successfully!")
        except Exception as e:
            print(f"  ✗ Servo movement failed: {e}")

if __name__ == "__main__":
    main()
