import time
import os
import sys
import yaml

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path += [os.path.join(script_dir, p) for p in ('servo_control', 'ik', 'gait')]

from servo_control.servo_control import ServoController
from ik.ik import LegIK
from gait.gait import RectBoxGait

# Config files
def config(name): return os.path.join(script_dir, '..', 'configs', name)

servo_map_path = config('servo_map.yaml')
calib_path = config('servo_calib.yaml')
offset_path = config('servo_offset.yaml')
limb_len_path = config('limb_lengths.yaml')

joint_map = {
    'front_left':  ['front_left_shoulder', 'front_left_hip', 'front_left_knee'],
    'front_right': ['front_right_shoulder', 'front_right_hip', 'front_right_knee'],
    'back_left':   ['back_left_shoulder',  'back_left_hip',  'back_left_knee'],
    'back_right':  ['back_right_shoulder', 'back_right_hip', 'back_right_knee']
}

legs = list(joint_map.keys())
joints = ['shoulder', 'hip', 'knee']

# Init hardware and software modules
servo = ServoController(servo_map_path, calib_path, offset_path)
ik = LegIK(limb_len_path)

# Load offsets for stand angles
with open(offset_path, 'r') as f:
    offsets = yaml.safe_load(f)

# Move immediately to stand position on start (no interpolation)
print("Moving to stand position...")
servo.stand()

# Compute and set home XYZ foot positions from stand angles
ik.compute_home_from_offsets(offsets, joint_map)
home_positions = ik.home_xyz

# Instantiate gait planner with home positions
gait = RectBoxGait(home_positions, step_length=6, step_depth=4)

# Initialize current joint angles dictionary (start at stand)
current_angles = {jn: offsets[jn]['stand'] for leg in joint_map for jn in joint_map[leg]}
interp_speed = 50  # interpolation steps per second
substep_duration = 0.5  # seconds (total step 2s / 4 substeps)

def move_angles(target_angles):
    # Move only joints in target_angles smoothly, update current angles
    currents = {jn: current_angles[jn] for jn in target_angles}
    servo.move_servos_interpolated(target_angles, currents, steps_per_second=interp_speed)
    current_angles.update(target_angles)

def execute_sequence(sequence):
    # Execute a full sequence (4 substeps), alternating leg groups movement
    for phase in sequence:
        target_angles = {}
        # Solve ik and build joint commands for legs moving in this phase
        for leg, pos in phase.items():
            ang = ik.ik(*pos)
            for i, joint_name in enumerate(joint_map[leg]):
                target_angles[joint_name] = ang[joints[i]]
        move_angles(target_angles)
        time.sleep(substep_duration)

def sit_pose():
    # Sit maneuver lowers feet by 7cm vertically, shoulders move outward
    print("Moving to sit position immediately...")
    targets = {}
    for leg in legs:
        h = home_positions[leg]
        x, y, z = h['x'], h['y'] + 7, h['z']
        if leg == 'front_left':
            z += 3  # shoulders outward
        elif leg == 'front_right':
            z -= 3
        ang = ik.ik(x, y, z)
        for i, joint_name in enumerate(joint_map[leg]):
            targets[joint_name] = ang[joints[i]]
    # Move servos immediately (no interpolation)
    servo.move_servos_interpolated(targets, targets, steps_per_second=interp_speed)
    current_angles.update(targets)

def stand_pose():
    # Immediately move servos to stand offsets
    print("Setting to stand position immediately...")
    targets = {jn: offsets[jn]['stand'] for jn in current_angles}
    servo.move_servos_interpolated(targets, targets, steps_per_second=interp_speed)
    current_angles.update(targets)

def main():
    def print_menu():
        print("\nDOGBot Options:")
        print("1 Walk forward (FR+BL, then FL+BR)")
        print("2 Walk backward (FR+BL, then FL+BR)")
        print("3 Turn left (FR+BL, then FL+BR)")
        print("4 Turn right (FR+BL, then FL+BR)")
        print("5 Sit (immediate)")
        print("6 Stand (immediate)")
        print("q Quit")

    running = True
    while running:
        print_menu()
        choice = input("Choice: ").strip().lower()

        if choice == '1':
            seq = gait.get_step_sequence('fwd')
            execute_sequence(seq)
        elif choice == '2':
            seq = gait.get_step_sequence('back')
            execute_sequence(seq)
        elif choice == '3':
            seq = gait.get_step_sequence('turn_left')
            execute_sequence(seq)
        elif choice == '4':
            seq = gait.get_step_sequence('turn_right')
            execute_sequence(seq)
        elif choice == '5':
            sit_pose()
        elif choice == '6':
            stand_pose()
        elif choice == 'q':
            running = False
        else:
            print("Invalid choice. Please enter 1-6 or q.")

if __name__ == "__main__":
    main()
