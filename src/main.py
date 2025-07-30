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
offset_path = config('servo_offset.yaml')  # Using your corrected filename
limb_len_path = config('limb_lengths.yaml')

# Joint map using numeric leg identifiers
# 0=front_left, 1=front_right, 2=back_left, 3=back_right
joint_map = {
    0: ['front_left_shoulder', 'front_left_hip', 'front_left_knee'],      # front_left
    1: ['front_right_shoulder', 'front_right_hip', 'front_right_knee'],   # front_right
    2: ['back_left_shoulder',  'back_left_hip',  'back_left_knee'],       # back_left
    3: ['back_right_shoulder', 'back_right_hip', 'back_right_knee']       # back_right
}

leg_names = {0: 'front_left', 1: 'front_right', 2: 'back_left', 3: 'back_right'}
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

# Compute and set home XYZ foot positions (all at origin when standing)
ik.compute_home_from_offsets(offsets, joint_map)
home_positions = ik.home_xyz

# Instantiate gait planner with home positions (XY plane movement)
gait = RectBoxGait(home_positions, step_length=6)

# Initialize current joint angles dictionary (start at stand)
current_angles = {jn: offsets[jn]['stand'] for leg_num in joint_map for jn in joint_map[leg_num]}
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
        for leg_num, pos in phase.items():
            ang = ik.ik(*pos)
            for i, joint_name in enumerate(joint_map[leg_num]):
                target_angles[joint_name] = ang[joints[i]]
        move_angles(target_angles)
        time.sleep(substep_duration)

def continuous_walk(mode='fwd'):
    """Continuous walking until user interrupts"""
    print(f"Starting continuous {mode} walking in XY plane. Press Ctrl+C to stop...")
    step_count = 0
    try:
        while True:
            step_count += 1
            print(f"Step {step_count} - {mode}")
            seq = gait.get_step_sequence(mode)
            execute_sequence(seq)
            time.sleep(0.2)  # Brief pause between steps
    except KeyboardInterrupt:
        print(f"\nContinuous walking stopped after {step_count} steps.")

def sit_pose():
    # Sit maneuver lowers feet by 7cm vertically, shoulders move outward
    print("Moving to sit position immediately...")
    targets = {}
    for leg_num in range(4):
        h = home_positions[leg_num]
        x, y, z = h['x'], h['y'] - 7, h['z']  # Lower in Y direction (body height)
        if leg_num == 0:  # front_left
            y += 3  # shoulders outward in Y direction
        elif leg_num == 1:  # front_right
            y -= 3
        ang = ik.ik(x, y, z)
        for i, joint_name in enumerate(joint_map[leg_num]):
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
        print("\nDOGBot Options (XY plane movement - Legs: 0=FL, 1=FR, 2=BL, 3=BR):")
        print("X = forward(+)/backward(-), Y = right(+)/left(-), Z = up(+)/down(-)")
        print("1 Walk forward (legs 1+2, then 0+3)")
        print("2 Walk backward (legs 1+2, then 0+3)")
        print("3 Turn left (legs 1+2, then 0+3)")
        print("4 Turn right (legs 1+2, then 0+3)")
        print("5 Sit (immediate)")
        print("6 Stand (immediate)")
        print("7 Continuous forward walk")
        print("8 Continuous backward walk")
        print("9 Continuous left turn")
        print("0 Continuous right turn")
        print("q Quit")

    running = True
    while running:
        print_menu()
        choice = input("Choice: ").strip().lower()

        if choice == '1':
            print("Walking forward in XY plane...")
            seq = gait.get_step_sequence('fwd')
            execute_sequence(seq)
        elif choice == '2':
            print("Walking backward in XY plane...")
            seq = gait.get_step_sequence('back')
            execute_sequence(seq)
        elif choice == '3':
            print("Turning left in XY plane...")
            seq = gait.get_step_sequence('turn_left')
            execute_sequence(seq)
        elif choice == '4':
            print("Turning right in XY plane...")
            seq = gait.get_step_sequence('turn_right')
            execute_sequence(seq)
        elif choice == '5':
            sit_pose()
        elif choice == '6':
            stand_pose()
        elif choice == '7':
            continuous_walk('fwd')
        elif choice == '8':
            continuous_walk('back')
        elif choice == '9':
            continuous_walk('turn_left')
        elif choice == '0':
            continuous_walk('turn_right')
        elif choice == 'q':
            running = False
        else:
            print("Invalid choice. Please enter 1-9, 0, or q.")

if __name__ == "__main__":
    main()
