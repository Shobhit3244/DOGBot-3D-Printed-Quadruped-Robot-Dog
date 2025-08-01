import time
import os
import sys
import yaml

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path += [os.path.join(script_dir, p) for p in ('servo_control', 'ik', 'gait')]

from servo_control.servo_control import ServoController
from ik.ik import LegIK
from gait.gait import TrotBoxGait

# Config files
def config(name): return os.path.join(script_dir, '..', 'configs', name)

servo_map_path = config('servo_map.yaml')
calib_path = config('servo_calib.yaml')
offset_path = config('servo_offset.yaml')
limb_len_path = config('limb_lengths.yaml')

# Joint map using numeric leg identifiers
# 0=front_left, 1=front_right, 2=back_left, 3=back_right
joint_map = {
    0: ['front_left_shoulder', 'front_left_hip', 'front_left_knee'],     # front_left
    1: ['front_right_shoulder', 'front_right_hip', 'front_right_knee'],  # front_right  
    2: ['back_left_shoulder', 'back_left_hip', 'back_left_knee'],        # back_left
    3: ['back_right_shoulder', 'back_right_hip', 'back_right_knee']      # back_right
}

leg_names = {0: 'front_left', 1: 'front_right', 2: 'back_left', 3: 'back_right'}
joints = ['shoulder', 'hip', 'knee']

# Initialize hardware and software modules
servo = ServoController(servo_map_path, calib_path, offset_path)
ik = LegIK(limb_len_path)

# Load offsets for stand angles
with open(offset_path, 'r') as f:
    offsets = yaml.safe_load(f)

def apply_servo_offsets(leg_num, joint_type, angle):
    """
    Apply servo offsets as requested:
    - All 4 hip servos: +30 degrees offset
    - Front two knees: +4 degrees offset  
    - Back two knees: +18 degrees offset
    - Shoulders: No additional offset (already at stand angles)
    """
    if joint_type == 'hip':
        return angle + 30        # All hips +30°
    elif joint_type == 'knee':
        if leg_num in [0, 1]:    # Front legs
            return angle + 4     # Front knees +4°
        else:                    # Back legs (2, 3)
            return angle + 18    # Back knees +18°
    else:  # shoulder
        return angle             # No additional offset for shoulders

def set_shoulders_to_stand():
    """Set all shoulders to their stand positions and keep them there"""
    print("Setting shoulders to stand positions (FIXED)...")
    for leg_num in range(4):
        shoulder_joint = joint_map[leg_num][0]
        stand_angle = offsets[shoulder_joint]['stand']
        servo.set_servo_absolute_no_offset(shoulder_joint, stand_angle)
        print(f"  {shoulder_joint}: {stand_angle}°")

# Move to stand position on start
print("=" * 60)
print("DOGBOT 2-DOF TROT SYSTEM INITIALIZING")  
print("=" * 60)
print("🔒 Shoulders: FIXED at stand angles (never move)")
print("⚙️ Servo offsets: Hip +30°, Front knees +4°, Back knees +18°")
print("🚶 Gait: Trot with diagonal pairs only")

print("\nMoving to stand position...")
servo.stand()
time.sleep(1)

# Set shoulders to stand and keep them fixed
set_shoulders_to_stand()

# Compute and set home XY foot positions using geometry
ik.compute_home_from_offsets(offsets, joint_map)
home_positions = ik.home_xyz

# Instantiate trot gait planner with home positions (2D movement)
gait = TrotBoxGait(home_positions, step_length=4.0, lift_height=4.0)

# Initialize current joint angles dictionary (start at stand)
current_angles = {jn: offsets[jn]['stand'] for leg_num in joint_map for jn in joint_map[leg_num]}

interp_speed = 50  # interpolation steps per second

def move_legs_to_positions(phase_positions):
    """
    Move legs to target positions with 2-DOF IK and servo offsets applied
    phase_positions: {leg_num: (x, y)}
    """
    target_angles = {}
    
    for leg_num, (x, y) in phase_positions.items():
        try:
            # Calculate IK for this leg (2D coordinates only)
            angles = ik.ik(x, y, leg_num)
            
            # Apply servo offsets and build target angles
            for i, joint_type in enumerate(['shoulder', 'hip', 'knee']):
                joint_name = joint_map[leg_num][i]
                base_angle = angles[joint_type]
                
                if joint_type == 'shoulder':
                    # Shoulders stay fixed - don't move them
                    continue
                elif joint_type in ['hip', 'knee']:
                    # Apply offsets for hip and knee joints
                    final_angle = apply_servo_offsets(leg_num, joint_type, base_angle)
                    target_angles[joint_name] = final_angle
                    
        except Exception as e:
            print(f"IK failed for leg {leg_num} at position ({x:.1f}, {y:.1f}): {e}")
            continue
    
    if target_angles:
        # Move only hip and knee joints (shoulders stay fixed)  
        currents = {jn: current_angles[jn] for jn in target_angles}
        servo.move_servos_interpolated(target_angles, currents, steps_per_second=interp_speed)
        current_angles.update(target_angles)

def execute_trot_sequence(sequence):
    """Execute a full trot sequence (8 phases)"""
    for phase_num, phase_positions in enumerate(sequence):
        print(f"  Phase {phase_num + 1}/8: Moving legs...")
        move_legs_to_positions(phase_positions)
        time.sleep(0.8)  # 0.8 seconds per phase

def single_trot_step(direction='forward'):
    """Execute one complete trot step cycle"""
    print(f"Executing single trot step {direction}...")
    sequence = gait.get_step_sequence(direction)
    execute_trot_sequence(sequence)
    print(f"Trot step {direction} completed!")

def continuous_trot(direction='forward'):
    """Continuous trot walking until interrupted"""
    print(f"Starting continuous trot {direction}. Press Ctrl+C to stop...")
    step_count = 0
    
    try:
        while True:
            step_count += 1
            print(f"\nTrot Step {step_count} ({direction})")
            sequence = gait.get_step_sequence(direction)
            execute_trot_sequence(sequence)
            time.sleep(0.1)  # Brief pause between steps
            
    except KeyboardInterrupt:
        print(f"\nContinuous trot stopped after {step_count} steps.")

def return_to_stand():
    """Return all legs to stand position"""
    print("Returning to stand position...")
    servo.stand()
    set_shoulders_to_stand()  # Ensure shoulders stay fixed
    time.sleep(1)
    
    # Update current angles to match stand position
    for jn in current_angles:
        current_angles[jn] = offsets[jn]['stand']
    
    print("Returned to stand position!")

def main():
    def print_menu():
        print("\n" + "=" * 60)
        print("DOGBOT TROT GAIT CONTROLLER - FIXED SHOULDERS")
        print("=" * 60)
        print("🔒 SHOULDERS: Fixed at stand angles (NEVER move during walking)")
        print("🚶 TROT GAIT: Only diagonal pairs swing (FL+BR, then FR+BL)")
        print("📦 BOX STEP: lift 2cm → advance 2cm → lower 2cm → return 2cm")
        print("⚙️ OFFSETS: Hips +30°, Front knees +4°, Back knees +18°")
        print()
        print("1. Single trot step forward")
        print("2. Single trot step backward") 
        print("3. Continuous trot forward")
        print("4. Continuous trot backward")
        print("5. Stand position")
        print("s. Set shoulders to stand position")
        print("q. Quit")

    running = True
    while running:
        print_menu()
        choice = input("\nChoice: ").strip().lower()

        if choice == '1':
            single_trot_step('forward')
            
        elif choice == '2':
            single_trot_step('backward')
            
        elif choice == '3':
            continuous_trot('forward')
            
        elif choice == '4':
            continuous_trot('backward')
            
        elif choice == '5':
            return_to_stand()
            
        elif choice == 's':
            set_shoulders_to_stand()
            
        elif choice == 'q':
            print("Shutting down...")
            return_to_stand()  # Return to safe position before exit
            running = False
            
        else:
            print("Invalid choice. Please enter 1-5, s, or q.")

if __name__ == "__main__":
    main()