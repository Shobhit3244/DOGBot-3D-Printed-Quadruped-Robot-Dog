import time
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, 'servo_control'))
sys.path.append(os.path.join(script_dir, 'ik'))
sys.path.append(os.path.join(script_dir, 'gait'))

from servo_control.servo_control import ServoController
from ik.ik import LegIK
from gait.gait import GaitPlanner

def main():
    servo_map_path = os.path.join(script_dir, '../configs/servo_map.yaml')
    calib_path = os.path.join(script_dir, '../configs/servo_calib.yaml')
    limb_len_path = os.path.join(script_dir, '../configs/limb_lengths.yaml')

    servo = ServoController(servo_map_path, calib_path)
    ik = LegIK(limb_len_path)
    gait = GaitPlanner()

    step_duration = 1.0
    hop_repeats = 3

    legs = ['front_left', 'front_right', 'back_left', 'back_right']
    joints = ['shoulder', 'hip', 'knee']
    joint_map = {
        'front_left':  ['front_left_shoulder', 'front_left_hip', 'front_left_knee'],
        'front_right': ['front_right_shoulder', 'front_right_hip', 'front_right_knee'],
        'back_left':   ['back_left_shoulder', 'back_left_hip', 'back_left_knee'],
        'back_right':  ['back_right_shoulder', 'back_right_hip', 'back_right_knee'],
    }

    print("Starting robot gait cycle loop...")

    try:
        while True:
            # Walk forward - 10 steps
            for step in range(10):
                t = (step / 10.0) * step_duration
                for leg in legs:
                    phase_offset = 0.5 if leg in ['front_right', 'back_left'] else 0.0
                    x, y, z = gait.walk_cycle(t + phase_offset, step_duration)
                    angles = ik.solve(x, y, z)
                    for i, joint in enumerate(joints):
                        servo.set_servo_angle(joint_map[leg][i], angles[joint])
                time.sleep(step_duration / 10)

            # Turn right (10 steps)
            for step in range(10):
                t = (step / 10.0) * step_duration
                for leg in legs:
                    x, y, z = gait.turn_cycle(t, step_duration, radius=6.0, side='right')
                    angles = ik.solve(x, y, z)
                    for i, joint in enumerate(joints):
                        servo.set_servo_angle(joint_map[leg][i], angles[joint])
                time.sleep(step_duration / 10)

            # Walk forward again (10 steps)
            for step in range(10):
                t = (step / 10.0) * step_duration
                for leg in legs:
                    phase_offset = 0.5 if leg in ['front_right', 'back_left'] else 0.0
                    x, y, z = gait.walk_cycle(t + phase_offset, step_duration)
                    angles = ik.solve(x, y, z)
                    for i, joint in enumerate(joints):
                        servo.set_servo_angle(joint_map[leg][i], angles[joint])
                time.sleep(step_duration / 10)

            # Turn left (10 steps)
            for step in range(10):
                t = (step / 10.0) * step_duration
                for leg in legs:
                    x, y, z = gait.turn_cycle(t, step_duration, radius=6.0, side='left')
                    angles = ik.solve(x, y, z)
                    for i, joint in enumerate(joints):
                        servo.set_servo_angle(joint_map[leg][i], angles[joint])
                time.sleep(step_duration / 10)

            # Hop 3 times synchronously
            for hop in range(hop_repeats):
                for step in range(10):
                    t = (step / 10.0) * step_duration
                    x, y, z = gait.hop_pattern(t, step_duration)
                    for leg in legs:
                        angles = ik.solve(x, y, z)
                        for i, joint in enumerate(joints):
                            servo.set_servo_angle(joint_map[leg][i], angles[joint])
                    time.sleep(step_duration / 10)

            print("Pausing for 5 minutes...")
            time.sleep(300)  # 5 minutes pause

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting...")

if __name__ == "__main__":
    main()
