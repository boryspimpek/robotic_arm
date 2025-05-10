# robot_arm/main.py

import time
from kinematics import Kinematics
from kinematics_full import ik_to_servo_angles, solve_ik_3d
from servos import ServoController
from controller import ArmController
from sc_controll import open_gripper, sc_servo, close_gripper, sc_servo_position
from config import L1, L2, gripper, wrist, port, base, shoulder, elbow

kin = Kinematics(L1, L2)
servo_ctrl = ServoController(port)
arm = ArmController(kin, servo_ctrl)


angles, positions = solve_ik_3d(150, 0, 200)
end_angles = ik_to_servo_angles(angles)

for sid, angle in end_angles.items():
    print(f"  Servo {sid}: {angle:.2f}°")

start_angles = servo_ctrl.get_all_servo_positions_deg([base, shoulder, elbow, wrist])

servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
