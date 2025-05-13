# robot_arm/main.py

import time

from matplotlib import pyplot as plt
from kinematics import Kinematics
from kinematics_full import FullKinematics
from servos import ServoController
from controller import ArmController
from sc_controll import open_gripper, sc_servo, close_gripper, sc_servo_position
from config import L1, L2, L3, gripper, wrist, port, base, shoulder, elbow

kin = Kinematics(L1, L2)
fullkin = FullKinematics(L1, L2, L3)
servo_ctrl = ServoController(port)
arm = ArmController(kin, servo_ctrl)


a = "min_angle_sum"
b = "flat"
c = "vertical_up"
d = "vertical_down"
e = "standard"
tempo_dps=60


def move_arm_sequentially(start_x, end_x, y, z, tempo_dps, cost_mode):

    step = 5 if start_x <= end_x else -5
    for x in range(start_x, end_x + step, step):
        arm.move_to_point_ik_full(x, y, z, tempo_dps, cost_mode)

# Call the function
while True:
    a = "min_angle_sum"
    b = "flat"
    c = "vertical_up"
    d = "vertical_down"
    e = "standard"
    tempo_dps=160
    cost_mode = d

    move_arm_sequentially(240, 70, 0, -50, tempo_dps, cost_mode)
    move_arm_sequentially(70, 240, 0, -50, tempo_dps, cost_mode)

# arm.move_to_point_ik_full(x, 0, 0, tempo_dps, cost_mode="standard")


# ik_angles, angles_deg, positions = fullkin.solve_ik_2d(x, y)
# plt.show()

# ik_angles, angles_deg, positions = fullkin.solve_ik_3d(x, y, z, cost_mode=d)
# print(f"[INFO] Targets: {angles_deg}")

# plt.show()
