# robot_arm/main.py

import time

from matplotlib import pyplot as plt
from kinematics import Kinematics
from kinematics_full import FullKinematics
from servos import ServoController
from controller import ArmController
from sc_controll import open_gripper, sc_servo, close_gripper, sc_servo_position
from config import L1, L2, L3, gripper, wrist, port_bus, base, shoulder, elbow

kin = Kinematics(L1, L2)
fullkin = FullKinematics(L1, L2, L3)
servo_ctrl = ServoController(port_bus)
arm = ArmController(kinematics=Kinematics(L1, L2), servo_ctrl=ServoController(port_bus), fullkin=FullKinematics(L1, L2, L3))


# angles = servo_ctrl.get_positions([base, shoulder, elbow, wrist])
# print(f"[INFO] Current angles: {angles}")

# arm.move_to_point_full(300, 0, 50, tempo_dps=60, cost_mode="flat")
# time.sleep(3)
# angles = servo_ctrl.get_positions([shoulder, elbow, wrist])
# print(f"[INFO] Current angles: {angles}")

# lenghts = [L1, L2, L3]

# fullkin.forward_ik_full(angles, lenghts)


point=(150, 100, 100)
arm.move_to_point_simple(point)

# servo_ctrl.move_servo(1, 90)
# servo_ctrl.move_servo(2, 90)
# servo_ctrl.move_servo(3, 90)
# servo_ctrl.move_servo(4, 0)

a = "min_angle_sum"
b = "flat"
c = "vertical_up"
d = "vertical_down"
e = "standard"
tempo_dps=60


# def move_arm_sequentially(start_x, end_x, y, z, tempo_dps, cost_mode):

#     step = 5 if start_x <= end_x else -5
#     for x in range(start_x, end_x + step, step):
#         arm.move_to_point_ik_full(x, y, z, tempo_dps, cost_mode)

# # Call the function
# while True:
#     a = "min_angle_sum"
#     b = "flat"
#     c = "vertical_up"
#     d = "vertical_down"
#     e = "standard"
#     tempo_dps=160
#     cost_mode = d

#     move_arm_sequentially(240, 70, 0, -50, tempo_dps, cost_mode)
#     move_arm_sequentially(70, 240, 0, -50, tempo_dps, cost_mode)

# arm.move_to_point_ik_full(120, 0, 100, tempo_dps=60, cost_mode="flat")


# ik_angles, positions = fullkin.solve_ik_2d(x, y)
# plt.show()

# ik_angles, positions = fullkin.solve_ik_3d(310, 0, 0, cost_mode=e)
# angles = fullkin.ik_3d_to_servo_angles(ik_angles)
# print(f"[INFO] Angles: {angles}")

# plt.show()
