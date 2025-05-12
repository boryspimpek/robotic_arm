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
b = "flat_end_effector"
c = "vertical_end_effector"
d = "inverted_vertical_end_effector"
x = 120
y = 0
z = 200
tempo_dps=60

while True:
    arm.move_to_point_ik_full(x, y, z, tempo_dps, cost_mode=b)
    time.sleep(2)
    arm.move_to_point_ik_full(x, y, z, tempo_dps, cost_mode=c)
    time.sleep(2)
    # arm.move_to_point_ik_full(x, y, z, tempo_dps, cost_mode=c)
    # time.sleep(2)
    # arm.move_to_point_ik_full(x, y, z, tempo_dps, cost_mode=d)


# point = (100, 0, 0)
# arm.move_to_point_dps(point, tempo_dps=30)


