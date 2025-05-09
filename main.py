# robot_arm/main.py

import time
from kinematics import Kinematics
from servos import ServoController
from controller import ArmController
from sc_controll import open_gripper, sc_servo, close_gripper, sc_servo_position
from config import L1, L2, gripper, wrist, port, base, shoulder, elbow

kin = Kinematics(L1, L2)
servo_ctrl = ServoController(port)
arm = ArmController(kin, servo_ctrl)



a = (0.0, 0.142, 0.087)

arm.move_to_point_dps_ikpy(a, tempo_dps=30)
