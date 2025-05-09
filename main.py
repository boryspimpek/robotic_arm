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



# a = (10, 0, 10)

# arm.move_to_point_dps(a, tempo_dps=30)

# ids = [base, shoulder, elbow, wrist, gripper]
# servo_ctrl.get_all_servo_positions_deg(ids)
# print(servo_ctrl.get_all_servo_positions_deg(ids))

# current_angles = servo_ctrl.get_all_servo_positions_deg([1, 2, 3, 4])
# print(current_angles)    

# start_angles = current_angles
# end_angles = {
#     1: 90,
#     2: 6,
#     3: 180,
#     4: 22
# }

# print(kin.forward(0, 180))

# servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)

arm.homepos()