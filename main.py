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


angles, positions = fullkin.solve_ik_3d(150, 50, 50)
end_angles = fullkin.ik_3d_to_servo_angles(angles)
new_angles = {sid: int(round(angle)) for sid, angle in end_angles.items()}
# for sid, angle in end_angles.items():
#     print(f"  Servo {sid}: {angle:.2f}°")
# plt.show()
start_angles = servo_ctrl.get_all_servo_positions_deg([base, shoulder, elbow, wrist])
# servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)


# angles, positions = fullkin.solve_ik_2d(100, 0)
# end_angles = fullkin.ik_2d_to_servo_angles(angles)
# for sid, angle in end_angles.items():
#     print(f"  Servo {sid}: {angle:.2f}°")
# # plt.show()
# start_angles = servo_ctrl.get_all_servo_positions_deg([shoulder, elbow, wrist])
# servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
