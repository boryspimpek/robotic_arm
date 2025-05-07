# robot_arm/main.py

import time
from kinematics import Kinematics
from servos import ServoController
from controller import ArmController
from sc_controll import open_gripper, sc_servo, close_gripper, sc_servo_position
from config import L1, L2, gripper, wrist, port, base, schoulder, elbow

kin = Kinematics(L1, L2)
servo_ctrl = ServoController(port)
arm = ArmController(kin, servo_ctrl)



# servo_ctrl.ikpy([0.0, 0.58, 0.115])



# GRIPPER
# while True:
#     open_gripper()
#     time.sleep(1)
#     close_gripper()
#     time.sleep(1)

# open_gripper()
# time.sleep(1)

# sc_servo_position(gripper)

# IDS = {
#     base: 1,
#     schoulder: 2,
#     elbow: 3,
#     wrist: 4,
# }

# angles = servo_ctrl.get_all_servo_positions_deg(IDS)
# print(f"Base: {angles[base]:.1f}°")
# print(f"Shoulder: {angles[schoulder]:.1f}°")
# print(f"Elbow: {angles[elbow]:.1f}°")
# print(f"Wrist: {angles[wrist]:.1f}°")

####### SET ANGELS
# home = {
#     base: 90,
#     schoulder: 90,
#     elbow: 90,
#     wrist: 90,
# }
# servo_ctrl.safe_move_to(home)


# servo_ctrl.safe_move_servo(schoulder, 150)


# POINT TO POINT
# arm.point_to_point((142, 0, 0), (200, 0, 0))

# arm.point_to_point((200, 0, 0), (142, 0, 0))

# arm.point_to_point((142, 0, 0), (199, -9, 0))

# arm.point_to_point((199, -9, 0), (142, 0, 0))

# arm.point_to_point((176, 0, 73), (160, 0, 73))


# MOVE TO POINT
a = (0.0, 0.0, 0.6)
# b = (0, 0, 200)
# c = (100, 0, 100)
# d = (123, 75, 0)
# e = (143.266, 0, 138)


arm.move_to_point_dps_ikpy(a, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(b, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(a, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(c, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(a, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(d, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(c, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(d, tempo_dps=30)
# time.sleep(1)
# arm.move_to_point_dps(e, tempo_dps=30)
# time.sleep(1)
