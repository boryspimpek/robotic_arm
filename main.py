### robot_arm/main.py

import time
from kinematics import Kinematics
from servos import ServoController
from controller import ArmController
from sc_controll import open_gripper, sc_servo, close_gripper, sc_servo_position
from config import L1, L2, SERVO_GRIPPER_ID, SERVO_WRIST_ID, UART_PORT, SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID

kin = Kinematics(L1, L2)
servo_ctrl = ServoController(UART_PORT)
arm = ArmController(kin, servo_ctrl)


###### GRIPPER
# while True:
#     open_gripper()
#     time.sleep(1)
#     close_gripper()
#     time.sleep(1)

# open_gripper()
# time.sleep(1)

sc_servo_position(SERVO_GRIPPER_ID)

IDS = {
    SERVO_BASE_ID: 1,
    SERVO_SHOULDER_ID: 2,
    SERVO_ELBOW_ID: 3,
    SERVO_WRIST_ID: 4,
}

angles = servo_ctrl.get_all_servo_positions_deg(IDS)
print(f"Base: {angles[SERVO_BASE_ID]:.1f}°")
print(f"Shoulder: {angles[SERVO_SHOULDER_ID]:.1f}°")
print(f"Elbow: {angles[SERVO_ELBOW_ID]:.1f}°")
print(f"Wrist: {angles[SERVO_WRIST_ID]:.1f}°")

# ####### SET ANGELS
home = {
    SERVO_BASE_ID: 90,      
    SERVO_SHOULDER_ID: 90, 
    SERVO_ELBOW_ID: 90,
    SERVO_WRIST_ID: 90,
}
servo_ctrl.move_to(home)


####### POINT TO POINT
# arm.point_to_point((142, 0, 0), (200, 0, 0))

# arm.point_to_point((200, 0, 0), (142, 0, 0))

# arm.point_to_point((142, 0, 0), (199, -9, 0))

# arm.point_to_point((199, -9, 0), (142, 0, 0))

# arm.point_to_point((176, 0, 73), (160, 0, 73))



###### MOVE TO POINT
# a = (10, 0, 20)
# b = (0, 0, 200)
# c = (100, 0, 100)
# d = (123, 75, 0)
# e = (143.266, 0, 138)


# arm.move_to_point_dps(a, tempo_dps=30)
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

