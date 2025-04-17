### robot_arm/main.py

import time
from kinematics import Kinematics
from servos import ServoController
from controller import ArmController
from config import L1, L2, UART_PORT, SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID

kin = Kinematics(L1, L2)
servo_ctrl = ServoController(UART_PORT)
arm = ArmController(kin, servo_ctrl)



### SET ANGELS
# angles = {
#     SERVO_BASE_ID: 90,      
#     SERVO_SHOULDER_ID: 90, 
#     SERVO_ELBOW_ID: 90      
# }
# servo_ctrl.move_to(angles)


### ALONG AXIS
arm.point_to_point((143.266, 0, 47.619), (191.608, 0, 47.619))

arm.point_to_point((191.608, 0, 47.619), (191.608, 0, -35.036))

arm.point_to_point((191.608, 0, -35.036), (143.266, 0, -35.036))

arm.point_to_point((143.266, 0, -35.036), (143.266, 0, 47.619))


