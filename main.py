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



positions = servo_ctrl.get_positions([1, 2, 3, 4])
angles = {sid: int(round(angle)) for sid, angle in positions.items()}

# Oblicz FK
x2, y2, z2, x3, y3, z3 = fullkin.forward_ik_full(angles)  # Upewnij się, że masz instancję `kinematics`


