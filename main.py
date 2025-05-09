# robot_arm/main.py

import time
from kinematics import FullKinematics, Kinematics
from servos import ServoController
from controller import ArmController
from sc_controll import open_gripper, sc_servo, close_gripper, sc_servo_position
from config import L1, L2, gripper, wrist, port, base, shoulder, elbow

kin = Kinematics(L1, L2)
servo_ctrl = ServoController(port)
arm = ArmController(kin, servo_ctrl)

full = FullKinematics(120, 120, 70)  # Przykładowe długości ramion
x, y, z = 310, 0, 0  # Pozycja końcówki
angles = full.full_inverse(x, y, z, elbow_up=True)  # Możliwość ustawienia elbow_up
print(f"Kąty: Phi = {angles[0]}°, Theta1 = {angles[1]}°, Theta2 = {angles[2]}°, Theta3 = {angles[3]}°")