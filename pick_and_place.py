import time
from config import L1, L2, L3, port_bus
from controller import ArmController
from kinematics import Kinematics
from kinematics_full import FullKinematics
from sc_controll import close_gripper, open_gripper
from servos import ServoController


arm = ArmController(kinematics=Kinematics(L1, L2), servo_ctrl=ServoController(port_bus), fullkin=FullKinematics(L1, L2, L3))




def pickup_place(pickup, place, tempo_dps=60, lift_height=40, cost_mode="vertical_down"):
    x1, y1, z1 = pickup
    x2, y2, z2 = place
    above_pickup = (x1, y1, z1 + lift_height)
    above_place = (x2, y2, z2 + lift_height)

    def move_to(position, tempo_dps=tempo_dps, cost_mode=cost_mode):
        arm.move_to_point_full(*position, tempo_dps, cost_mode)
        time.sleep(0.5)

    def grip(action):
        if action == "open":
            open_gripper()
        elif action == "close":
            close_gripper()
        time.sleep(0.5)

    # Sekwencja pick & place
    move_to(above_pickup)
    grip("open")
    move_to(pickup, tempo_dps=30)
    grip("close")
    move_to(above_pickup)
    move_to(above_place)
    move_to(place, tempo_dps=30)
    grip("open")
    move_to(above_place)
    grip("close")
    arm.homepos()

pickup_place((150, 0, -110), (150, -100, -110), lift_height=30)
time.sleep(1)
pickup_place((150, -100, -110), (150, 0, -110), lift_height=30)
