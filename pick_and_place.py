import time
from config import L1, L2, L3, port_bus
from controller import ArmController
from kinematics import Kinematics
from kinematics_full import FullKinematics
from sc_controll import close_gripper, open_gripper
from servos import ServoController

arm = ArmController(
    kinematics=Kinematics(L1, L2),
    servo_ctrl=ServoController(port_bus),
    fullkin=FullKinematics(L1, L2, L3)
)

def pick_drop(pickup_point, place_point, lift_height=40):
    x1, y1, z1 = pickup_point
    x2, y2, z2 = place_point
    above_pickup = (x1, y1, z1 + lift_height)
    above_place = (x2, y2, z2 + lift_height)

    move_to(above_pickup)
    perform_pick(pickup_point, above_pickup)
    move_to(above_place)
    perform_place(place_point, above_place)
    arm.homepos()

def move_to(position, tempo_dps=60, cost_mode="vertical_down"):
    arm.move_to_point_full(*position, tempo_dps, cost_mode)
    time.sleep(0.5)

def perform_pick(pick_point, above_point, tempo_dps=30, cost_mode="vertical_down"):
    open_gripper()
    time.sleep(0.5)
    arm.move_to_point_full(*pick_point, tempo_dps, cost_mode)
    time.sleep(0.5)
    close_gripper()
    time.sleep(0.5)
    arm.move_to_point_full(*above_point, tempo_dps, cost_mode)
    time.sleep(0.5)

def perform_place(place_point, above_point, tempo_dps=30, cost_mode="vertical_down"):
    arm.move_to_point_full(*place_point, tempo_dps, cost_mode)
    time.sleep(0.5)
    open_gripper()
    time.sleep(0.5)
    arm.move_to_point_full(*above_point, tempo_dps, cost_mode)
    time.sleep(0.5)
    close_gripper()

