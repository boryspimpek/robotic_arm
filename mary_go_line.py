import time
from config import L1, L2, L3, port_bus
from controller import ArmController
from kinematics import Kinematics
from kinematics_full import FullKinematics
from sc_controll import close_gripper, open_gripper
from servos import ServoController

SLEEP_TIME = 0.3  # Set sleep duration here

# Initialize arm controller
arm = ArmController(
    kinematics=Kinematics(L1, L2),
    servo_ctrl=ServoController(port_bus),
    fullkin=FullKinematics(L1, L2, L3)
)

# Define points
pointA = (125, -160, -110)
pointB = (125, -160, -93)
pointC = (125, -160, -78)
pointD = (125, -160, -63)
pointE = (125, -160, -48)

point1 = (125, 0, -113)
point2 = (125, 0, -98)
point3 = (125, 0, -83)
point4 = (125, 0, -68)
point5 = (125, 0, -53)

# Movement functions
def move_to(position, tempo_dps=60, cost_mode="vertical_down"):
    arm.move_to_point_full(*position, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)

def perform_pick(pick_point, above_point, tempo_dps=30, cost_mode="vertical_down"):
    # open_gripper()
    # time.sleep(SLEEP_TIME)
    arm.move_to_point_full(*pick_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)
    close_gripper()
    time.sleep(SLEEP_TIME)
    arm.move_to_point_full(*above_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)

def perform_place(place_point, above_point, tempo_dps=30, cost_mode="vertical_down"):
    arm.move_to_point_full(*place_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)
    open_gripper()
    time.sleep(SLEEP_TIME)
    arm.move_to_point_full(*above_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)
    # close_gripper()

def pick_drop(pickup_point, place_point, lift_height=40):
    x1, y1, z1 = pickup_point
    x2, y2, z2 = place_point

    above_pickup = (x1, y1, z1 + lift_height)
    above_place = (x2, y2, z2 + lift_height)

    move_to(above_pickup)
    perform_pick(pickup_point, above_pickup)
    move_to(above_place)
    perform_place(place_point, above_place)
    # arm.homepos()

# Execute pick and place sequence
open_gripper()  # Ensure gripper is open before starting
pick_drop(pointE, point1)
pick_drop(pointD, point2)
pick_drop(pointC, point3)
pick_drop(pointB, point4)
pick_drop(pointA, point5)

arm.homepos()  # Return to home position after operations

open_gripper()  # Ensure gripper is open before starting next sequence
pick_drop(point5, pointA)
pick_drop(point4, pointB)
pick_drop(point3, pointC)       
pick_drop(point2, pointD)
pick_drop(point1, pointE)

arm.homepos()  # Return to home position after operations