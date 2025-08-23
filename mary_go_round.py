import time
from config import L1, L2, L3, port_bus
from controller import ArmController
from kinematics import Kinematics
from kinematics_full import FullKinematics
from sc_controll import close_gripper, open_gripper
from servos import ServoController

SLEEP_TIME = 0.2  # Set sleep duration here

# Initialize arm controller
arm = ArmController(
    kinematics=Kinematics(L1, L2),
    servo_ctrl=ServoController(port_bus),
    fullkin=FullKinematics(L1, L2, L3)
)

# Define points
point1 = (145, 65, -110)
point2 = (225, 0, -105)
point3 = (145, -65, -110)
empty_slot = (100, 0, -110)

# Movement functions
def move_to(position, tempo_dps=90, cost_mode="vertical_down"):
    arm.move_to_point_full(*position, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)

def perform_pick(pick_point, above_point, tempo_dps=60, cost_mode="vertical_down"):
    open_gripper()
    time.sleep(SLEEP_TIME)
    arm.move_to_point_full(*pick_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)
    close_gripper()
    time.sleep(SLEEP_TIME)
    arm.move_to_point_full(*above_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)

def perform_place(place_point, above_point, tempo_dps=60, cost_mode="vertical_down"):
    arm.move_to_point_full(*place_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)
    open_gripper()
    time.sleep(SLEEP_TIME)
    arm.move_to_point_full(*above_point, tempo_dps, cost_mode)
    time.sleep(SLEEP_TIME)
    close_gripper()

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
pick_drop(point1, empty_slot)
pick_drop(point2, point1)
pick_drop(point3, point2)
pick_drop(empty_slot, point3)
arm.homepos()  # Return to home position after operations