import math
import numpy as np
from st3215 import ST3215
from scservo_sdk import gripper

import time

servo = ST3215('/dev/ttyACM0')

ids = [1, 2, 3, 4]
current_positions = {}
servo_targets = {}
max_speed = 1000
wait = True
INITIAL_POSITION = (200, 0, 110)


# Read and store current positions
for sts_id in ids:
    pos = servo.ReadPosition(sts_id)
    current_positions[sts_id] = pos
    print(f"Servo {sts_id} position: {pos}")


def base():
    for i in range(1):

        # First move
        servo_targets = {
            1: 3072,
            2: 385,
            3: 3763,
            4: 2697
        }
        servo.SyncMoveTo(servo_targets, max_speed=1000, wait=True)
        time.sleep(0.5)

        # Second move
        servo_targets = {
            1: 1024,
            2: 385,
            3: 3763,
            4: 2697
        }
        servo.SyncMoveTo(servo_targets, max_speed=1000, wait=True)
        time.sleep(0.5)

def schoulder():
    for i in range(1):

        # First move
        servo_targets = {
            1: 1024,
            2: 0,
            3: 2048,
            4: 2048
        }
        servo.SyncMoveTo(servo_targets, max_speed=1000, wait=True)
        time.sleep(0.5)

        # Second move
        servo_targets = {
            1: 1024,
            2: 1995,
            3: 2048,
            4: 2048
        }
        servo.SyncMoveTo(servo_targets, max_speed=700, wait=True)

def elbow():
    for i in range(1):

        # First move
        servo_targets = {
            1: 1024,
            2: 990,
            3: 3600,
            4: 2048
        }
        servo.SyncMoveTo(servo_targets, max_speed=1000, wait=True)
        time.sleep(0.5)

        # Second move
        servo_targets = {
            1: 1024,
            2: 990,
            3: 500,
            4: 2048
        }
        servo.SyncMoveTo(servo_targets, max_speed=700, wait=True)

def wrist():
    for i in range(1):

        # First move
        servo_targets = {
            1: 1024,
            2: 990,
            3: 1475,
            4: 700
        }
        servo.SyncMoveTo(servo_targets, max_speed=1000, wait=True)
        time.sleep(0.5)

        # Second move
        servo_targets = {
            1: 1024,
            2: 990,
            3: 1475,
            4: 3400
        }
        servo.SyncMoveTo(servo_targets, max_speed=700, wait=True)

def home():
    for i in range(1):

        # First move
        servo_targets = {
            1: 1024,
            2: 990,
            3: 1475,
            4: 3400
        }
        servo.SyncMoveTo(servo_targets, max_speed=1000, wait=True)
        time.sleep(0.5)

        # Second move
        servo_targets = {
            1: 2048,
            2: 385,
            3: 3763,
            4: 2697
        }
        servo.SyncMoveTo(servo_targets, max_speed=700, wait=True)

def gripperos():
    servo_targets = {
        1: 2682,
        2: 1150,
        3: 2729,
        4: 2545
    }
    servo.SyncMoveTo(servo_targets, max_speed=300, wait=True)
    time.sleep(0.5)

    gripper("open")
    gripper("close")
    gripper("open")
    gripper("close")
    gripper("open")
    gripper("close")        

    servo_targets = {
    1: 2048,
    2: 385,
    3: 3763,
    4: 2697
    }
    servo.SyncMoveTo(servo_targets, max_speed=300, wait=True)

def dance():
    pos1 = {
        1: 1270,
        2: 330,
        3: 3633,
        4: 2935
    }

    pos2 = {
        1: 3009,
        2: 1813,
        3: 462,
        4: 1184
    }
    time.sleep(2)
    servo.SyncMoveTo(pos1, max_speed=1000, wait=True)
    time.sleep(0.5)

    servo.SyncMoveTo(pos2, max_speed=1000, wait=True)
    time.sleep(0.5)

def show_axes():
    x = {
        1: 2048,
        2: 2020,
        3: 2017,
        4: 2101
    }

    y = {
        1: 1024,
        2: 2020,
        3: 2017,
        4: 2101
    }

    z = {
        1: 2048,
        2: 1024,
        3: 2017,
        4: 2101
    }
    home = {
        1: 2048,
        2: 385,
        3: 3763,
        4: 2697
    }

    home_y = {
        1: 1024,
        2: 385,
        3: 3763,
        4: 2697
    }


    servo.SyncMoveTo(home, max_speed=700, wait=True)
    time.sleep(1)
    servo.SyncMoveTo(x, max_speed=500, wait=True)
    time.sleep(1)

    servo.SyncMoveTo(home_y, max_speed=700, wait=True)
    time.sleep(1)
    servo.SyncMoveTo(y, max_speed=500, wait=True)
    time.sleep(1)

    servo.SyncMoveTo(home, max_speed=700, wait=True)
    time.sleep(1)
    servo.SyncMoveTo(z, max_speed=500, wait=True)
    time.sleep(1)

    servo.SyncMoveTo(home, max_speed=700, wait=True)


time.sleep(3)
show_axes()