import time
import math
from math import cos, pi, sin
from math import cos, hypot, sin
import numpy as np

from scservo_sdk import gripper
from utilis import move_to_point


method = "wrist"
orientation_mode = "down"

home = (120, 0, 150)
speed_approach = 500
speed_move = 700
t = 0.2


def pickup(point):
    above = (point[0], point[1], point[2] + 40)

    # Move above the pickup location
    move_to_point(above, method, orientation_mode, speed_move)
    time.sleep(t)

    # Open gripper before moving down
    gripper("open")
    time.sleep(t)

    # Move down to pickup point
    move_to_point(point, method, orientation_mode, speed_approach)
    time.sleep(t)

    # Close gripper to grab object
    gripper("close")
    time.sleep(t)

    # Move back up
    move_to_point(above, method, orientation_mode, speed_approach)
    time.sleep(t)

    # # Return home
    # move_to_point(home, method, orientation_mode, speed_move)
    # time.sleep(t)

def place(point):
    above = (point[0], point[1], point[2] + 40)

    # gripper("open")
    # time.sleep(2)

    # gripper("close")
    # time.sleep(0.5)

    # Move above the target
    move_to_point(above, method, orientation_mode, speed_move)
    time.sleep(t)

    # Move down to the place position
    move_to_point(point, method, orientation_mode, speed_approach)
    time.sleep(t)

    # Open gripper to release object
    gripper("open")
    time.sleep(t)

    # Move back up
    move_to_point(above, method, orientation_mode, speed_approach)
    time.sleep(t)

    # Close gripper befor returning to home
    gripper("close")
    time.sleep(t)

    # # Return home
    # move_to_point(home, method, orientation_mode, speed_move)
    # time.sleep(0.5)

def generate_points(start_point, num_objects=6, step=-40):
    """
    Generuje x punktów na podstawie jednego punktu startowego.
    
    start_point: tuple (x, y, z) – punkt pierwszego klocka
    num_objects: liczba klocków do przeniesienia
    step: przesunięcie w osi Y
    
    Zwraca listę punktów: [pickup1, pickup2, pickup3...]
    """
    x, y, z = start_point
    points = [(x, y + i * step, z) for i in range(num_objects)]
    return points

def bricks_forward(points):
    """
    Przenosi pierwszą połowę klocków do drugiej połowy pozycji.
    Działa z dowolną liczbą punktów.
    """
    num_objects = len(points)
    half = num_objects // 2
    
    for i in range(half):
        pickup(points[i])
        place(points[i + half])
        print(f"Przeniesiono klocek {i+1} z pozycji {points[i]} do {points[i + half]}")

def bricks_backward(points):
    """
    Przenosi drugą połowę klocków do pierwszej połowy pozycji.
    Działa z dowolną liczbą punktów.
    """
    num_objects = len(points)
    half = num_objects // 2
    
    for i in range(half):
        pickup(points[i + half])
        place(points[i])
        print(f"Przeniesiono klocek {i+half+1} z pozycji {points[i + half]} do {points[i]}")


move_to_point(home, method, orientation_mode, speed_move)
time.sleep(0.5)

start = (100, 60, -5)
points_6 = generate_points(start, num_objects=6)

bricks_forward(points_6)
# bricks_backward(points_6)

move_to_point(home, method, orientation_mode, speed_move)
time.sleep(0.5)
