import time
import numpy as np
import math
from math import sin, cos, hypot, pi
from config import l1, l2, l3


def singularity_check(angles, max_speed):
    wrist_x, wrist_y, wrist_z = find_wrist_point(angles) # type: ignore
    
    wrist_distance = hypot(wrist_x, wrist_y, wrist_z)

    max_reach = l1 + l2
    distance_to_max = max_reach - wrist_distance
    print(f"distance to max: {distance_to_max:.2f}")

    exponent = 2
    normalized = distance_to_max / (l1 + l2)
    corrected_speed = (1 - (1 - normalized) ** exponent) * max_speed
    corrected_speed = max(400, round(corrected_speed))  
    # print(f"corrected speed: {corrected_speed:.2f}")

    return corrected_speed

def find_wrist_point(angles):
    theta1, theta2, theta3 = angles

    wrist_x = l1 * cos(theta2) * cos(theta1) + l2 * cos(theta2 + theta3) * cos(theta1)
    wrist_y = l1 * cos(theta2) * sin(theta1) + l2 * cos(theta2 + theta3) * sin(theta1)
    wrist_z = l1 * sin(theta2) + l2 * sin(theta2 + theta3)

    wrist_point = (wrist_x, wrist_y, wrist_z)
    return wrist_point

