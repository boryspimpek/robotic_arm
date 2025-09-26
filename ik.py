import time
import numpy as np
import math
from math import sin, cos, hypot, pi
from config import l1, l2, l3


def solve_ik(x_target, y_target, z_target, theta4_desired):
    if theta4_desired is None:
        theta4_candidates = np.arange(-np.pi, np.pi, np.radians(3))
    else:
        theta4_candidates = [theta4_desired]

    r_target = math.hypot(x_target, y_target)
    theta1 = math.atan2(y_target, x_target)

    if (d := math.hypot(r_target, z_target)) > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    min_cost, best_angles = float('inf'), None

    for theta4_c in theta4_candidates:
        wrist_r = r_target - l3 * math.cos(theta4_c)
        wrist_z = z_target - l3 * math.sin(theta4_c)

        D = math.hypot(wrist_r, wrist_z)
        if not (abs(l1 - l2) <= D <= (l1 + l2)):
            continue

        cos_theta3 = (wrist_r**2 + wrist_z**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if not -1 <= cos_theta3 <= 1:
            continue

        theta3 = -math.acos(cos_theta3)
        k1, k2 = l1 + l2 * math.cos(theta3), l2 * math.sin(theta3)
        theta2 = math.atan2(wrist_z, wrist_r) - math.atan2(k2, k1)
        theta4 = theta4_c - (theta2 + theta3)

        cost = theta3**2 + theta4**2
        if cost < min_cost:
            min_cost, best_angles = cost, (theta1, theta2, theta3, theta4)

    if best_angles is None:
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    return best_angles


def forward_kinematics(theta0, theta1, theta2, theta3):
    # suma kątów dla płaszczyzny XY-Z
    t1 = theta1
    t2 = theta1 + theta2
    t3 = theta1 + theta2 + theta3

    # współrzędne w układzie obróconym o theta0
    r = l1 * math.cos(t1) + l2 * math.cos(t2) + l3 * math.cos(t3)
    z = l1 * math.sin(t1) + l2 * math.sin(t2) + l3 * math.sin(t3)

    # obrót wokół osi Z (podstawa)
    x = r * math.cos(theta0)
    y = r * math.sin(theta0)

    return x, y, z, t3


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
