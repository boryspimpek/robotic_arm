import time
import math
from math import cos, pi, sin
from math import cos, hypot, sin
import numpy as np
from config import l1, l2, l3

def solve_ik_full(x_target, y_target, z_target):
    theta4_candidates = np.arange(-np.pi, np.pi, np.radians(3))
    
    r_target = math.hypot(x_target, y_target)
    theta1 = math.atan2(y_target, x_target)
    
    if (d := math.hypot(r_target, z_target)) > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    min_cost, best_angles = float('inf'), None

    for theta4_c in theta4_candidates:
        wrist_r, wrist_z = r_target - l3 * math.cos(theta4_c), z_target - l3 * math.sin(theta4_c)
        
        if not (abs(l1 - l2) <= (D := math.hypot(wrist_r, wrist_z)) <= (l1 + l2)):
            continue
            
        if not (-1 <= (cos_theta3 := (wrist_r**2 + wrist_z**2 - l1**2 - l2**2) / (2 * l1 * l2)) <= 1):
            continue

        theta3 = -math.acos(cos_theta3)
        k1, k2 = l1 + l2 * math.cos(theta3), l2 * math.sin(theta3)
        theta2 = math.atan2(wrist_z, wrist_r) - math.atan2(k2, k1)
        theta4 = theta4_c - (theta2 + theta3)

        if (cost := theta3**2 + theta4**2) < min_cost:
            min_cost, best_angles = cost, (theta1, theta2, theta3, theta4)

    if best_angles is None:
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    return best_angles

def solve_ik_wrist(x_target, y_target, z_target, orientation_mode):
    r_target, theta1 = math.hypot(x_target, y_target), math.atan2(y_target, x_target)
    
    if (d := math.hypot(r_target, z_target)) > (l1 + l2):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    if not (-1 <= (cos_theta3 := (r_target**2 + z_target**2 - l1**2 - l2**2) / (2 * l1 * l2)) <= 1):
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    theta3 = -math.acos(cos_theta3)
    k1, k2 = l1 + l2 * math.cos(theta3), l2 * math.sin(theta3)
    theta2 = math.atan2(z_target, r_target) - math.atan2(k2, k1)
    
    theta4 = (-pi/2 if orientation_mode == "down" else 0) - (theta2 + theta3)
    
    return (theta1, theta2, theta3, theta4)

def find_wrist_point(angles):
    theta1, theta2, theta3, theta4 = angles

    wrist_x = l1 * cos(theta2) * cos(theta1) + l2 * cos(theta2 + theta3) * cos(theta1)
    wrist_y = l1 * cos(theta2) * sin(theta1) + l2 * cos(theta2 + theta3) * sin(theta1)
    wrist_z = l1 * sin(theta2) + l2 * sin(theta2 + theta3)

    wrist_point = (wrist_x, wrist_y, wrist_z)
    return wrist_point

def singularity_check(angles, max_speed):
    wrist_x, wrist_y, wrist_z = find_wrist_point(angles)
    
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
