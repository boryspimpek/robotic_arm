import math
import numpy as np
from config import l1, l2, l3

def solve_ik_full_2d(x_target, z_target):
    """
    Rozwiązanie IK w 2D dla pełnego manipulatora (3 ogniwa)
    Zwraca kąty: theta2, theta3, theta4
    """
    theta4_candidates = np.arange(-np.pi, np.pi, np.radians(3))
    
    if (d := math.hypot(x_target, z_target)) > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    min_cost, best_angles = float('inf'), None

    for theta4_c in theta4_candidates:
        wrist_x = x_target - l3 * math.cos(theta4_c)
        wrist_z = z_target - l3 * math.sin(theta4_c)
        
        if not (abs(l1 - l2) <= (D := math.hypot(wrist_x, wrist_z)) <= (l1 + l2)):
            continue
            
        if not (-1 <= (cos_theta3 := (wrist_x**2 + wrist_z**2 - l1**2 - l2**2) / (2 * l1 * l2)) <= 1):
            continue

        theta3 = -math.acos(cos_theta3)
        k1, k2 = l1 + l2 * math.cos(theta3), l2 * math.sin(theta3)
        theta2 = math.atan2(wrist_z, wrist_x) - math.atan2(k2, k1)
        theta4 = theta4_c - (theta2 + theta3)

        if (cost := theta3**2 + theta4**2) < min_cost:
            min_cost, best_angles = cost, (theta2, theta3, theta4)
        
    if best_angles is None:
        raise ValueError("Brak rozwiązania IK dla tej pozycji")
    
    return best_angles

def solve_ik_wrist_2d(x_target, z_target, orientation_mode):
    """
    Rozwiązanie IK w 2D dla pozycji nadgarstka (2 ogniwa)
    Zwraca kąty: theta2, theta3, theta4
    """
    if (d := math.hypot(x_target, z_target)) > (l1 + l2):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    if not (-1 <= (cos_theta3 := (x_target**2 + z_target**2 - l1**2 - l2**2) / (2 * l1 * l2)) <= 1):
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    theta3 = -math.acos(cos_theta3)
    k1, k2 = l1 + l2 * math.cos(theta3), l2 * math.sin(theta3)
    theta2 = math.atan2(z_target, x_target) - math.atan2(k2, k1)
    
    theta4 = (-math.pi/2 if orientation_mode == "down" else 0) - (theta2 + theta3)
    
    return (theta2, theta3, theta4)

def find_wrist_point_2d(angles):
    """
    Znajduje pozycję nadgarstka w płaszczyźnie 2D (X-Z)
    angles: kąty (theta2, theta3) w radianach
    returns: (x, z) pozycja nadgarstka
    """
    theta2, theta3, _ = angles
    
    wrist_x = l1 * math.cos(theta2) + l2 * math.cos(theta2 + theta3)
    wrist_z = l1 * math.sin(theta2) + l2 * math.sin(theta2 + theta3)

    wrist_point = (wrist_x, wrist_z)
    return wrist_point