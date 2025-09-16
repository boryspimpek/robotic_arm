import math
import numpy as np
from config import l1, l2, l3


def solve_ik_2d(x_target, z_target, orientation_mode):
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
