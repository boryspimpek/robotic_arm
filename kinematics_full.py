import math
import numpy as np



l1 = 120
l2 = 120
l3 = 110
delta_deg = 1

def solve_ik(x_target, y_target, z_target, cost_mode):
    delta_theta = np.radians(delta_deg)
    theta3_candidates = np.arange(-np.pi, np.pi, delta_theta)

    r_target = np.hypot(x_target, y_target)
    px = r_target
    py = z_target

    d = math.hypot(px, py)
    if d > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem")

    theta0 = np.arctan2(y_target, x_target)

    min_cost = np.inf
    best_angles = None

    for theta3_c in theta3_candidates:
        wrist_r = r_target - l3 * np.cos(theta3_c)
        wrist_z = z_target - l3 * np.sin(theta3_c)

        D = np.hypot(wrist_r, wrist_z)
        if D > (l1 + l2) or D < abs(l1 - l2):
            continue

        cos_theta2 = (wrist_r**2 + wrist_z**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if not (-1 <= cos_theta2 <= 1):
            continue

        theta2 = -np.arccos(cos_theta2)  # Elbow-up
        k1 = l1 + l2 * np.cos(theta2)
        k2 = l2 * np.sin(theta2)
        theta1 = np.arctan2(wrist_z, wrist_r) - np.arctan2(k2, k1)
        theta3 = theta3_c - (theta1 + theta2)

        if cost_mode == "min_angle_sum":
            cost = theta0**2 + theta1**2 + theta2**2 + theta3**2
        elif cost_mode == "vertical_up":
            end_orientation = theta1 + theta2 + theta3
            cost = (end_orientation - np.pi/2)**2    
        elif cost_mode == "vertical_down":
            end_orientation = theta1 + theta2 + theta3
            cost = (end_orientation + np.pi/2)**2            
        elif cost_mode == "flat":
            end_orientation = theta1 + theta2 + theta3
            cost = (end_orientation)**2  
        elif cost_mode == "normal":
            cost = (theta2)**2 + (theta3)**2
        else:
            raise ValueError("Nieznany tryb kosztu")
        
        if cost < min_cost:
            min_cost = cost

            phi_deg = np.degrees(theta0)
            theta1_deg = np.degrees(theta1)
            theta2_deg = np.degrees(theta2)
            theta3_deg = np.degrees(theta3)

            best_angles = (phi_deg, theta1_deg, theta2_deg, theta3_deg)
        

    return best_angles

