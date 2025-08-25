import math
import numpy as np

l1 = 120
l2 = 120
l3 = 110
delta_deg = 1

def solve_ik(x_target, y_target, z_target, cost_mode):
    delta_theta = np.radians(delta_deg)
    theta4_candidates = np.arange(-np.pi, np.pi, delta_theta)  

    r_target = np.hypot(x_target, y_target)
    px = r_target
    py = z_target

    d = math.hypot(px, py)
    if d > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem")

    theta1 = np.arctan2(y_target, x_target)  

    min_cost = np.inf
    best_angles = None

    for theta4_c in theta4_candidates:
        wrist_r = r_target - l3 * np.cos(theta4_c)
        wrist_z = z_target - l3 * np.sin(theta4_c)

        D = np.hypot(wrist_r, wrist_z)
        if D > (l1 + l2) or D < abs(l1 - l2):
            continue

        cos_theta3 = (wrist_r**2 + wrist_z**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if not (-1 <= cos_theta3 <= 1):
            continue

        theta3 = -np.arccos(cos_theta3)  
        k1 = l1 + l2 * np.cos(theta3)
        k2 = l2 * np.sin(theta3)
        theta2 = np.arctan2(wrist_z, wrist_r) - np.arctan2(k2, k1) 
        theta4 = theta4_c - (theta2 + theta3)  

        if cost_mode == "min_angle_sum":
            cost = theta1**2 + theta2**2 + theta3**2 + theta4**2
        elif cost_mode == "vertical_up":
            end_orientation = theta2 + theta3 + theta4
            cost = (end_orientation - np.pi/2)**2    
        elif cost_mode == "vertical_down":
            end_orientation = theta2 + theta3 + theta4
            cost = (end_orientation + np.pi/2)**2            
        elif cost_mode == "flat":
            end_orientation = theta2 + theta3 + theta4
            cost = (end_orientation)**2  
        elif cost_mode == "normal":
            cost = (theta3)**2 + (theta4)**2
        else:
            raise ValueError("Nieznany tryb kosztu")
        
        if cost < min_cost:
            min_cost = cost


            best_angles = (theta1, theta2, theta3, theta4)
        
    return best_angles

def rad_to_servo(rad):
    center = 2048
    scale = 2048 / math.pi
    raw_position = 4095 - int(round(rad * scale + center))
    return raw_position

names = ["Base", "Shoulder", "Elbow", "Wrist"]


angles = solve_ik(240, 0, 90, "flat")

for name, angle in zip(names, angles):
    print(f"{name:10}: {angle:.2f}°")
print("===============================")

for name, angle in zip(names, angles):
    print(f"{name:10}: {np.degrees(angle):.2f}°")
print("===============================")

servo_angle = [rad_to_servo(angle) for angle in angles]
print(servo_angle)