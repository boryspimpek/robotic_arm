import math
import numpy as np
from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')

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
    
    # Sprawdź ograniczenia bazy (-90° do 90°)
    if not (-np.pi/2 <= theta1 <= np.pi/2):
        return None
    
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
        
        # Sprawdź ograniczenia stawów
        if not (0 <= theta2 <= np.pi):  # pierwsze ramię 0-180°
            continue
        if not (-130*np.pi/180 <= theta3 <= 130*np.pi/180):  # drugie ramię -130° do 130°
            continue
        if not (-135*np.pi/180 <= theta4 <= 135*np.pi/180):  # trzecie ramię -140° do 140°
            continue
        
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
    raw_position = 4096 - int(round(rad * scale + center))
    return raw_position


#############################################################

names = ["Base", "Shoulder", "Elbow", "Wrist"]
ids = [1, 2, 3, 4]

# obliczenie kątów z IK
rad_angles = solve_ik(120, 0, 120, "vertical_up")

for name, angle in zip(names, rad_angles):
    print(f"{name:10}: {angle:.2f}°")
print("===============================")

for name, angle in zip(names, rad_angles):
    print(f"{name:10}: {np.degrees(angle):.2f}°")
print("===============================")

# przeliczenie kątów na jednostki serw
servo_angles = [rad_to_servo(angle) for angle in rad_angles]
print(servo_angles)

# ruch serw
for sid, angle in zip(ids, servo_angles):
    servo.MoveTo(sid, angle, 500, 50)


