import math
import numpy as np
l1, l2, l3 = [120, 120, 110]

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

angles = solve_ik_full(250, 0, 80)

print("Kąty [stopnie]:")
print(f"Theta1: {math.degrees(angles[0]):.2f}°")
print(f"Theta2: {math.degrees(angles[1]):.2f}°")
print(f"Theta3: {math.degrees(angles[2]):.2f}°")
print(f"Theta4: {math.degrees(angles[3]):.2f}°")
print("-" * 30)