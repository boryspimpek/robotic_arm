import time
import math
from math import cos, pi, sin

import numpy as np

from st3215 import ST3215

from utilis import l1, l2, l3
from utilis import servo_to_rad, rad_to_servo, check_servo_angles, find_wrist_point, singularity_check, initialize_joystick, process_joystick_input

method = "full"
orientation_mode = "flat"

servo = ST3215('/dev/ttyACM0')

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

def move_to_point(point, method, max_speed=1000):
    angles = solve_ik_full(*point) if method == "full" else solve_ik_wrist(*point, orientation_mode)
    servo_angles = [rad_to_servo(angle) for angle in angles]
    servo_targets = {
        1 : servo_angles[0],
        2 : servo_angles[1],
        3 : servo_angles[2],
        4 : servo_angles[3]
    }

    if errors := check_servo_angles(servo_targets): print("Błędy:", errors); return
    servo.SyncMoveTo(servo_targets, max_speed)

def main():
    global method, orientation_mode
    current_position = (200, 0, 120)
    
    move_to_point(current_position, method, max_speed=500)
    time.sleep(3)
    


if __name__ == "__main__":
    main()