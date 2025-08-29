import time
import math
from math import cos, pi, sin

import numpy as np
import pygame

from st3215 import ST3215

from utilis import l1, l2, l3, INITIAL_POSITION
from utilis import servo_to_rad, rad_to_servo, check_servo_angles, find_wrist_point, singularity_check, initialize_joystick, process_joystick_input

TRIANGLE_BUTTON_ID = 2
CIRCLE_BUTTON_ID = 1
CROSS_BUTTON_ID = 0

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
    step = 2 if method == "wrist" else 5
    joystick = initialize_joystick()
    current_position = INITIAL_POSITION
    button_states = {TRIANGLE_BUTTON_ID: 0, CIRCLE_BUTTON_ID: 0, CROSS_BUTTON_ID: 0}
    
    move_to_point(current_position, method, max_speed=500)
    
    try:
        while True:
            triangle, circle, cross = (joystick.get_button(btn) for btn in [TRIANGLE_BUTTON_ID, CIRCLE_BUTTON_ID, CROSS_BUTTON_ID])
            
            if triangle == 1 and button_states[TRIANGLE_BUTTON_ID] == 0:
                orientation_mode = "down"
                move_to_point(current_position, method)
            
            if circle == 1 and button_states[CIRCLE_BUTTON_ID] == 0:
                orientation_mode = "flat"
                move_to_point(current_position, method)
            
            if cross == 1 and button_states[CROSS_BUTTON_ID] == 0:
                method = "wrist" if method == "full" else "full"
                step = 3 if method == "wrist" else 6
                if method == "wrist":
                    wrist_point = find_wrist_point(solve_ik_full(*current_position))
                    move_to_point(wrist_point, "wrist")
                    current_position = wrist_point
                print(f"Zmieniono tryb na: {method}, step = {step}")

            button_states = {TRIANGLE_BUTTON_ID: triangle, CIRCLE_BUTTON_ID: circle, CROSS_BUTTON_ID: cross}
            
            new_position = process_joystick_input(joystick, current_position, step)
            
            if new_position != current_position:
                try:
                    move_to_point(new_position, method)
                    current_position = new_position
                    print(f"Position: ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f}), Method: {method}, Orientation: {orientation_mode}, Step: {step}")
                
                except ValueError as e:
                    print(f"Nieosiągalna pozycja: {e}")
                except Exception as e:
                    print(f"Błąd ruchu: {e}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Sterowanie zakończone")

if __name__ == "__main__":
    main()