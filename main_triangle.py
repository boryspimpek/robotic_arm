import pygame
import time
import math
from math import pi, radians, degrees
import numpy as np
from st3215 import ST3215
from utilis import check_servo_angles, initialize_joystick, servo_to_rad, rad_to_servo, LINK_LENGTHS, singularity_check

step = 5
DEADZONE = 0.8
INITIAL_POSITION = (200, 0, 110)

# Definicje przycisków
TRIANGLE_BUTTON_ID = 2
CIRCLE_BUTTON_ID = 1
CROSS_BUTTON_ID = 0

method = "full"
orientation_mode = "flat"

servo = ST3215('/dev/ttyACM0')

def solve_ik_full(x_target, y_target, z_target):
    """Rozwiązanie IK dla pełnego manipulatora 4-DOF"""
    l1, l2, l3 = LINK_LENGTHS
    delta_theta = np.radians(1)
    theta4_candidates = np.arange(-np.pi, np.pi, delta_theta)

    r_target = math.hypot(x_target, y_target)
    px = r_target
    py = z_target

    d = math.hypot(px, py)
    if d > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    theta1 = math.atan2(y_target, x_target)

    min_cost = float('inf')
    best_angles = None

    for theta4_c in theta4_candidates:
        wrist_r = r_target - l3 * math.cos(theta4_c)
        wrist_z = z_target - l3 * math.sin(theta4_c)

        D = math.hypot(wrist_r, wrist_z)
        if D > (l1 + l2) or D < abs(l1 - l2):
            continue

        cos_theta3 = (wrist_r**2 + wrist_z**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if not (-1 <= cos_theta3 <= 1):
            continue

        theta3 = -math.acos(cos_theta3)
        k1 = l1 + l2 * math.cos(theta3)
        k2 = l2 * math.sin(theta3)
        theta2 = math.atan2(wrist_z, wrist_r) - math.atan2(k2, k1)
        theta4 = theta4_c - (theta2 + theta3)

        # Koszt normalny: minimalizacja theta3 i theta4
        cost = theta3**2 + theta4**2

        if cost < min_cost:
            min_cost = cost
            best_angles = (theta1, theta2, theta3, theta4)

    if best_angles is None:
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    return best_angles

def solve_ik_wrist(x_target, y_target, z_target, orientation_mode):
    """Rozwiązanie IK dla pozycji nadgarstka (tylko theta1, theta2, theta3)"""
    l1, l2, _ = LINK_LENGTHS  # Ignorujemy l3

    r_target = math.hypot(x_target, y_target)
    px = r_target
    py = z_target

    d = math.hypot(px, py)
    if d > (l1 + l2):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    theta1 = math.atan2(y_target, x_target)

    cos_theta3 = (r_target**2 + z_target**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    if not (-1 <= cos_theta3 <= 1):
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    theta3 = -math.acos(cos_theta3)
    
    k1 = l1 + l2 * math.cos(theta3)
    k2 = l2 * math.sin(theta3)
    theta2 = math.atan2(z_target, r_target) - math.atan2(k2, k1)

    if orientation_mode == "down":
        theta4 = -pi/2 - (theta2 + theta3)  # Końcówka skierowana w dół
    elif orientation_mode == "flat":
        theta4 = 0 - (theta2 + theta3)  # Końcówka poziomo

    best_angles = (theta1, theta2, theta3, theta4)
    return best_angles

def move_to_point(point, method, max_speed=2400):
    x, y, z = point
    servo_ids = [1, 2, 3, 4]

    current_angles = [servo_to_rad(servo.ReadPosition(id)) for id in servo_ids]
    
    if method == "full":
        angles = solve_ik_full(x, y, z)
    elif method == "wrist":
        angles = solve_ik_wrist(x, y, z, orientation_mode)

    corrected_speed = singularity_check(angles, max_speed)

    delta_angles = [abs(target - current) for target, current in zip(angles, current_angles)]
    max_delta = max(delta_angles) if delta_angles else 0
    
    servo_speeds = [int((delta / max_delta) * corrected_speed) if max_delta != 0 else 0 for delta in delta_angles]
    servo_targets = [rad_to_servo(angle) for angle in angles]

    errors = check_servo_angles(servo_targets)
    if errors:
        print("Błędy:", errors)
        return
    
    for id, target, speed in zip(servo_ids, servo_targets, servo_speeds):
        servo.MoveTo(id, target, speed, 150)

def process_joystick_input(joystick, current_pos, step_size):
    pygame.event.pump()
    
    x, y, z = current_pos
    
    ly = joystick.get_axis(0)
    lx = joystick.get_axis(1)
    ry = joystick.get_axis(4)

    lx = 0 if abs(lx) < DEADZONE else lx
    ly = 0 if abs(ly) < DEADZONE else ly
    ry = 0 if abs(ry) < DEADZONE else ry
    
    x += -lx * step_size
    y -= ly * step_size
    z -= ry * step_size
    
    return (x, y, z)

def main():
    global method, orientation_mode
    joystick = initialize_joystick()
    current_position = INITIAL_POSITION
    
    move_to_point(current_position, method, 1500)
    time.sleep(2)

    last_triangle_state = 0
    last_circle_state = 0
    last_cross_state = 0
    
    try:
        while True:
            triangle_state = joystick.get_button(TRIANGLE_BUTTON_ID)
            circle_state = joystick.get_button(CIRCLE_BUTTON_ID)
            cross_state = joystick.get_button(CROSS_BUTTON_ID)
            
            if triangle_state == 1 and last_triangle_state == 0:
                orientation_mode = "down"
            
            if circle_state == 1 and last_circle_state == 0:
                orientation_mode = "flat"
            
            if cross_state == 1 and last_cross_state == 0:
                method = "wrist" if method == "full" else "full"

            last_triangle_state = triangle_state
            last_circle_state = circle_state
            last_cross_state = cross_state
            
            new_position = process_joystick_input(joystick, current_position, step)
            
            if new_position != current_position:
                try:
                    move_to_point(new_position, method)
                    current_position = new_position
                    # print(f"Position: ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f}), Method: {method}, Orientation: {orientation_mode}")
                
                except ValueError as e:
                    print(f"Nieosiągalna pozycja: {e}")
                except Exception as e:
                    print(f"Błąd ruchu: {e}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Sterowanie zakończone")

if __name__ == "__main__":
    main()