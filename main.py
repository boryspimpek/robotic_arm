import pygame
import time
import math
import numpy as np
from st3215 import ST3215
from math import radians, degrees
from jacobian import calculate_manipulability

LINK_LENGTHS = (120, 120, 110)  # l1, l2, l3
step = 5
DEADZONE = 0.8
INITIAL_POSITION = (200, 0, 110)
cost_mode = "flat"

SERVO_LIMITS = {
    1: (1024, 3072),
    2: (0, 2048), 
    3: (400, 3700),
    4: (600, 3500)
}

servo = ST3215('/dev/ttyACM0')

def rad_to_servo(rad):
    center = 2048
    scale = 2048 / 3.1415926535
    return 4095 - int(round(rad * scale + center))

def servo_to_rad(raw_position):
    center = 2048
    scale = 2048 / 3.1415926535
    return ((4095 - raw_position) - center) / scale

def check_servo_angles(servo_targets):
    errors = []
    for id, target in zip([1, 2, 3, 4], servo_targets):
        min_angle, max_angle = SERVO_LIMITS[id]
        if not (min_angle <= target <= max_angle):
            errors.append(f"Kąt serwa {id} poza zakresem ({min_angle}-{max_angle}): {target}")
    return errors

def solve_ik(x_target, y_target, z_target, cost_mode):
    global step  
    if cost_mode == "normal":
        step = 5  
    else:
        step = 2

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
    best_orientation_error = float('inf')
    tol = math.radians(2)

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

        end_orientation = theta2 + theta3 + theta4

        if cost_mode == "down":
            orientation_error = abs(end_orientation + math.pi/2)
            cost = orientation_error**2
        elif cost_mode == "flat":
            orientation_error = abs(end_orientation)
            cost = orientation_error**2
        else:
            cost = theta3**2 + theta4**2
            orientation_error = 0.0

        if cost < min_cost:
            min_cost = cost
            best_angles = (theta1, theta2, theta3, theta4)
            best_orientation_error = orientation_error

    if best_angles is None:
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    if best_orientation_error > tol:
        raise ValueError(f"Orientacja '{cost_mode}' poza zakresem ({degrees(best_orientation_error):.2f}°)")

    return best_angles

def move_to_point(point, max_speed=2400):
    x, y, z = point
    servo_ids = [1, 2, 3, 4]

    current_angles = [servo_to_rad(servo.ReadPosition(id)) for id in servo_ids]
    
    angles = solve_ik(x, y, z, cost_mode)
    theta1, theta2, theta3, theta4 = angles
    manipulat = calculate_manipulability(theta1, theta2, theta3, theta4)
    print(manipulat/1000)

    delta_angles = [abs(target - current) for target, current in zip(angles, current_angles)]
    max_delta = max(delta_angles) if delta_angles else 0
    
    servo_speeds = [int((delta / max_delta) * max_speed) if max_delta != 0 else 0 for delta in delta_angles]
    servo_targets = [rad_to_servo(angle) for angle in angles]

    errors = check_servo_angles(servo_targets)
    if errors:
        print("Błędy:", errors)
        return
    
    for id, target, speed in zip(servo_ids, servo_targets, servo_speeds):
        servo.MoveTo(id, target, speed, 150)

def initialize_joystick():
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        raise Exception("Nie wykryto pada PS4!")
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"Pad wykryty: {joystick.get_name()}")
    return joystick

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
    joystick = initialize_joystick()
    current_position = INITIAL_POSITION
    
    move_to_point(current_position, 400)
    time.sleep(2)
    
    try:
        while True:
            new_position = process_joystick_input(joystick, current_position, step)
            
            try:
                move_to_point(new_position)
                current_position = new_position
            except ValueError as e:
                print(f"Nieosiągalna pozycja: {e}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Sterowanie zakończone")

if __name__ == "__main__":
    main()