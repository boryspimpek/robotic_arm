from math import cos, hypot, sin
import pygame
import time
from ik import solve_ik_full, solve_ik_wrist
from config import SERVO_LIMITS, DEADZONE, INITIAL_POSITION, l1, l2, l3
from st3215 import ST3215
from scservo_sdk import gripper

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
    for id, target in servo_targets.items():
        min_angle, max_angle = SERVO_LIMITS[id]
        if not (min_angle <= target <= max_angle):
            errors.append(
                f"Kąt serwa {id} poza zakresem ({min_angle}-{max_angle}): {target}"
            )
    return errors

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

def find_wrist_point(angles):
    theta1, theta2, theta3, theta4 = angles

    wrist_x = l1 * cos(theta2) * cos(theta1) + l2 * cos(theta2 + theta3) * cos(theta1)
    wrist_y = l1 * cos(theta2) * sin(theta1) + l2 * cos(theta2 + theta3) * sin(theta1)
    wrist_z = l1 * sin(theta2) + l2 * sin(theta2 + theta3)

    wrist_point = (wrist_x, wrist_y, wrist_z)
    return wrist_point

def singularity_check(angles, max_speed):
    wrist_x, wrist_y, wrist_z = find_wrist_point(angles)
    
    wrist_distance = hypot(wrist_x, wrist_y, wrist_z)

    max_reach = l1 + l2
    distance_to_max = max_reach - wrist_distance
    print(f"distance to max: {distance_to_max:.2f}")

    exponent = 2
    normalized = distance_to_max / (l1 + l2)
    corrected_speed = (1 - (1 - normalized) ** exponent) * max_speed
    corrected_speed = max(400, round(corrected_speed))  
    # print(f"corrected speed: {corrected_speed:.2f}")

    return corrected_speed

def move_to_point(point, method, orientation_mode, max_speed=1000):
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
