import time
from math import cos, hypot, sin
import pygame

from config import DEADZONE, INITIAL_POSITION, SERVO_LIMITS, home, l1, l2, l3
from ik import solve_ik_full, solve_ik_wrist
from ik_2d import solve_ik_full_2d, solve_ik_wrist_2d

from scservo_sdk import gripper
from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')

def rad_to_servo(rad):
    center = 2048
    scale = 2048 / 3.1415926535
    return 4095 - int(round(rad * scale + center))

def servo_to_rad(raw_position):
    center = 2048
    scale = 2048 / 3.1415926535
    return ((4095 - raw_position) - center) / scale

def servo_positions():
    ids = [1, 2, 3, 4]
    positions = {}
    for i in ids:
        positions[i] = servo.ReadPosition(i)  # Dodaj do słownika
    print(positions)    
    return positions

def go_home():
    servo.SyncMoveTo(home, max_speed=500)

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

def process_joystick_input_2d(joystick, current_pos, step_size=0.1):
    pygame.event.pump()
    
    x, z = current_pos
    
    lx = joystick.get_axis(1)  # Lewy joystick X
    ry = joystick.get_axis(4)  # Prawy joystick Y
    rx = joystick.get_axis(0)  # Prawy joystick X - dla obrotu podstawy

    # Deadzone dla wszystkich osi
    lx = 0 if abs(lx) < DEADZONE else lx
    ry = 0 if abs(ry) < DEADZONE else ry
    rx = 0 if abs(rx) < DEADZONE else rx
    
    # Nieliniowa charakterystyka dla płynniejszego sterowania
    lx_nonlinear = lx * abs(lx)  # lx^2 zachowując znak
    ry_nonlinear = ry * abs(ry)  # ry^2 zachowując znak
    
    # Płynne sterowanie pozycją z nieliniową charakterystyką
    x += -lx_nonlinear * step_size
    z -= ry_nonlinear * step_size
    
    # Płynniejsze sterowanie obrotem podstawy z nieliniową charakterystyką
    # Kwadratowa charakterystyka dla lepszej kontroli przy małych wychyleniach
    rotation_input = rx * abs(rx)  # rx^2 zachowując znak
    
    return (x, z), rotation_input

def move_to_point(point, method, orientation_mode, max_speed=2000):
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

def move_to_point_2d(point, method, orientation_mode, base_rotation, max_speed=1000):
    angles = solve_ik_full_2d(*point) if method == "full" else solve_ik_wrist_2d(*point, orientation_mode)
    servo_angles = [rad_to_servo(angle) for angle in angles]

    servo_targets = {
        1: base_rotation,  
        2: servo_angles[0],
        3: servo_angles[1],
        4: servo_angles[2]
    }

    if errors := check_servo_angles(servo_targets): 
        print("Błędy:", errors)
        return
    
    servo.SyncMoveTo(servo_targets, max_speed)