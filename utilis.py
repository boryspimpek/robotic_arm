import time
import math
from math import cos, hypot, sin
import pygame

from config import DEADZONE, INITIAL_POSITION, SERVO_LIMITS, home, l1, l2, l3
from ik import solve_ik, forward_kinematics, solve_ik_2d

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
    
    lx = joystick.get_axis(1)  
    ry = joystick.get_axis(4)  
    rx = joystick.get_axis(0)  

    lx = 0 if abs(lx) < DEADZONE else lx
    ry = 0 if abs(ry) < DEADZONE else ry
    rx = 0 if abs(rx) < DEADZONE else rx
    
    lx_nonlinear = lx * abs(lx)  
    ry_nonlinear = ry * abs(ry)  
    
    x += -lx_nonlinear * step_size
    z -= ry_nonlinear * step_size
    
    rotation_input = rx * abs(rx)  
    
    return (x, z), rotation_input

def move_to_point(point, max_speed, acc, wait, theta4_desired):
    angles = solve_ik(*point, theta4_desired)
    servo_angles = [rad_to_servo(angle) for angle in angles]
    servo_targets = {j+1: servo_angles[j] for j in range(4)}

    if errors := check_servo_angles(servo_targets):
        print("Błędy:", errors)
        return    
    servo.SyncMoveTo(servo_targets, max_speed, acc, wait)

def move_to_point_2d(point, orientation_mode, base_rotation, max_speed, acc, wait):
    """ This one uses Base orientation because IK function 
        calculates angles only in x/ z plane. Base rotation is provided with joystkick. 
        This function is meant to be used with joystick controller"""
    
    angles = solve_ik_2d(*point, orientation_mode)
    # corrected_speed = singularity_check(angles, max_speed)
    servo_angles = [rad_to_servo(angle) for angle in angles]
    servo_targets = {
        1 : base_rotation,
        2 : servo_angles[0],
        3 : servo_angles[1],
        4 : servo_angles[2]
    }

    if errors := check_servo_angles(servo_targets):
        print("Błędy:", errors)
        return    
    servo.SyncMoveTo(servo_targets, max_speed, acc, wait)

def trajectory(x_start, y_start, z_start, theta4_start,
                 x_end, y_end, z_end, theta4_end,
                 steps=50, max_speed=700, acc=50, min_step_time=0.02):
    
    trajectory = []
    
    # Generowanie trajektorii (interpolacja między punktami)
    for i in range(steps + 1):
        t = i / steps

        # interpolacja pozycji
        x_t = x_start + (x_end - x_start) * t
        y_t = y_start + (y_end - y_start) * t
        z_t = z_start + (z_end - z_start) * t

        # interpolacja orientacji
        theta4_t = theta4_start + (theta4_end - theta4_start) * t

        # obliczanie kinematyki odwrotnej dla punktu pośredniego
        try:
            angles = solve_ik(x_t, y_t, z_t, theta4_desired=theta4_t)
            trajectory.append(angles)
        except ValueError:
            print(f"Brak rozwiązania IK w kroku {i} (t={t:.2f})")
            break

    # Wykonywanie ruchu po wygenerowanej trajektorii
    for i, angles in enumerate(trajectory):
        servo_angles = [rad_to_servo(angle) for angle in angles]
        servo_targets = {j+1: servo_angles[j] for j in range(4)}

        if errors := check_servo_angles(servo_targets):
            print("Błędy zakresu serw:", errors)
            break

        current_positions = {j: servo.ReadPosition(j) for j in servo_targets.keys()}
        distances = {j: abs(servo_targets[j] - current_positions[j]) for j in servo_targets.keys()}

        if max_distance := max(distances.values()):
            step_time = max_distance / max_speed
            step_time = max(step_time, min_step_time)  # Zapewnienie minimalnego czasu kroku
        else:
            step_time = min_step_time

        servo.SyncMoveTo(servo_targets, max_speed=max_speed, acc=acc, wait=False)
        time.sleep(step_time)

def line_in_space(step):
    servo_angles = servo_positions()
    print(f"Servo_angles {servo_angles}") # wynik w takiej postaci {1: 1449, 2: 1595, 3: 3036, 4: 1242}

    angles_rad = {servo_id: servo_to_rad(angle) for servo_id, angle in servo_angles.items()}
    print("Angles in radians:", {sid: f"{rad:.2f}" for sid, rad in angles_rad.items()})

    x, y, z, t3 = forward_kinematics(
            angles_rad[1],  # podstawa
            angles_rad[2],  # ramię 1
            angles_rad[3],  # ramię 2
            angles_rad[4],  # ramię 3
        )
    print(f"Pozycja efektora: x={x:.2f}, y={y:.2f}, z={z:.2f}, t3 angle={t3:.2f}")

    point = (x, y, z)
    alpha = t3 + math.radians(90)  # kąt nachylenia w radianach
    
    x0, y0, z0 = point
    
    if x0 != 0 or y0 != 0:
        # Obliczamy kąt azymutalny (kierunek w płaszczyźnie XY)
        azimuth = math.atan2(y0, x0)
        
        # Ruch w płaszczyźnie XZ: przesuwamy się pod kątem alpha
        dx_horizontal = math.cos(alpha) * step  # składowa pozioma
        dz = math.sin(alpha) * step             # składowa pionowa
        
        # Rozkładamy składową poziomą na osie X i Y zachowując azymut
        dx = dx_horizontal * math.cos(azimuth)
        dy = dx_horizontal * math.sin(azimuth)
        
        x = x0 + dx
        y = y0 + dy
        z = z0 + dz
        
    else:
        # Punkt leży na osi Z - ruch czysto w płaszczyźnie XZ
        x = step * math.cos(alpha)
        y = 0
        z = z0 + step * math.sin(alpha)
    
    print(f"Punkt początkowy: ({x0:.2f}, {y0:.2f}, {z0:.2f})")
    print(f"Kąt α: {alpha:.2f} rad ({math.degrees(alpha):.1f}°)")
    print(f"Punkt końcowy: ({x:.2f}, {y:.2f}, {z:.2f})")
