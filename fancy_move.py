import pygame
import time
import math
import numpy as np
from st3215 import ST3215
from math import radians, degrees

# -------------------------
# Inicjalizacja serw i robota
# -------------------------
servo = ST3215('/dev/ttyACM0')

l1, l2, l3 = 120, 120, 110

# -------------------------
# Funkcje konwersji kątów
# -------------------------
def rad_to_servo(rad):
    center = 2048
    scale = 2048 / 3.1415926535
    return 4095 - int(round(rad * scale + center))

def servo_to_rad(raw_position):
    center = 2048
    scale = 2048 / 3.1415926535
    return ((4095 - raw_position) - center) / scale

# -------------------------
# Funkcja FK (Forward Kinematics)
# -------------------------
def forward_kinematics(theta1, theta2, theta3, theta4):
    # Pozycja podstawy
    x0, y0, z0 = 0, 0, 0
    
    # Pozycja stawu 1 (ramię)
    x1 = x0
    y1 = y0
    z1 = z0
    
    # Pozycja stawu 2 (łokieć)
    x2 = x1 + l1 * math.cos(theta2) * math.cos(theta1)
    y2 = y1 + l1 * math.cos(theta2) * math.sin(theta1)
    z2 = z1 + l1 * math.sin(theta2)
    
    # Pozycja stawu 3 (nadgarstek)
    x3 = x2 + l2 * math.cos(theta2 + theta3) * math.cos(theta1)
    y3 = y2 + l2 * math.cos(theta2 + theta3) * math.sin(theta1)
    z3 = z2 + l2 * math.sin(theta2 + theta3)
    
    # Pozycja końcówki
    x4 = x3 + l3 * math.cos(theta2 + theta3 + theta4) * math.cos(theta1)
    y4 = y3 + l3 * math.cos(theta2 + theta3 + theta4) * math.sin(theta1)
    z4 = z3 + l3 * math.sin(theta2 + theta3 + theta4)
    
    return (x4, y4, z4), (x3, y3, z3), (x2, y2, z2)

# -------------------------
# Funkcja IK z redundancją
# -------------------------
def solve_ik_with_redundancy(x_target, y_target, z_target, current_angles, cost_mode="normal", redundancy_param=0.0):
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
    
    # Aktualna konfiguracja (do uwzględnienia w koszcie)
    theta2_curr, theta3_curr, theta4_curr = current_angles[1], current_angles[2], current_angles[3]

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

        if cost_mode == "vertical_up":
            orientation_error = abs(end_orientation - math.pi/2)
            cost = orientation_error**2
        elif cost_mode == "vertical_down":
            orientation_error = abs(end_orientation + math.pi/2)
            cost = orientation_error**2
        elif cost_mode == "flat":
            orientation_error = abs(end_orientation)
            cost = orientation_error**2
        else:
            # Koszt uwzględniający redundancję - minimalizacja zmiany kątów
            cost = (theta2 - theta2_curr)**2 + (theta3 - theta3_curr)**2 + (theta4 - theta4_curr)**2
            orientation_error = 0.0

        # Dodajemy komponent redundancji - preferujemy rozwiązania z większym kątem theta3
        cost -= redundancy_param * theta3
        
        if cost < min_cost:
            min_cost = cost
            best_angles = (theta1, theta2, theta3, theta4)
            best_orientation_error = orientation_error

    if best_angles is None:
        raise ValueError("Brak rozwiązania IK dla tej pozycji")

    if best_orientation_error > tol:
        raise ValueError(f"Orientacja '{cost_mode}' poza zakresem ({degrees(best_orientation_error):.2f}°)")

    return best_angles

# -------------------------
# Funkcja ruchu z redundancją
# -------------------------
def move_with_redundancy(point, redundancy_param=0.0):
    x, y, z = point
    ids = [1, 2, 3, 4]

    current_angles = [servo_to_rad(servo.ReadPosition(id)) for id in ids]

    angles = solve_ik_with_redundancy(x, y, z, current_angles, "normal", redundancy_param)

    delta_angles = [abs(target - current) for target, current in zip(angles, current_angles)]

    max_delta = max(delta_angles)
    servo_speeds = [int((delta / max_delta) * 2400) if max_delta != 0 else 0 for delta in delta_angles]

    servo_targets = [rad_to_servo(angle) for angle in angles]

    for id, target, speed in zip(ids, servo_targets, servo_speeds):
        servo.MoveTo(id, target, speed, 150)
        
    return angles

# -------------------------
# Funkcja do efektownego ruchu bez przemieszczania końcówki
# -------------------------
def fancy_move(redundancy_range=2.0, steps=20, delay=0.1):
    """Wykonuje efektowny ruch bez przemieszczania końcówki"""
    # Pobierz aktualne kąty
    ids = [1, 2, 3, 4]
    current_angles = [servo_to_rad(servo.ReadPosition(id)) for id in ids]
    
    # Oblicz aktualną pozycję końcówki
    current_pos, wrist_pos, elbow_pos = forward_kinematics(*current_angles)
    
    print(f"Pozycja początkowa: {current_pos}")
    
    # Wykonaj serię ruchów z różnymi parametrami redundancji
    for i in range(steps + 1):
        # Zmieniaj parametr redundancji sinusoidalnie
        param = redundancy_range * math.sin(2 * math.pi * i / steps)
        
        try:
            new_angles = move_with_redundancy(current_pos, param)
            new_pos, new_wrist, new_elbow = forward_kinematics(*new_angles)
            
            # Sprawdź, czy końcówka się nie przemieściła
            error = math.sqrt(
                (new_pos[0] - current_pos[0])**2 +
                (new_pos[1] - current_pos[1])**2 +
                (new_pos[2] - current_pos[2])**2
            )
            
            print(f"Krok {i}: redundancja={param:.2f}, błąd pozycji={error:.2f}mm")
            
            time.sleep(delay)
        except Exception as e:
            print(f"Błąd w kroku {i}: {e}")
            break

# -------------------------
# Inicjalizacja Pygame i pad PS4
# -------------------------
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise Exception("Nie wykryto pada PS4!")
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Pad wykryty: {joystick.get_name()}")

# -------------------------
# Początkowa pozycja końcówki
# -------------------------
x, y, z = 200, 0, 0
step = 5.0
deadzone = 0.8

move_with_redundancy((x, y, z))
time.sleep(2)

try:
    while True:
        pygame.event.pump()

        ly = joystick.get_axis(0)
        lx = joystick.get_axis(1)
        ry = joystick.get_axis(4)
        
        # Sprawdź przyciski dla efektownych ruchów
        triangle_btn = joystick.get_button(2)  # Trójkąt

        # deadzone
        lx = 0 if abs(lx) < deadzone else lx
        ly = 0 if abs(ly) < deadzone else ly
        ry = 0 if abs(ry) < deadzone else ry
        
        # ruch XYZ
        x += -lx * step
        y -= ly * step
        z -= ry * step

        # ruch robota
        try:
            if triangle_btn:
                # Naciśnięty trójkąt - wykonaj efektowny ruch
                fancy_move(redundancy_range=2.0, steps=30, delay=0.05)
            else:
                # Normalny ruch
                move_with_redundancy((x, y, z))
                
            print(f"Ruch do punktu: x={x:.1f}, y={y:.1f}, z={z:.1f}")
        except ValueError as e:
            print(f"Nieosiągalna pozycja: {e}")

        time.sleep(0.001)

except KeyboardInterrupt:
    print("Sterowanie zakończone")