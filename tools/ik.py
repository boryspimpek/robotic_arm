import math
import numpy as np
import time
from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')

l1 = 120
l2 = 120
l3 = 110

def solve_ik(x_target, y_target, z_target, cost_mode):
    delta_theta = np.radians(1)
    theta4_candidates = np.arange(-np.pi, np.pi, delta_theta)  

    r_target = np.hypot(x_target, y_target)
    px = r_target
    py = z_target

    d = math.hypot(px, py)
    if d > (l1 + l2 + l3):
        raise ValueError("Punkt poza zasięgiem manipulatora")

    theta1 = np.arctan2(y_target, x_target)  

    min_cost = np.inf
    best_angles = None
    best_orientation_error = np.inf

    # tolerancja orientacji końcówki (rad)
    tol = np.radians(2)   # np. ±2°

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

        end_orientation = theta2 + theta3 + theta4

        if cost_mode == "vertical_up":
            orientation_error = abs(end_orientation - np.pi/2)
            cost = orientation_error**2
        elif cost_mode == "vertical_down":
            orientation_error = abs(end_orientation + np.pi/2)
            cost = orientation_error**2
        elif cost_mode == "flat":
            orientation_error = abs(end_orientation)
            cost = orientation_error**2
        elif cost_mode == "normal":
            cost = (theta3)**2 + (theta4)**2
            orientation_error = 0.0
        else:
            raise ValueError("Nieznany tryb kosztu")
        
        if cost < min_cost:
            min_cost = cost
            best_angles = (theta1, theta2, theta3, theta4)
            best_orientation_error = orientation_error

    if best_angles is None:
        raise ValueError("Punkt jest w zasięgu, ale brak rozwiązań spełniających geometrię")

    # tu sprawdzamy orientację
    if best_orientation_error > tol:
        raise ValueError(f"Punkt osiągalny, ale orientacja '{cost_mode}' nie jest możliwa (odchyłka {np.degrees(best_orientation_error):.2f}°)")

    return best_angles

def rad_to_servo(rad):
    center = 2048
    scale = 2048 / math.pi
    raw_position = 4095 - int(round(rad * scale + center))
    return raw_position

def servo_to_rad(raw_position):
    center = 2048
    scale = 2048 / math.pi
    rad = ((4095 - raw_position) - center) / scale
    return rad

def move_to_point(point):
    x, y, z = point
    names = ["Base", "Shoulder", "Elbow", "Wrist"]
    ids = [1, 2, 3, 4]

    # odczyt aktualnych pozycji w radianach
    current_angles = [servo_to_rad(servo.ReadPosition(id)) for id in ids]

    print("Aktualne pozycje serw (radiany):")
    for name, angle in zip(names, current_angles):
        print(f"{name:10}: {angle:6.3f} rad, {angle*180/3.14159:6.1f}°")
    print("===============================")

    # obliczenie docelowych kątów
    angles = solve_ik(x, y, z, "normal")
    print("Docelowe pozycje (IK):")
    for name, angle in zip(names, angles):
        print(f"{name:10}: {angle:6.3f} rad, {angle*180/3.14159:6.1f}°")
    print("===============================")

    # różnica kątów
    delta_angles = [abs(target - current) for target, current in zip(angles, current_angles)]
    print("Różnice kątów do pokonania:")
    for name, delta in zip(names, delta_angles):
        print(f"{name:10}: {delta:6.3f} rad, {delta*180/3.14159:6.1f}°")
    print("===============================")

    # maksymalna różnica kąta
    max_delta = max(delta_angles)
    print(f"Maksymalna różnica kąta: {max_delta:.3f} rad, {max_delta*180/3.14159:.1f}°")
    print("===============================")

    # obliczenie prędkości dla każdego serwa
    servo_speeds = [int((delta / max_delta) * 200) if max_delta != 0 else 0 for delta in delta_angles]
    print("Prędkości serw:")
    for name, speed in zip(names, servo_speeds):
        print(f"{name:10}: {speed:4d}")
    print("===============================")

    # konwersja docelowych kątów na wartości serw
    servo_targets = [rad_to_servo(angle) for angle in angles]
    print("Docelowe wartości serw:")
    for name, target in zip(names, servo_targets):
        print(f"{name:10}: {target:4d}")
    print("===============================")

    # ruch serw zsynchronizowany
    for id, target, speed in zip(ids, servo_targets, servo_speeds):
        servo.MoveTo(id, target, speed, 150)
