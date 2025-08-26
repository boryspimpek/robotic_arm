from st3215 import ST3215
from ik import solve_ik, rad_to_servo, servo_to_rad
servo = ST3215('/dev/ttyACM0')


limits = {
    1: (1024, 3072),
    2: (0, 2048), 
    3: (400, 3700),
    4: (600, 3500)
}

def check_servo_angles(servo_targets):
    """Sprawdza czy kąty serw są w ich indywidualnych zakresach"""
    errors = []
    for id, target in zip([1, 2, 3, 4], servo_targets):
        min_angle, max_angle = limits[id]  # Pobierz limity dla konkretnego serwa
        if not (min_angle <= target <= max_angle):
            errors.append(f"Kąt serwa {id} poza zakresem ({min_angle}-{max_angle}): {target}")
    return errors

def move(point):
    x, y, z = point
    ids = [1, 2, 3, 4]

    current_angles = [servo_to_rad(servo.ReadPosition(id)) for id in ids]

    angles = solve_ik(x, y, z, "normal")

    delta_angles = [abs(target - current) for target, current in zip(angles, current_angles)]

    max_delta = max(delta_angles)
    servo_speeds = [int((delta / max_delta) * 500) if max_delta != 0 else 0 for delta in delta_angles]
    print("Prędkości serw:", servo_speeds)

    servo_targets = [rad_to_servo(angle) for angle in angles]

    errors = check_servo_angles(servo_targets)
    if errors:
        print("Błędy:", errors)
        return
    else:
        print("Brak błędów")

        for id, target, speed in zip(ids, servo_targets, servo_speeds):
            servo.MoveTo(id, target, speed, 150)


