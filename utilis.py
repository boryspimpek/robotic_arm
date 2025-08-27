import pygame


SERVO_LIMITS = {
    1: (1024, 3072),
    2: (0, 2048), 
    3: (400, 3700),
    4: (600, 3500)
}



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

def initialize_joystick():
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        raise Exception("Nie wykryto pada PS4!")
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"Pad wykryty: {joystick.get_name()}")
    return joystick
