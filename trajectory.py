import time
import math
from utilis import move_to_point, trajectory, go_home, servo_positions, servo_to_rad, line_in_space
from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')



# Porusza ramieniem tak aby końcówka griperrwa pozostawała nie ruchomo
def one():
    start = (170, 0, 115)    
    end = (170, 0, 115)
    x = 50
    move_to_point(start, max_speed=1200, acc=5, wait=True, theta4_desired=math.radians(x))

    trajectory(
        *start, math.radians(x),   
        *end, -math.radians(x),     
        steps=200,
        max_speed=1000,
        acc=100
    )
    trajectory(
        *end, -math.radians(x),     
        *start, math.radians(x),   
        steps=200,
        max_speed=1000,
        acc=100
    )

# Przesuwa ramie góra dół w lini zmieniając orientacje grippera
def two():
    start = (170, 0, 294)    
    end = (170, 0, -70)
    x = 60
    move_to_point(start, max_speed=1200, acc=5, wait=True, theta4_desired=math.radians(x))

    trajectory(
        *start, math.radians(x),   
        *end, -math.radians(90),     
        steps=100,
        max_speed=1000,
        acc=100
    )
    trajectory(
        *end, -math.radians(90),     
        *start, math.radians(x),   
        steps=100,
        max_speed=1000,
        acc=100
    )

def three():
    line_in_space(step=100)
    end=(141.63, 216.25, 17.23)
    start=(118.59, 181.06, 107.96)
    t3 = 0.43

    move_to_point(start, max_speed=500, acc=5, wait=True, theta4_desired=t3)
    time.sleep(0.5)

    trajectory(
        *start, t3,   
        *end, t3,     
        steps=25,
        max_speed=1000,
        acc=100
    )
    trajectory(
        *end, t3,     
        *start, t3,   
        steps=25,
        max_speed=1000,
        acc=100
    )

def four():
    line_in_space(step=100)

    end=(160.90, 242.41, 12.12)
    start=(147.71, 222.54, 109.23)
    t3 = 0.24

    move_to_point(start, max_speed=500, acc=5, wait=True, theta4_desired=t3)
    # time.sleep(0.5)

    trajectory(
        *start, t3,   
        *end, t3,     
        steps=25,
        max_speed=1000,
        acc=100
    )
    trajectory(
        *end, t3,     
        *start, t3,   
        steps=25,
        max_speed=1000,
        acc=100
    )


time.sleep(2)
three()
four()
time.sleep(0.5)

servo_targets = {
    1 : 1398,
    2 : 432,
    3 : 3725,
    4 : 1733
}
servo.SyncMoveTo(servo_targets, 500, 50, wait=True)

