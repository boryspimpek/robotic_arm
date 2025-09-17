import time
import math
from utilis import move_to_point, trajectory, go_home

def one():
    start = (170, 0, 115)    
    end = (170, 0, 115)
    x = 50
    move_to_point(start, max_speed=1200, acc=5, wait=True, theta4_desired=math.radians(x))

    trajectory(
        *start, math.radians(x),   
        *end, -math.radians(x),     
        steps=80,
        max_speed=1000,
        acc=100
    )
    trajectory(
        *end, -math.radians(x),     
        *start, math.radians(x),   
        steps=80,
        max_speed=1000,
        acc=100
    )

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


one()

# two()

go_home()