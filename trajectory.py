import time
import math
from utilis import move_to_point, trajectory

def one():
    start = (180, 0, 115)    
    end = (180, 0, 115)
    x = 50
    move_to_point(start, max_speed=1200, acc=5, wait=True, theta4_desired=math.radians(x))

    for i in range(1):
        trajectory(
            *start, math.radians(x),   
            *end, -math.radians(x),     
            steps=100,
            max_speed=1000,
            acc=100
        )
        trajectory(
            *end, -math.radians(x),     
            *start, math.radians(x),   
            steps=100,
            max_speed=1000,
            acc=100
        )

one()