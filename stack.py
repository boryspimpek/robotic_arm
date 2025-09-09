import time
import math
from math import cos, pi, sin, hypot
import numpy as np

from scservo_sdk import gripper
from utilis import move_to_point

method = "wrist"
orientation_mode = "down"

home = (120, 0, 150)
speed_approach = 200
speed_move = 300
t = 0.1

def stack(pickup, target, bricks):
    x, y, z = pickup  # Pozycja podstawy stosu źródłowego
    a, b, c = target  # Pozycja podstawy stosu docelowego

    for i in range(bricks):
        # Obliczanie wysokości dla i-tej cegły w stosie źródłowym
        pickup_height = z + (bricks - i - 1) * 15
        
        # Obliczanie wysokości dla i-tej cegły w stosie docelowym
        target_height = c + i * 15

        # Podnieś się nad pozycję źródłową
        move_to_point((x, y, pickup_height + 40), method, orientation_mode, speed_move)
        time.sleep(t)

        # Otwórz chwytak przed opuszczeniem
        gripper("open")
        time.sleep(t)

        # Opuść się do pozycji chwytania
        move_to_point((x, y, pickup_height), method, orientation_mode, speed_approach)
        time.sleep(t)

        # Zamknij chwytak aby chwycić cegłę
        gripper("close")
        time.sleep(t)

        # Podnieś się z cegłą
        move_to_point((x, y, pickup_height + 40), method, orientation_mode, speed_approach)
        time.sleep(t)

        # Przenieś się nad pozycję docelową
        move_to_point((a, b, target_height + 40), method, orientation_mode, speed_move)
        time.sleep(t)

        # Opuść się do pozycji docelowej
        move_to_point((a, b, target_height), method, orientation_mode, speed_approach)
        time.sleep(t)

        # Otwórz chwytak aby uwolnić cegłę
        gripper("open")
        time.sleep(t)

        # Podnieś się bez cegły
        move_to_point((a, b, target_height + 40), method, orientation_mode, speed_approach)
        time.sleep(t)

        # Zamknij chwytak przed powrotem do domu
        gripper("close")
        time.sleep(t)

# Powrót do pozycji domowej
move_to_point(home, method, orientation_mode, speed_move)
time.sleep(0.5)

pickup = (150, 50, 0)  # Pozycja podstawy stosu źródłowego
place = (150, -50, 0)   # Pozycja podstawy stosu docelowego
bricks = 3             # Liczba cegieł do przeniesienia

stack(pickup, place, bricks)

# Powrót do pozycji domowej
move_to_point(home, method, orientation_mode, speed_move)
time.sleep(0.5)