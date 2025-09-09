import time
import math
from math import cos, pi, sin

import numpy as np
import pygame

from st3215 import ST3215

from utilis import l1, l2, l3,  DEADZONE, INITIAL_POSITION, process_joystick_input_2d 
from utilis import move_to_point_2d, initialize_joystick
from ik_2d import find_wrist_point_2d, solve_ik_full_2d

TRIANGLE_BUTTON_ID = 2
CIRCLE_BUTTON_ID = 1
CROSS_BUTTON_ID = 0

method = "full"
orientation_mode = "down"

servo = ST3215('/dev/ttyACM0')

def main():
    global method, orientation_mode
    step = 2 if method == "wrist" else 5
    joystick = initialize_joystick()
    current_position = (INITIAL_POSITION[0], INITIAL_POSITION[2])
    button_states = {TRIANGLE_BUTTON_ID: 0, CIRCLE_BUTTON_ID: 0, CROSS_BUTTON_ID: 0}
    
    move_to_point_2d(current_position, method, orientation_mode, max_speed=500)
    
    try:
        while True:
            triangle, circle, cross = (joystick.get_button(btn) for btn in [TRIANGLE_BUTTON_ID, CIRCLE_BUTTON_ID, CROSS_BUTTON_ID])
            
            if triangle == 1 and button_states[TRIANGLE_BUTTON_ID] == 0:
                orientation_mode = "down"
                move_to_point_2d(current_position, method, orientation_mode)
            
            if circle == 1 and button_states[CIRCLE_BUTTON_ID] == 0:
                orientation_mode = "flat"
                move_to_point_2d(current_position, method, orientation_mode)
            
            if cross == 1 and button_states[CROSS_BUTTON_ID] == 0:
                method = "wrist" if method == "full" else "full"
                step = 3 if method == "wrist" else 5
                if method == "wrist":
                    wrist_point = find_wrist_point_2d(solve_ik_full_2d(*current_position))
                    move_to_point_2d(wrist_point, "wrist", orientation_mode)
                    current_position = wrist_point
                print(f"Zmieniono tryb na: {method}, step = {step}")

            button_states = {TRIANGLE_BUTTON_ID: triangle, CIRCLE_BUTTON_ID: circle, CROSS_BUTTON_ID: cross}
            
            new_position = process_joystick_input_2d(joystick, current_position, step)
            
            if new_position != current_position:
                try:
                    move_to_point_2d(new_position, method, orientation_mode)
                    current_position = new_position
                    print(f"Position: ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f}), Method: {method}, Orientation: {orientation_mode}, Step: {step}")
                
                except ValueError as e:
                    print(f"Nieosiągalna pozycja: {e}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Sterowanie zakończone")

if __name__ == "__main__":
    main()