import time
import math
from math import cos, pi, sin

import numpy as np
import pygame

from st3215 import ST3215
from scservo_sdk import gripper

from utilis import DEADZONE, INITIAL_POSITION
from utilis import move_to_point_2d, initialize_joystick, process_joystick_input_2d, go_home
from ik_2d import find_wrist_point_2d, solve_ik_full_2d

TRIANGLE_BUTTON_ID = 2
CIRCLE_BUTTON_ID = 1
CROSS_BUTTON_ID = 0
SQUARE_BUTTON_ID = 3
R1_BUTTON_ID = 5

method = "full"
orientation_mode = "flat"
gripper_state = "close"

servo = ST3215('/dev/ttyACM0')

def main():
    global method, orientation_mode, gripper_state
    step = 2 if method == "wrist" else 5
    joystick = initialize_joystick()
    current_position = (INITIAL_POSITION[0], INITIAL_POSITION[2])
    button_states = {TRIANGLE_BUTTON_ID: 0, CIRCLE_BUTTON_ID: 0, CROSS_BUTTON_ID: 0, SQUARE_BUTTON_ID:0, R1_BUTTON_ID:0}
    
    base_position = 2048  # Integer initial value
    base_speed = 500  
    last_time = time.time()
    
    move_to_point_2d(current_position, method, orientation_mode, base_position, max_speed=500)
    
    try:
        while True:
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time
            
            triangle, circle, cross, square, r1 = (joystick.get_button(btn) for btn in [TRIANGLE_BUTTON_ID, CIRCLE_BUTTON_ID, CROSS_BUTTON_ID, SQUARE_BUTTON_ID, R1_BUTTON_ID])
            
            if triangle == 1 and button_states[TRIANGLE_BUTTON_ID] == 0:
                orientation_mode = "down"
                move_to_point_2d(current_position, method, orientation_mode, base_position)
            
            if circle == 1 and button_states[CIRCLE_BUTTON_ID] == 0:
                orientation_mode = "flat"
                move_to_point_2d(current_position, method, orientation_mode, base_position)
            
            if cross == 1 and button_states[CROSS_BUTTON_ID] == 0:
                method = "wrist" if method == "full" else "full"
                step = 3 if method == "wrist" else 5
                if method == "wrist":
                    wrist_point = find_wrist_point_2d(solve_ik_full_2d(*current_position))
                    move_to_point_2d(wrist_point, "wrist", orientation_mode, base_position)
                    current_position = wrist_point
                print(f"Zmieniono tryb na: {method}, step = {step}")
            
            # if square == 1 and button_states[SQUARE_BUTTON_ID] == 0:
            #     go_home()
            
            if r1 == 1 and button_states[R1_BUTTON_ID] == 0:
                if gripper_state == "close":
                    gripper("open")
                    gripper_state = "open"
                    print("Chwytak otwarty")
                else:
                    gripper("close")
                    gripper_state = "close"
                    print("Chwytak zamknięty")

            button_states = {TRIANGLE_BUTTON_ID: triangle, CIRCLE_BUTTON_ID: circle, CROSS_BUTTON_ID: cross, SQUARE_BUTTON_ID: square, R1_BUTTON_ID: r1}
            
            new_position, rotation_input = process_joystick_input_2d(joystick, current_position, step)
            
            needs_move = False
            
            if abs(rotation_input) > DEADZONE:
                # Convert to integer after calculation to avoid floating-point issues
                base_position += int(rotation_input * base_speed * delta_time)
                base_position = max(0, min(4095, base_position))
                needs_move = True
            
            if new_position != current_position or needs_move:
                try:
                    # base_position is already an integer now
                    move_to_point_2d(new_position, method, orientation_mode, base_position)
                    current_position = new_position
                    print(f"Position: ({current_position[0]:.2f}, {current_position[1]:.2f}), Base: {base_position}, Method: {method}")
                
                except ValueError as e:
                    print(f"Nieosiągalna pozycja: {e}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Sterowanie zakończone")        

if __name__ == "__main__":
    main()