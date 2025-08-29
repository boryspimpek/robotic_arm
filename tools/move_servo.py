from advance import solve_ik_full
from st3215 import ST3215
from utilis import INITIAL_POSITION, rad_to_servo, servo_to_rad
servo = ST3215('/dev/ttyACM0')

# servo.MoveTo(4, 2048, 200, 150)

# # Przykład użycia zsynchronizowanego ruchu
# servo_targets = {
#     1: 2048,   # Serwo ID 1: docelowa pozycja 1024
#     2: 2048,   # Serwo ID 2: docelowa pozycja 2048  
#     3: 2048     # Serwo ID 3: docelowa pozycja 512
# }

# # Wszystkie serwa dotrą do celu w tym samym momencie
# result = servo.MoveManyTo(servo_targets, max_speed=500, acc=60, wait=True)

servo_ids = [1, 2, 3, 4]

current_position = INITIAL_POSITION
x, y, z = current_position

angles = solve_ik_full(x, y, z)
servo_targets = [rad_to_servo(angle) for angle in angles]
print(servo_targets)

targets = {servo_id: servo_targets[i] for i, servo_id in enumerate(servo_ids)}
print(targets)
