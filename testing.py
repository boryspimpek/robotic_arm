import time
import math
from utilis import servo_positions, move_to_point, trajectory
from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')


########## SETTING SECTION #################

######################## List servos
# print(servos := servo.ListServos())

######################## Ping servo
# print(alive := servo.PingServo(3))

######################## Change ID os servo
# servo.ChangeId(1, 2)

######################## Define middle point
# servo.DefineMiddle(4)



############ CHECKING SECTION ###############

######################## Move single servo
# servo.MoveTo(4, 2048, speed = 500, acc = 50, wait = True)
# print(position := servo.ReadPosition(4))

####################### Move all
# servo_targets = {
#     1 : 2048,
#     2 : 1500,
#     3 : 2500,
#     4 : 2048
# }
# servo.SyncMoveTo(servo_targets, 500, 50, wait=True)

###################### Print servo angles
# print(servo_angles := servo_positions())

########################### Move to point
# point = (200, 0, 118)
# move_to_point(point, max_speed=1000, acc=5, wait=True, theta4_desired=-math.radians(30))

########################## Test trajectory
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

