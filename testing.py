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
# point = (0, 0, 115)
# move_to_point(point, max_speed=1000, acc=10, wait=True, theta4_desired=None)

########################## Test trajectory
start = (0, 0, 340)    
end = (0, 0, 115)
move_to_point(start, max_speed=1200, acc=5, wait=True, theta4_desired=math.radians(90))

for i in range(1):
    trajectory(
        *start, math.radians(90),   
        *end, -math.radians(0),     
        steps=100,
        max_speed=1000,
        acc=100
    )
    trajectory(
        *end, -math.radians(0),     
        *start, math.radians(90),   
        steps=100,
        max_speed=1000,
        acc=100
    )

