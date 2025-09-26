import time
import math
from utilis import servo_positions, move_to_point, trajectory, servo_to_rad
from ik import forward_kinematics
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




######################## SET MODE

# servo.SetMode(1, 0)
# time.sleep(1)
# mode = servo.ReadMode(1)
# print(mode)


######################## Move single servo
# servo.MoveTo(1, 0, speed = 1000, acc = 20, wait = True)
# time.sleep(1)
# print(position := servo.ReadPosition(1))

# while True:
#     servo.MoveTo(1, 0, speed = 2000, acc = 20, wait = True)
#     time.sleep(1)
#     print(position := servo.ReadPosition(1))
#     time.sleep(1)
#     servo.MoveTo(1, 4000, speed = 2000, acc = 20, wait = True)
#     time.sleep(1)
#     print(position := servo.ReadPosition(1))
#     time.sleep(1)



###################################################### Zatrzymanie serwa
servo.StopServo(1)



####################### Move all
# servo_targets = {
#     1 : 1449,
#     2 : 1595,
#     3 : 3036,
#     4 : 1242
# }
# servo.SyncMoveTo(servo_targets, 500, 50, wait=True)


###################### Print servo angles and x, y, z, and 3 arm orientation to the ground
# servo_angles = servo_positions()
# print(servo_angles) # wynik w takiej postaci {1: 1449, 2: 1595, 3: 3036, 4: 1242}

# angles_rad = {servo_id: servo_to_rad(angle) for servo_id, angle in servo_angles.items()}
# print("Kąty w radianach:", angles_rad)

# x, y, z, t3 = forward_kinematics(
#         angles_rad[1],  # podstawa
#         angles_rad[2],  # ramię 1
#         angles_rad[3],  # ramię 2
#         angles_rad[4],  # ramię 3
#     )
# print(f"Pozycja efektora: x={x:.2f}, y={y:.2f}, z={z:.2f}, t3 angle={t3:.2f}")






########################### Move to point
# point = (168, 220, 31)
# move_to_point(point, max_speed=500, acc=10, wait=True, theta4_desired=0.39)




########################## Test trajectory
# start = (0, 0, 340)    
# end = (0, 0, 115)
# move_to_point(start, max_speed=1200, acc=5, wait=True, theta4_desired=math.radians(90))

# for i in range(1):
#     trajectory(
#         *start, math.radians(90),   
#         *end, -math.radians(0),     
#         steps=100,
#         max_speed=1000,
#         acc=100
#     )
#     trajectory(
#         *end, -math.radians(0),     
#         *start, math.radians(90),   
#         steps=100,
#         max_speed=1000,
#         acc=100
#     )

