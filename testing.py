import time
import math
from utilis import servo_positions, move_to_point, trajectory, servo_to_rad
from ik import forward_kinematics
from st3215 import ST3215, ContinuousServoController


servo = ST3215('/dev/ttyACM0')
multi = ContinuousServoController(servo)

###################################################### SETTING SECTION #################

######################## List servos
# print(servos := servo.ListServos())

######################## Ping servo
# print(alive := servo.PingServo(1))

######################## Change ID os servo
# servo.ChangeId(1, 2)

####################### Define middle point
# servo.DefineMiddle(1)
# time.sleep(5)
# print(position := servo.ReadPosition(1))



###################################################### SET MODE #############################

# servo.SetMode(1, 0)
# time.sleep(1)
# mode = servo.ReadMode(1)
# print(mode)


###################################################### Move single servo #############################
# servo.MoveTo(1, 2000, speed = 1000, acc = 20, wait = True)
# time.sleep(1)
# print(position := servo.ReadPosition(1))

# while True:
#     servo.MoveTo(1, 1000, speed = 500, acc = 20, wait = True)
#     time.sleep(1)
#     print(position := servo.ReadPosition(1))
#     time.sleep(1)
#     servo.MoveTo(1, 3000, speed = 500, acc = 20, wait = True)
#     time.sleep(1)
#     print(position := servo.ReadPosition(1))
#     time.sleep(1)


###################################################### MULTI ROTATE ####################################
# multi.rotate_by(1, -500, 1000)

###################################################### Zatrzymanie serwa ##############################
# servo.StopServo(1)



###################################################### Move all #######################################
# servo_targets = {
#     1 : 1449,
#     2 : 1595,
#     3 : 3036,
#     4 : 1242
# }
# servo.SyncMoveTo(servo_targets, 500, 50, wait=True)


###################################################### Print servo angles and x, y, z, and 3 arm orientation to the ground
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






##################################################### Move to point
# point1 = (100, 0, -60)
# point2 = (100, -100, -60)
# point3 = (100, 100, -60)
# for i in range(1):
#     move_to_point(point1, max_speed=500, acc=250, wait=True, theta4_desired=math.radians(-90))

#     move_to_point(point2, max_speed=500, acc=250, wait=True, theta4_desired=math.radians(-90))
#     time.sleep(2.5)
#     move_to_point(point3, max_speed=500, acc=250, wait=True, theta4_desired=math.radians(-90))
#     time.sleep(2.5)

#     move_to_point(point1, max_speed=500, acc=250, wait=True, theta4_desired=math.radians(-90))



# point1 = (150, 100, -60)

# move_to_point(point1, max_speed=500, acc=250, wait=True, theta4_desired=math.radians(-90))


##################################################### Test trajectory
start = (100, -100, -60)    
end = (100, 100, -60)
theta4_desired = math.radians(-90)
steps = 100
max_speed = 1000
acc = 250
move_to_point(start, max_speed=200, acc=200, wait=True, theta4_desired=theta4_desired)
print("STARTING POINT")
time.sleep(1)

for i in range(1):
    trajectory(
        *start, theta4_desired,   
        *end, theta4_desired,     
        steps=steps,
        max_speed=max_speed,
        acc=acc
    )
    trajectory(
        *end, theta4_desired,     
        *start, theta4_desired,   
        steps=steps,
        max_speed=max_speed,
        acc=acc
    )

