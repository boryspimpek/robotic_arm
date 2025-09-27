import time
from st3215 import ST3215, ContinuousServoController


servo = ST3215('/dev/ttyACM0')
multi = ContinuousServoController(servo)


multi.rotate_by(1, 1024, 1000)
