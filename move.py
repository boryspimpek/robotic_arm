import time
import math
from math import cos, pi, sin

import numpy as np

from st3215 import ST3215

from utilis import servo_positions
from config import home

servo = ST3215('/dev/ttyACM0')




def go_home():
    servo.SyncMoveTo(home, max_speed=500)

