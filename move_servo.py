from st3215 import ST3215
servo = ST3215('/dev/ttyACM0')

servo.MoveTo(3, 3072, 200, 150)