from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')



servo.MoveTo(4, 397, 500, 50)

# servo.DefineMiddle(2)