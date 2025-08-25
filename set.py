from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')



# servo.MoveTo(4, 397, 500, 50)

# servo.DefineMiddle(2)

ids = [1, 2, 3, 4]

for id in ids:
    current_angles = servo.ReadPosition(id)
    print(f"{current_angles}")


