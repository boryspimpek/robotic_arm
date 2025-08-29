from st3215 import ST3215
servo = ST3215('/dev/ttyACM0')

# servo.MoveTo(4, 2048, 200, 150)

# Przykład użycia zsynchronizowanego ruchu
servo_targets = {
    1: 2048,   # Serwo ID 1: docelowa pozycja 1024
    2: 2048,   # Serwo ID 2: docelowa pozycja 2048  
    3: 2048     # Serwo ID 3: docelowa pozycja 512
}

# Wszystkie serwa dotrą do celu w tym samym momencie
result = servo.MoveManyTo(servo_targets, max_speed=500, acc=60, wait=True)