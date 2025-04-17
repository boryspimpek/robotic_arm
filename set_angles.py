from config import *
from servos import ServoController

# Ustawienia kątów w stopniach
base_angle = 90
shoulder_angle = 90
elbow_angle = 90

# Inicjalizacja kontrolera serw
servo = ServoController(UART_PORT)

# Wysłanie pozycji do serw
servo.move_to({
    SERVO_BASE_ID: base_angle,
    SERVO_SHOULDER_ID: shoulder_angle,
    SERVO_ELBOW_ID: elbow_angle
})

print(f"Ustawiono kąty: Baza={base_angle}°, Bark={shoulder_angle}°, Łokieć={elbow_angle}°")
