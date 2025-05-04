# robot_arm/config.py

L1 = 120  # mm, długość pierwszego ramienia
L2 = 120  # mm, długość drugiego ramienia

##### ST Servos ####
SERVO_BASE_ID = 1       # obrót podstawy
SERVO_SHOULDER_ID = 2   # ramię nr 1
SERVO_ELBOW_ID = 3      # ramię nr 2
SERVO_WRIST_ID = 4
#### SC Servos ####
SERVO_GRIPPER_ID = 5

UART_PORT = "/dev/ttyACM0"
BAUDRATE = 1000000

SERVO_MID_ANGLE = 90    # pozycja pionowa serwa
GLOBAL_SERVO_SPEED = 500  # surowa prędkość dla WritePosEx()
SC_SERVO_SPEED = 500      # prędkość dla SCServo
GLOBAL_SERVO_ACC = 25      # surowa akceleracja dla WritePosEx()
SC_SERVO_ACC = 25         # akceleracja dla SCServo
SERVO_ANGLE_LIMITS = (0, 180)  # dozwolony zakres dla każdego serwa

SERVO_TRIMS = {
    SERVO_BASE_ID: 0,
    SERVO_SHOULDER_ID: 0,  # przykład korekty (możesz zmienić)
    SERVO_ELBOW_ID: 0,
    SERVO_WRIST_ID: 0
}
