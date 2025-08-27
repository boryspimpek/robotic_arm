# sc09_gripper.py
from scservo_sdk import *

DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
PROTOCOL = 1
MOVING_THRESHOLD = 20

ADDR_TORQUE_ENABLE     = 40
ADDR_GOAL_ACC          = 41
ADDR_GOAL_POSITION     = 42
ADDR_GOAL_SPEED        = 46
ADDR_PRESENT_POSITION  = 56

def gripper(state):
    SCS_ID = 5
    speed = 500
    acceleration = 50

    if state.lower() == "open":
        position = 800
    elif state.lower() == "close":
        position = 500
    else:
        raise ValueError("Niepoprawny stan. Użyj 'open' lub 'close'.")

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL)

    if not portHandler.openPort():
        raise SystemExit("Nie udało się otworzyć portu")
    if not portHandler.setBaudRate(BAUDRATE):
        raise SystemExit("Nie udało się ustawić baudrate")

    packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_GOAL_ACC, acceleration)
    packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_GOAL_SPEED, speed)
    packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_GOAL_POSITION, position)

    while True:
        val, comm, err = packetHandler.read4ByteTxRx(portHandler, SCS_ID, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err != 0:
            continue
        current_pos = SCS_LOWORD(val)
        if abs(current_pos - position) <= MOVING_THRESHOLD:
            break

    packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
