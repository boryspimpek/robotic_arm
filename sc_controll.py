from scservo_sdk import *
from config import sc_acc, sc_speed, gripper  # Używamy tej samej biblioteki SCServo SDK

# Parametry domyślne
BAUDRATE = 1000000           # Domyślny baudrate SCServo
DEVICENAME = '/dev/ttyACM0'  # Domyślny port, sprawdź na swoim systemie

# Adresy kontrolki SCServo
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56

# Inne parametry
SCS_MOVING_STATUS_THRESHOLD = 20  # Próg ruchu (określa, kiedy serwomechanizm zatrzymał się)
protocol_end = 1  # Protokół SCS (0 dla STS/SMS, 1 dla SCS)

# Funkcja do sterowania SCServo
def sc_servo(SCS_ID, position, speed, acceleration):
    # Inicjalizowanie portu
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(protocol_end)  # używamy protokołu SCS (0)

    # Otwórz port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        return
    
    # Ustawienie baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        return

    # Ustawienie przyspieszenia
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_ACC, acceleration)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    
    # Ustawienie prędkości
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, speed)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    
    # Ustawienie pozycji
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION, position)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    
    # Oczekiwanie na osiągnięcie pozycji
    while True:
        scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID, ADDR_SCS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        scs_present_position = SCS_LOWORD(scs_present_position_speed)
        scs_present_speed = SCS_HIWORD(scs_present_position_speed)
        # print(f"[ID:{SCS_ID}] GoalPos:{position} PresPos:{scs_present_position} PresSpd:{SCS_TOHOST(scs_present_speed, 15)}")

        if abs(position - scs_present_position) <= SCS_MOVING_STATUS_THRESHOLD:
            break
    
    # Wyłączenie momentu obrotowego po zakończeniu
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    # Zamknięcie portu
    portHandler.closePort()

def open_gripper():
    sc_servo(gripper, 800, sc_speed, sc_acc)

def close_gripper():
    sc_servo(gripper, 500, sc_speed, sc_acc)

def sc_torque_off():
    # Inicjalizowanie portu
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(protocol_end)  # używamy protokołu SCS (0)

    # Otwórz port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        return
    
    # Ustawienie baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        return

    # Wyłączenie momentu obrotowego
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, 5, ADDR_SCS_TORQUE_ENABLE, 0)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    # Zamknięcie portu
    portHandler.closePort()

def sc_servo_position(SCS_ID):
    # Inicjalizacja portu
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(protocol_end)

    # Otwórz port
    if not portHandler.openPort():
        print("[Błąd] Nie udało się otworzyć portu")
        return None

    # Ustaw baudrate
    if not portHandler.setBaudRate(BAUDRATE):
        print("[Błąd] Nie udało się ustawić baudrate")
        portHandler.closePort()
        return None

    # Odczyt pozycji
    scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID, ADDR_SCS_PRESENT_POSITION)
    if scs_comm_result != COMM_SUCCESS:
        print(f"[Błąd komunikacji] {packetHandler.getTxRxResult(scs_comm_result)}")
        portHandler.closePort()
        return None
    elif scs_error != 0:
        print(f"[Błąd pakietu] {packetHandler.getRxPacketError(scs_error)}")
        portHandler.closePort()
        return None

    # Rozbij wynik na pozycję
    scs_present_position = SCS_LOWORD(scs_present_position_speed)

    # Zamknij port
    portHandler.closePort()

    # Wypisz wynik
    print(f"[ID:{SCS_ID}] Aktualna pozycja: {scs_present_position}")


    return scs_present_position  
