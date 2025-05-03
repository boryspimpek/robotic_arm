from STservo_sdk.port_handler import PortHandler
from STservo_sdk.sts import sts

# Ustawienia portu i prędkości transmisji
DEVICENAME = "/dev/ttyACM0"  # lub odpowiedni port, np. 'COM3' na Windowsie
BAUDRATE = 1000000

# Aktualne ID serwa i nowe ID, które chcemy ustawić
old_id = 1
new_id = 10

# Inicjalizacja portu
portHandler = PortHandler(DEVICENAME)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# Inicjalizacja obiektu dla serwa
servo = sts(portHandler)

# Odblokowanie EEPROM
servo.unLockEprom(old_id)

# Zmiana ID serwa
STS_ID_ADDRESS = 5  # Adres rejestru ID w pamięci serwa
servo.write1ByteTxRx(old_id, STS_ID_ADDRESS, new_id)

# Zablokowanie EEPROM
servo.LockEprom(new_id)

print(f"ID serwa zostało zmienione z {old_id} na {new_id}")

# Zamknięcie portu
portHandler.closePort()
