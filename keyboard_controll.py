import keyboard
from controller import ArmController
from kinematics import Kinematics
from servos import ServoController
from config import L1, L2, UART_PORT

# Inicjalizacja modułów
kin = Kinematics(L1, L2)
servo = ServoController(UART_PORT)
arm = ArmController(kin, servo)

# Początkowa pozycja
x, y, z = 142.0, 0.0, 0.0
step = 2.0  # mm

# Przejście do pozycji startowej
print("[INFO] Przechodzę do pozycji startowej...")
arm.move_to_point((x, y, z), steps=100)

print("Sterowanie:")
print("  W/S = oś X przód/tył")
print("  A/D = oś Y lewo/prawo")
print("  Q/E = oś Z góra/dół")
print("  ESC = wyjście")

while True:
    dx = dy = dz = 0.0

    if keyboard.is_pressed('w'):
        dx = step
    elif keyboard.is_pressed('s'):
        dx = -step
    elif keyboard.is_pressed('a'):
        dy = -step
    elif keyboard.is_pressed('d'):
        dy = step
    elif keyboard.is_pressed('q'):
        dz = step
    elif keyboard.is_pressed('e'):
        dz = -step
    elif keyboard.is_pressed('esc'):
        print("Zamykanie...")
        break
    else:
        continue

    # Wyliczamy nową pozycję – nie zapisujemy jeszcze
    new_x = x + dx
    new_y = y + dy
    new_z = z + dz

    # Próbujemy wykonać ruch
    success = arm.move_to_point((new_x, new_y, new_z), steps=5)

    if success:
        x, y, z = new_x, new_y, new_z
        print(f"Nowa pozycja: x={x:.1f}, y={y:.1f}, z={z:.1f}")
    else:
        print("[INFO] Pozycja nieosiągalna – współrzędne nie zmienione.")
