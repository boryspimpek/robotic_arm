import json
import time
from st3215 import ST3215
from scservo_sdk import gripper  # import funkcji gripper

FILE_NAME = "servo_positions.json"

servo = ST3215('/dev/ttyACM0')
ids = [1, 2, 3, 4]

# Wczytujemy pozycje z pliku
try:
    with open(FILE_NAME, "r") as f:
        all_positions = json.load(f)
except FileNotFoundError:
    print(f"Nie znaleziono pliku {FILE_NAME}")
    exit(1)

if not all_positions:
    print("Plik jest pusty lub nie zawiera pozycji.")
    exit(1)

print(f"Odtwarzanie {len(all_positions)} zestawów pozycji...")

for i, entry in enumerate(all_positions, start=1):
    targets = entry["servos"]           # pozycje serw
    g_state = entry.get("gripper", "closed")  # stan grippera (domyślnie closed)

    # Konwersja kluczy na int
    targets_int = {int(k): v for k, v in targets.items()}

    print(f"\nPozycja {i}: {targets_int}, gripper: {g_state}")

    # Ruch serw
    servo.SyncMoveTo(targets_int, 500, 50, wait=True)

    # Ustawienie grippera
    if g_state == "open":
        gripper("open")
    else:
        gripper("close")

    time.sleep(0.5)

print("\nOdtwarzanie zakończone!")
