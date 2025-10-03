import json
import time
from st3215 import ST3215

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

for i, targets in enumerate(all_positions, start=1):
    # ✅ Konwersja kluczy na int
    targets_int = {int(k): v for k, v in targets.items()}

    print(f"\nPozycja {i}: {targets_int}")
    servo.SyncMoveTo(targets_int, 500, 50, wait=True)
    time.sleep(0.5)

print("\nOdtwarzanie zakończone!")
