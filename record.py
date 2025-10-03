import os
import json
import time
from utilis import servo_positions
from st3215 import ST3215
from scservo_sdk import gripper

FILE_NAME = "servo_positions.json"

servo = ST3215('/dev/ttyACM0')
ids = [1, 2, 3, 4]

# 1. Wyłączamy torque
for id_ in ids:
    servo.StopServo(id_)

# 2. Jeśli plik istnieje, zaczynamy od pustego
if os.path.exists(FILE_NAME):
    with open(FILE_NAME, "w") as f:
        json.dump([], f)

print("Ustaw serwa ręcznie. "
      "Naciśnij Enter, aby zapisać pozycje, 'o' + Enter aby otworzyć gripper, "
      "'c' + Enter aby zamknąć gripper, lub 'q' + Enter, by zakończyć.")

all_positions = []
gripper_state = "closed"  # domyślnie zamknięty

while True:
    user_input = input().strip().lower()

    if user_input == "q":
        break
    elif user_input == "o":
        gripper("open")
        gripper_state = "open"
        print("Gripper otwarty.")
        continue
    elif user_input == "c":
        gripper("close")
        gripper_state = "closed"
        print("Gripper zamknięty.")
        continue

    # Zapisujemy pozycję serw wraz ze stanem grippera
    pos = servo_positions()
    print(f"Odczytano pozycje: {pos}, gripper: {gripper_state}")

    all_positions.append({"servos": pos, "gripper": gripper_state})

    with open(FILE_NAME, "w") as f:
        json.dump(all_positions, f, indent=2)

    print("Pozycje zapisane. Ustaw kolejną pozę albo wpisz 'q' żeby skończyć.")

print(f"Zapisano do pliku {FILE_NAME}.")

# ✅ Po zakończeniu pytamy o odtworzenie
choice = input("Czy odtworzyć zapisane pozycje? (t/n): ").strip().lower()

if choice == "t":
    print("Odtwarzam zapisane pozycje...")

    for i, entry in enumerate(all_positions, start=1):
        targets = entry["servos"]
        g_state = entry["gripper"]
        print(f"Pozycja {i}: {targets}, gripper: {g_state}")
        
        # Poruszamy serwami
        servo.SyncMoveTo(targets, 500, 50, wait=True)
        
        # Ustawiamy gripper
        gripper("open" if g_state == "open" else "close")
        
        time.sleep(0.5)

    print("Odtwarzanie zakończone.")
else:
    print("Zakończono bez odtwarzania.")
