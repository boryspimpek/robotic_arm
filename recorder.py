# recorder.py

import time
import json
from config import SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID
from servos import ServoController
from controller import ArmController
from kinematics import Kinematics

POSES_FILE = "poses.json"
TEMPO_DPS = 30.0  # prędkość ruchu serw w °/s

def record_positions(ctrl):
    print("[TRYB NAGRYWANIA]")
    print("Ustawiaj serwa ręcznie. Naciśnij ENTER, aby zapisać pozycję. Q aby zakończyć.\n")

    ctrl.torque_off_all()
    poses = []

    while True:
        user_input = input(">> ")
        if user_input.lower() == 'q':
            break

        angles = ctrl.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID
        ])

        if len(angles) != 3:
            print("[BŁĄD] Nie udało się odczytać wszystkich serw.")
            continue

        poses.append({
            str(SERVO_BASE_ID): angles[SERVO_BASE_ID],
            str(SERVO_SHOULDER_ID): angles[SERVO_SHOULDER_ID],
            str(SERVO_ELBOW_ID): angles[SERVO_ELBOW_ID],
        })

        print(f"[ZAPISANO] {len(poses)}: "
              f"S1={angles[SERVO_BASE_ID]:.1f}°, "
              f"S2={angles[SERVO_SHOULDER_ID]:.1f}°, "
              f"S3={angles[SERVO_ELBOW_ID]:.1f}°")

    with open(POSES_FILE, "w") as f:
        json.dump(poses, f, indent=2)
    print(f"[OK] Zapisano {len(poses)} pozycji do pliku '{POSES_FILE}'.")

def playback(ctrl):
    print("[TRYB ODTWARZANIA]")

    try:
        with open(POSES_FILE, "r") as f:
            poses = json.load(f)
    except FileNotFoundError:
        print(f"[BŁĄD] Plik '{POSES_FILE}' nie istnieje.")
        return

    if not poses:
        print(f"[INFO] Lista pozycji jest pusta.")
        return

    ctrl.torque_on_all()

    # odczyt aktualnej pozycji jako punkt wyjściowy
    last_pose = ctrl.get_all_servo_positions_deg([
        SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID
    ])

    for idx, pose in enumerate(poses, start=1):
        # konwersja kluczy z string na int
        angles = {int(k): v for k, v in pose.items()}

        # wywołanie synchronicznego ruchu z dopasowaną prędkością
        ctrl.sync_angles(last_pose, angles, tempo_dps=TEMPO_DPS)

        # wyświetlenie informacji
        print(f"Poz. {idx}: "
              f"S1={angles[SERVO_BASE_ID]:.1f}°, "
              f"S2={angles[SERVO_SHOULDER_ID]:.1f}°, "
              f"S3={angles[SERVO_ELBOW_ID]:.1f}°")

        # obliczenie potrzebnego czasu ruchu na podstawie najdłuższego kąta
        delta = max(abs(angles[sid] - last_pose.get(sid, angles[sid])) for sid in angles)
        move_time = delta / TEMPO_DPS

        time.sleep(move_time + 0.1)

        last_pose = angles  # zapamiętaj aktualną jako następną startową

    print("[OK] Odtwarzanie zakończone.")


if __name__ == "__main__":
    servo_ctrl = ServoController(port_path="COM8")  # lub UART_PORT z config.py
    kin = Kinematics(100, 100)  # lub importuj L1, L2 z config
    controller = ArmController(kin, servo_ctrl)

    mode = input("Wybierz tryb ([N]agrywaj / [O]dtwarzaj): ").strip().lower()

    if mode == "n":
        record_positions(servo_ctrl)
    elif mode == "o":
        playback(servo_ctrl)
    else:
        print("[INFO] Nieznany tryb. Użyj N lub O.")
