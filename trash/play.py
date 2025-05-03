# recorder.py

import time
import json
from config import SC_SERVO_ACC, SC_SERVO_SPEED, SERVO_BASE_ID, SERVO_GRIPPER_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
from servos import ServoController
from controller import ArmController
from kinematics import Kinematics
from sc_controll import open_gripper, close_gripper, sc_servo, sc_servo_position 

recorded_poses_FILE = "recorded_poses.json"
TEMPO_DPS = 50.0  # prędkość ruchu serw w °/s

with open("positions.json", "r") as f:
    predefined_positions = json.load(f)

def record_positions(ctrl):
    print("[TRYB NAGRYWANIA]")
    print("Ustawiaj serwa ręcznie.")
    print("ENTER - zapisz pozycję, O - otwórz chwytak, Z - zamknij chwytak, Q - zakończ\n")

    ctrl.torque_off_all()
    recorded_poses = []

    while True:
        user_input = input(">> ").lower()

        if user_input == 'q':
            break

        elif user_input in predefined_positions:
            point = predefined_positions[user_input]
            controller.move_to_point_dps(point)
            ctrl.torque_off_all()
            print(f"[INFO] Przejście do pozycji: {user_input.upper()} {point}")
            continue

        elif user_input == 'o':
            open_gripper()
            print("[INFO] Chwytak otwarty.")
            continue

        elif user_input == 'z':
            close_gripper()
            print("[INFO] Chwytak zamknięty.")
            continue

        elif user_input == '':
            angles = ctrl.get_all_servo_positions_deg([
                SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
            ])
            sc_angle = sc_servo_position(SERVO_GRIPPER_ID)

            if len(angles) != 4:
                print("[BŁĄD] Nie udało się odczytać wszystkich serw.")
                continue

            recorded_poses.append({
                str(SERVO_BASE_ID): int(angles[SERVO_BASE_ID]),
                str(SERVO_SHOULDER_ID): int(angles[SERVO_SHOULDER_ID]),
                str(SERVO_ELBOW_ID): int(angles[SERVO_ELBOW_ID]),
                str(SERVO_WRIST_ID): int(angles[SERVO_WRIST_ID]),
                str(SERVO_GRIPPER_ID): int(sc_angle)
            })

            print(f"[ZAPISANO] {len(recorded_poses)}: "
                f"S1={angles[SERVO_BASE_ID]:.1f}°, "
                f"S2={angles[SERVO_SHOULDER_ID]:.1f}°, "
                f"S3={angles[SERVO_ELBOW_ID]:.1f}°, "
                f"S4={angles[SERVO_WRIST_ID]:.1f}°, "
                f"Gripper={sc_angle}")

        else:
            print("[INFO] Nieznana komenda. ENTER=Zapisz, O/Z=Chwytak, H/S/P=Pozycje, Q=Wyjdź")

    with open(recorded_poses_FILE, "w") as f:
        json.dump(recorded_poses, f, indent=2)
    print(f"[OK] Zapisano {len(recorded_poses)} pozycji do pliku '{recorded_poses_FILE}'.")

def playback(ctrl):
    print("[TRYB ODTWARZANIA]")

    try:
        with open(recorded_poses_FILE, "r") as f:
            recorded_poses = json.load(f)
    except FileNotFoundError:
        print(f"[BŁĄD] Plik '{recorded_poses_FILE}' nie istnieje.")
        return

    if not recorded_poses:
        print(f"[INFO] Lista pozycji jest pusta.")
        return

    ctrl.torque_on_all()

    last_pose = ctrl.get_all_servo_positions_deg([
        SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
    ])

    for idx, pose in enumerate(recorded_poses, start=1):
        angles = {int(k): v for k, v in pose.items()}

        # Oddziel ST (ramię) od SC (chwytak)
        arm_angles = {
            SERVO_BASE_ID: angles[SERVO_BASE_ID],
            SERVO_SHOULDER_ID: angles[SERVO_SHOULDER_ID],
            SERVO_ELBOW_ID: angles[SERVO_ELBOW_ID],
            SERVO_WRIST_ID: angles[SERVO_WRIST_ID]
        }
        gripper_angle = angles[SERVO_GRIPPER_ID]

        ctrl.sync_angles(last_pose, arm_angles, tempo_dps=TEMPO_DPS)
        sc_servo(SERVO_GRIPPER_ID, gripper_angle, SC_SERVO_SPEED, SC_SERVO_ACC)

        print(f"Poz. {idx}: "
              f"S1={arm_angles[SERVO_BASE_ID]:.1f}°, "
              f"S2={arm_angles[SERVO_SHOULDER_ID]:.1f}°, "
              f"S3={arm_angles[SERVO_ELBOW_ID]:.1f}°, "
              f"S4={arm_angles[SERVO_WRIST_ID]:.1f}°, "
              f"Gripper={gripper_angle:.1f}°")

        # oblicz czas ruchu
        delta = max(abs(arm_angles[sid] - last_pose.get(sid, arm_angles[sid])) for sid in arm_angles)
        move_time = delta / TEMPO_DPS

        time.sleep(move_time + 0.1)

        last_pose = arm_angles

    print("[OK] Odtwarzanie zakończone.")

if __name__ == "__main__":
    servo_ctrl = ServoController(port_path="/dev/ttyACM0")  # lub UART_PORT z config.py
    kin = Kinematics(100, 100)  # lub importuj L1, L2 z config
    controller = ArmController(kin, servo_ctrl)

    mode = input("Wybierz tryb ([N]agrywaj / [O]dtwarzaj): ").strip().lower()

    if mode == "n":
        record_positions(servo_ctrl)
    elif mode == "o":
        playback(servo_ctrl)
    else:
        print("[INFO] Nieznany tryb. Użyj N lub O.")
