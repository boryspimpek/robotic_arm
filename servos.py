### robot_arm/servos.py

import sys
import os
import numpy as np
from kinematics import Kinematics
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Library')))
from STservo_sdk.port_handler import PortHandler
from STservo_sdk.sts import sts
from config import L1, L2, baudrate, st_speed, st_acc, angle_limits, base, elbow, schoulder, trims, wrist

class ServoController:
    def __init__(self, port_path):
        self.port = PortHandler(port_path)
        self.port.openPort()
        self.port.setBaudRate(baudrate)
        self.ctrl = sts(self.port)
        self.last_positions = {}  # zapamiętanie poprzednich pozycji
        self.kin = Kinematics(L1, L2)  # Initialize kinematics (ensure Kinematics is imported or defined)

    def deg_to_raw(self, angle_deg):
        return int(angle_deg * 4095 / 2 / 180)

    def safe_move_servo(self, servo_id, target_angle_deg):
        # 1. Odczyt aktualnych kątów barku i łokcia
        relevant_ids = [schoulder, elbow]
        current_angles = self.get_all_servo_positions_deg(relevant_ids)

        if schoulder not in current_angles or elbow not in current_angles:
            print("[ERROR] Nie udało się odczytać aktualnych pozycji serw.")
            return False

        # 2. Symuluj nowy zestaw kątów
        theta1 = current_angles[schoulder]
        theta2 = current_angles[elbow]

        if servo_id == schoulder:
            theta1 = target_angle_deg
        elif servo_id == elbow:
            theta2 = target_angle_deg
        else:
            # Jeśli to nie bark ani łokieć – nie ma wpływu na z, wykonuj bez sprawdzania
            self.move_servo(servo_id, target_angle_deg)
            return True

        # 3. Wywołanie forward kinematics (FK)
        try:
            x, y, z = self.kin.forward(theta1, theta2)
        except Exception as e:
            print(f"[ERROR] Błąd kinematyki prostej: {e}")
            return False

        # 4. Sprawdzenie bezpieczeństwa
        if z < -50.0:
            print(f"[WARN] Ruch odrzucony – zbyt nisko: z = {z:.1f} mm")
            return False

        # 5. Ruch jest bezpieczny – wykonaj
        self.move_servo(servo_id, target_angle_deg)
        return True

    def move_servo(self, servo_id, angle_deg):
        position = self.deg_to_raw(angle_deg)
        print(f"Servo {servo_id}: {angle_deg}° → raw {position}")

        self.ctrl.WritePosEx(servo_id, position, st_speed, st_acc)
        # self.last_positions[servo_id] = angle_deg

    def safe_move_to(self, angles: dict):

        # Pobierz aktualne kąty serw barku i łokcia
        current = self.get_all_servo_positions_deg([schoulder, elbow])
        
        if schoulder not in current or elbow not in current:
            print("[ERROR] Nie udało się odczytać aktualnych pozycji serw.")
            return False

        # Ustal docelowe kąty lub użyj obecnych, jeśli nie podano
        target_theta1 = angles.get(schoulder, current[schoulder])
        target_theta2 = angles.get(elbow, current[elbow])
        
        current_theta1 = current[schoulder]
        current_theta2 = current[elbow]

        # Interpolacja w N krokach
        steps = 30
        theta1_traj = np.linspace(current_theta1, target_theta1, steps)
        theta2_traj = np.linspace(current_theta2, target_theta2, steps)

        # Sprawdź całą trajektorię pod kątem bezpieczeństwa
        for t1, t2 in zip(theta1_traj, theta2_traj):
            try:
                x, y, z = self.kin.forward(t1, t2)
                print(f"[INFO] FK: x = {x:.1f} mm, z = {z:.1f} mm (θ1 = {t1:.1f}°, θ2 = {t2:.1f}°)")

            except Exception as e:
                print(f"[ERROR] Błąd FK: {e}")
                return False
                
            if z < -20.0:
                print(f"[WARN] Ruch przerwany – punkt pośredni zbyt nisko: z = {z:.1f} mm")
                return False
            
            if x > -10.0 and z < 0.0:
                print(f"[WARN] Ruch przerwany – punkt pośredni zbyt daleko w lewo: x = {x:.1f} mm")
                return False
            
            # if x < 10.0:
            #     print(f"[WARN] Ruch przerwany – punkt pośredni zbyt daleko w lewo: x = {x:.1f} mm")
            #     return False

            # if int(round(t2)) == 180 and t1 > 117:
            #     print(f"[WARN] Niedozwolona konfiguracja: elbow=180°, schoulder={t1:.1f}° > 117°")
            #     return False

            # if t1 > 117 and t2 > 160:
            #     print(f"[WARN] Niedozwolona konfiguracja: schoulder={t1:.1f}° > 117°, a elbow={t2:.1f}° > 160°")
            #     return False
        
        # Jeśli cały tor jest bezpieczny – wykonaj ruch
        self.move_to(angles)
        return True

    def move_to(self, angles: dict):
        for sid, angle in angles.items():
            self.move_servo(sid, angle)

    def sync_points(self, angles: dict):
        max_delta = 0.01  # zabezpieczenie przed dzieleniem przez zero
        # print("[SYNC] Obliczanie max_delta:")
        for sid, target in angles.items():
            previous = self.last_positions.get(sid, target)
            delta = abs(target - previous)
            # print(f"  Servo {sid}: current={previous:.2f}, target={target:.2f}, Δ={delta:.2f}")
            if delta > max_delta:
                max_delta = delta

        # print(f"[SYNC] max_delta: {max_delta:.2f}°\n")

        for sid, angle in angles.items():
            prev = self.last_positions.get(sid, angle)
            delta = abs(angle - prev)
            speed_ratio = delta / max_delta if max_delta > 0 else 1.0
            speed = int(500 * speed_ratio)
            acc = 100
            raw = self.deg_to_raw(angle)

            # print(f"[SYNC] Servo {sid} ->")
            # print(f"  Previous: {prev:.2f}°, Target: {angle:.2f}°, Δ: {delta:.2f}°")
            # print(f"  Speed ratio: {speed_ratio:.2f}, Final speed: {speed}")

            self.ctrl.WritePosEx(sid, raw, speed, acc)
            self.last_positions[sid] = angle

        # print("[SYNC] --- Ruch zsynchronizowany ---\n")

    def sync_angles(self, start_angles: dict, end_angles: dict, tempo_dps):
        ids = end_angles.keys()
        max_delta = max(abs(end_angles[sid] - start_angles.get(sid, end_angles[sid])) for sid in ids)
        max_delta = max(max_delta, 1.0)  # min. 1 stopień, by uniknąć dzielenia przez 0

        total_time = max_delta / tempo_dps  # czas w sekundach

        for sid in ids:
            start = start_angles.get(sid, end_angles[sid])
            end = end_angles[sid]
            delta = abs(end - start)

            deg_per_sec = delta / total_time if total_time > 0 else 60.0
            raw_speed = int(deg_per_sec * (4095 / 2 / 180))
            raw_speed = max(raw_speed, 10)

            raw_pos = self.deg_to_raw(end)
            acc = st_acc
            self.ctrl.WritePosEx(sid, raw_pos, raw_speed, acc)
            self.last_positions[sid] = end

    def get_all_servo_positions_deg(self, ids):
        positions = {}
        for sid in ids:
            try:
                raw, _, result, _ = self.ctrl.ReadPosSpeed(sid)
                if result != 0:
                    continue
                deg = raw * 2 * 180 / 4095
                positions[sid] = deg
            except Exception as e:
                print(f"[WARN] Odczyt serwa ID {sid} nieudany: {e}")
        # print(f"[INFO] Odczytano pozycje serw {positions}")
        return positions

    def torque_off(self, servo_id):
        self.ctrl.write1ByteTxRx(servo_id, 40, 0)

    def torque_on(self, servo_id):
        self.ctrl.write1ByteTxRx(servo_id, 40, 1)

    def torque_off_all(self):
        for sid in [base, schoulder, elbow, wrist]:
            self.torque_off(sid)

    def torque_on_all(self):
        for sid in [base, schoulder, elbow, wrist]:
            self.torque_on(sid)

