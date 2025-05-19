### robot_arm/servos.py
import sys
import os
import numpy as np
from kinematics import Kinematics
from config import L1, L2, L3, baudrate, st_speed, st_acc, base, elbow, shoulder, wrist
from kinematics_full import FullKinematics

# Add the Library path to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'Library')))

from STservo_sdk.port_handler import PortHandler
from STservo_sdk.sts import sts


class ServoController:
    def __init__(self, port_path):
        self.port = PortHandler(port_path)
        self.port.openPort()
        self.port.setBaudRate(baudrate)
        self.ctrl = sts(self.port)
        self.last_positions = {}  
        self.kin = Kinematics(L1, L2)  
        self.fullkin = FullKinematics(L1, L2, L3)  

    def deg_to_raw(self, angle_deg):
        return int(angle_deg * 4095 / 2 / 180)

    def move_servo(self, servo_id, angle_deg):
        position = self.deg_to_raw(angle_deg)
        # print(f"Servo {servo_id}: {angle_deg}° → raw {position}")

        self.ctrl.WritePosEx(servo_id, position, st_speed, st_acc)
        # self.last_positions[servo_id] = angle_deg

    def safe_move_to(self, angles: dict):
        interpolated_keys = [shoulder, elbow, wrist]

        current = self.get_positions(interpolated_keys)

        if any(k not in current for k in interpolated_keys):
            print("[ERROR] Nie udało się odczytać aktualnych pozycji serw.")
            return False

        steps = 30
        traj = {
            k: np.linspace(current[k], angles.get(k, current[k]), steps)
            for k in interpolated_keys
        }

        for t1, t2, t3 in zip(traj[shoulder], traj[elbow], traj[wrist]):
            try:
                fk_angles = {shoulder: t1, elbow: t2, wrist: t3}
                x2, z2, x3, z3 = self.fullkin.forward_ik_full(fk_angles)
                print(f"[INFO] FK: x3 = {x3:.1f} mm, z3 = {z3:.1f} mm")
                print(f"[INFO] FK: x2 = {x2:.1f} mm, z2 = {z2:.1f} mm")

            except Exception as e:
                print(f"[ERROR] Błąd FK: {e}")
                return False

            if z3 < -100.0:
                print(f"[WARN] Ruch przerwany – punkt pośredni zbyt nisko: z3 = {z3:.1f} mm, ")
                return False
            if z2 < -90.0:
                print(f"[WARN] Ruch przerwany – punkt pośredni zbyt nisko: z2 = {z3:.1f} mm, ")
                return False
            if z3 < 0 and x3 < 30:
                print(f"[WARN] Ruch przerwany – punkt pośredni zbyt blisko: x2 = {x2:.1f} mm")
                return False
            if z2 < 0 and x2 < 40:
                print(f"[WARN] Ruch przerwany – punkt pośredni zbyt blisko: x2 = {x2:.1f} mm")
                return False

        self.move_to(angles)
        return True

    def move_to(self, angles: dict):
        for sid, angle in angles.items():
            self.move_servo(sid, angle)

    def sync_points(self, angles: dict):
        max_delta = 0.01  
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
        return total_time

    def get_positions(self, ids):
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
    
    def get_current_angles(self):
        ids = [base, shoulder, elbow, wrist]
        positions = self.get_positions(ids)
        return [positions[i] for i in ids]

    def torque_off(self, servo_id):
        self.ctrl.write1ByteTxRx(servo_id, 40, 0)

    def torque_on(self, servo_id):
        self.ctrl.write1ByteTxRx(servo_id, 40, 1)

    def torque_off_all(self):
        for sid in [base, shoulder, elbow, wrist]:
            self.torque_off(sid)

    def torque_on_all(self):
        for sid in [base, shoulder, elbow, wrist]:
            self.torque_on(sid)

