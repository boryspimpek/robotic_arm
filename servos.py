### robot_arm/servos.py

import time
from STservo_sdk.port_handler import PortHandler
from STservo_sdk.sts import sts
from config import BAUDRATE, GLOBAL_SERVO_SPEED, GLOBAL_SERVO_ACC, SERVO_ANGLE_LIMITS, SERVO_BASE_ID, SERVO_ELBOW_ID, SERVO_SHOULDER_ID, SERVO_TRIMS, SERVO_WRIST_ID

class ServoController:
    def __init__(self, port_path):
        self.port = PortHandler(port_path)
        self.port.openPort()
        self.port.setBaudRate(BAUDRATE)
        self.ctrl = sts(self.port)
        self.last_positions = {}  # zapamiętanie poprzednich pozycji

    def deg_to_raw(self, angle_deg):
        return int(angle_deg * 4095 / 2 / 180)

    def move_servo(self, servo_id, angle_deg):
        int_angle = int(angle_deg)

        position = self.deg_to_raw(int_angle)
        print(f"Servo {servo_id}: {angle_deg}° → raw {position}")

        speed = GLOBAL_SERVO_SPEED
        acc = GLOBAL_SERVO_ACC
        self.ctrl.WritePosEx(servo_id, position, speed, acc)
        self.last_positions[servo_id] = int_angle
        
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
            acc = GLOBAL_SERVO_ACC
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
        for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID]:
            self.torque_off(sid)

    def torque_on_all(self):
        for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID]:
            self.torque_on(sid)

