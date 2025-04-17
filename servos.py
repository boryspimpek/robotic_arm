### robot_arm/servos.py

import time
from STservo_sdk.port_handler import PortHandler
from STservo_sdk.sts import sts
from config import BAUDRATE, GLOBAL_SERVO_SPEED, GLOBAL_SERVO_ACC, SERVO_BASE_ID, SERVO_ELBOW_ID, SERVO_SHOULDER_ID

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
        position = self.deg_to_raw(angle_deg)
        speed = GLOBAL_SERVO_SPEED
        acc = GLOBAL_SERVO_ACC
        self.ctrl.WritePosEx(servo_id, position, speed, acc)

    def move_to(self, angles: dict):
        for sid, angle in angles.items():
            self.move_servo(sid, angle)

    def sync_points(self, angles: dict):
        max_delta = 0.1  # zabezpieczenie przed dzieleniem przez zero
        for sid, target in angles.items():
            previous = self.last_positions.get(sid, target)
            delta = abs(target - previous)
            if delta > max_delta:
                max_delta = delta

        for sid, angle in angles.items():
            prev = self.last_positions.get(sid, angle)
            delta = abs(angle - prev)
            speed_ratio = delta / max_delta if max_delta > 0 else 1.0
            speed = max(int(GLOBAL_SERVO_SPEED * speed_ratio), 10)
            acc = GLOBAL_SERVO_ACC
            raw = self.deg_to_raw(angle)
            self.ctrl.WritePosEx(sid, raw, speed, acc)
            self.last_positions[sid] = angle

    def sync_angles(self, start_angles: dict, end_angles: dict, tempo_dps=60.0):
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
        return positions

    def torque_off(self, servo_id):
        self.ctrl.write1ByteTxRx(servo_id, 40, 0)

    def torque_on(self, servo_id):
        self.ctrl.write1ByteTxRx(servo_id, 40, 1)

    def torque_off_all(self):
        for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID]:
            self.torque_off(sid)

    def torque_on_all(self):
        for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID]:
            self.torque_on(sid)

