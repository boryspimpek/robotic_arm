### robot_arm/controller.py

import time
import numpy as np
from config import SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID

class ArmController:
    def __init__(self, kinematics, servo_ctrl):
        self.kin = kinematics
        self.servo = servo_ctrl

    def move_to_point(self, target_xyz, steps, elbow_up=True):
        from config import SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID

        # 1. Odczyt aktualnych kątów serw
        current_angles = self.servo.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID
        ])

        # Walidacja — brak pozycji z któregoś serwa
        for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID]:
            if sid not in current_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return

        # 2. Wyliczenie docelowych kątów z inverse kinematics
        try:
            phi, t1, t2 = self.kin.inverse(*target_xyz, elbow_up)
            target_s1, target_s2, target_s3 = self.kin.to_servo_angles(phi, t1, t2, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Punkt docelowy nieosiągalny: {e}")
            return False

        # 3. Interpolacja między aktualnymi a docelowymi kątami
        s1_0 = current_angles[SERVO_BASE_ID]
        s2_0 = current_angles[SERVO_SHOULDER_ID]
        s3_0 = current_angles[SERVO_ELBOW_ID]

        s1s = np.linspace(s1_0, target_s1, steps)
        s2s = np.linspace(s2_0, target_s2, steps)
        s3s = np.linspace(s3_0, target_s3, steps)

        for s1, s2, s3 in zip(s1s, s2s, s3s):
            self.servo.sync_points({
                SERVO_BASE_ID: s1,
                SERVO_SHOULDER_ID: s2,
                SERVO_ELBOW_ID: s3
            })
            time.sleep(0.01)
        return True

    def point_to_point(self, start, end, steps=300, elbow_up=True):
        from config import SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID

        print("[INFO] Przejście do punktu startowego trajektorii...")
        self.move_to_point(start, steps=150, elbow_up=elbow_up)
        time.sleep(1)

        current_angles = self.servo.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID
        ])
        self.servo.last_positions.update(current_angles)

        # Trajektoria XYZ - zawsze liniowa
        x0, y0, z0 = start
        x1, y1, z1 = end
        ts = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, steps))
        xs = x0 + (x1 - x0) * ts
        ys = y0 + (y1 - y0) * ts
        zs = z0 + (z1 - z0) * ts

        last_angles = None

        for x, y, z in zip(xs, ys, zs):
            try:
                phi, t1, t2 = self.kin.inverse(x, y, z, elbow_up)
                s1, s2, s3 = self.kin.to_servo_angles(phi, t1, t2, apply_trim=True)

                # Oblicz, jak szybko się zmieniają kąty
                if last_angles:
                    delta = max(abs(s1 - last_angles[0]),
                                abs(s2 - last_angles[1]),
                                abs(s3 - last_angles[2]))
                    sleep_time = max(0.001, min(0.03, delta / 30))  # np. 6° → 0.02s
                else:
                    sleep_time = 0.01

                self.servo.sync_points({
                    SERVO_BASE_ID: s1,
                    SERVO_SHOULDER_ID: s2,
                    SERVO_ELBOW_ID: s3
                })
                time.sleep(sleep_time)
                last_angles = (s1, s2, s3)

            except Exception as e:
                print(f"[OSTRZEŻENIE] Punkt ({x:.1f}, {y:.1f}, {z:.1f}) pominięty: {e}")
