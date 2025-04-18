### robot_arm/controller.py

import time
import numpy as np
from config import GLOBAL_SERVO_ACC, SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID

class ArmController:
    def __init__(self, kinematics, servo_ctrl):
        self.kin = kinematics
        self.servo = servo_ctrl

    def move_to_point_steps(self, target_xyz, steps, elbow_up=True):
        from config import SERVO_WRIST_ID

        current_angles = self.servo.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
        ])

        for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID]:
            if sid not in current_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return

        try:
            phi, t1, t2, t3 = self.kin.inverse(*target_xyz, elbow_up)
            target_s1, target_s2, target_s3, target_s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Punkt docelowy nieosiągalny: {e}")
            return False

        s1s = np.linspace(current_angles[SERVO_BASE_ID], target_s1, steps)
        s2s = np.linspace(current_angles[SERVO_SHOULDER_ID], target_s2, steps)
        s3s = np.linspace(current_angles[SERVO_ELBOW_ID], target_s3, steps)
        s4s = np.linspace(current_angles[SERVO_WRIST_ID], target_s4, steps)

        for s1, s2, s3, s4 in zip(s1s, s2s, s3s, s4s):
            self.servo.sync_points({
                SERVO_BASE_ID: s1,
                SERVO_SHOULDER_ID: s2,
                SERVO_ELBOW_ID: s3,
                SERVO_WRIST_ID: s4
            })
            time.sleep(0.01)

        return True

    def point_to_point(self, start, end, steps=300, elbow_up=True):
        from config import SERVO_WRIST_ID

        print("[INFO] Przejście do punktu startowego trajektorii...")
        self.move_to_point_dps(start, tempo_dps=30, elbow_up=elbow_up)

        current_angles = self.servo.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
        ])
        self.servo.last_positions.update(current_angles)

        x0, y0, z0 = start
        x1, y1, z1 = end
        ts = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, steps))
        xs = x0 + (x1 - x0) * ts
        ys = y0 + (y1 - y0) * ts
        zs = z0 + (z1 - z0) * ts

        last_angles = None

        for x, y, z in zip(xs, ys, zs):
            try:
                phi, t1, t2, t3 = self.kin.inverse(x, y, z, elbow_up)
                s1, s2, s3, s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)

                if last_angles:
                    delta = max(
                        abs(s1 - last_angles[0]),
                        abs(s2 - last_angles[1]),
                        abs(s3 - last_angles[2]),
                        abs(s4 - last_angles[3])
                    )
                    sleep_time = max(0.001, min(0.03, delta / 100))
                else:
                    sleep_time = 0.001

                self.servo.sync_points({
                    SERVO_BASE_ID: s1,
                    SERVO_SHOULDER_ID: s2,
                    SERVO_ELBOW_ID: s3,
                    SERVO_WRIST_ID: s4
                })
                time.sleep(sleep_time)
                last_angles = (s1, s2, s3, s4)

            except Exception as e:
                print(f"[OSTRZEŻENIE] Punkt ({x:.1f}, {y:.1f}, {z:.1f}) pominięty: {e}")

    def move_xyz_interpolated(self, target_xyz, steps, elbow_up=True):
        from config import SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID

        # 1. Aktualne kąty
        current_angles = self.servo.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID
        ])
        if any(sid not in current_angles for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID]):
            print("[ERROR] Brak odczytu z któregoś serwa – ruch przerwany.")
            return False

        # 2. Kąt docelowy
        try:
            phi, t1, t2 = self.kin.inverse(*target_xyz, elbow_up)
            target_s1, target_s2, target_s3 = self.kin.to_servo_angles(phi, t1, t2, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Punkt docelowy nieosiągalny: {e}")
            return False

        # 3. Interpolacja kątów
        import numpy as np
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
            time.sleep(0.001)

        # Aktualizacja pozycji końcowej
        self.servo.last_positions[SERVO_BASE_ID] = target_s1
        self.servo.last_positions[SERVO_SHOULDER_ID] = target_s2
        self.servo.last_positions[SERVO_ELBOW_ID] = target_s3

        return True

    def pad_controll(self, x, z, phi_deg, steps=5, elbow_up=True):
        from config import (
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
        )

        try:
            # 1. Wylicz kąty (ustawiamy y=0 bo poruszamy się w XZ)
            phi, t1, t2, t3 = self.kin.inverse(x, 0.0, z, elbow_up)

            # 2. Oblicz kąty serw z uwzględnieniem trimów
            s1, s2, s3, s4 = self.kin.to_servo_angles(phi_deg, t1, t2, t3, apply_trim=True)

            # 3. Przypisz cele do serw
            angles = {
                SERVO_BASE_ID: s1,
                SERVO_SHOULDER_ID: s2,
                SERVO_ELBOW_ID: s3,
                SERVO_WRIST_ID: s4
            }

            # 4. Odczyt aktualnych kątów
            current_angles = self.servo.get_all_servo_positions_deg(list(angles.keys()))
            for sid in angles:
                if sid not in current_angles:
                    print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}")
                    return False

            # 5. Interpolacja pozycji
            import numpy as np
            interpolations = {
                sid: np.linspace(current_angles[sid], angles[sid], steps)
                for sid in angles
            }

            for i in range(steps):
                step_angles = {sid: interpolations[sid][i] for sid in angles}
                self.servo.sync_points(step_angles)
                time.sleep(0.001)

            # 6. Zapisz ostatnie pozycje
            self.servo.last_positions.update(angles)

            return True

        except Exception as e:
            print(f"[ERROR] Nie udało się wykonać ruchu: {e}")
            return False

    def move_to_point_dps(self, target_xyz, elbow_up=True, tempo_dps=60.0):
        from config import SERVO_WRIST_ID

        # Odczytanie aktualnych kątów serw (4 serwa)
        current_angles = self.servo.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
        ])

        # Walidacja — brak pozycji z któregoś serwa
        for sid in [SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID]:
            if sid not in current_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return

        # Obliczenie kątów serw na podstawie punktu docelowego
        try:
            phi, t1, t2, t3 = self.kin.inverse(*target_xyz, elbow_up)
            target_s1, target_s2, target_s3, target_s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Punkt docelowy nieosiągalny: {e}")
            return False

        # Synchronizacja kątów za pomocą sync_angles
        start_angles = current_angles
        end_angles = {
            SERVO_BASE_ID: target_s1,
            SERVO_SHOULDER_ID: target_s2,
            SERVO_ELBOW_ID: target_s3,
            SERVO_WRIST_ID: target_s4
        }

        # Obliczanie różnicy kątów
        angle_deltas = {sid: abs(end_angles[sid] - start_angles[sid]) for sid in end_angles}

        # Obliczanie czasu dla każdego serwa na podstawie dps
        max_delta = max(angle_deltas.values())  # Maksymalna różnica kątów (najdłuższy ruch)
        time_to_move = max_delta / tempo_dps  # Czas ruchu w sekundach

        # Wywołanie sync_angles z odpowiednim tempem (dps)
        self.servo.sync_angles(start_angles, end_angles, tempo_dps)

        # Czekanie na zakończenie ruchu
        print(f"[INFO] Czekam {time_to_move:.2f} sekund na zakończenie ruchu...")
        time.sleep(time_to_move)

        print("[INFO] Ruch zakończony.")
        return True