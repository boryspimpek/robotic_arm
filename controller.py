### robot_arm/controller.py

import math
import time
import numpy as np
from config import base, schoulder, elbow, wrist

class ArmController:
    def __init__(self, kinematics, servo_ctrl):
        self.kin = kinematics
        self.servo = servo_ctrl

    def point_to_point(self, start, end, steps=300, elbow_up=True):
        print("[INFO] Przejście do punktu startowego trajektorii...")
        self.move_to_point_dps(start, tempo_dps=30, elbow_up=elbow_up)

        x0, y0, z0 = start
        x1, y1, z1 = end
        ts = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, steps))
        xs = x0 + (x1 - x0) * ts
        ys = y0 + (y1 - y0) * ts
        zs = z0 + (z1 - z0) * ts

        for x, y, z in zip(xs, ys, zs):
            try:
                phi, t1, t2, t3 = self.kin.inverse(x, y, z, elbow_up)
                s1, s2, s3, s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)

                self.servo.sync_points({
                    base: s1,
                    schoulder: s2,
                    elbow: s3,
                    wrist: s4
                })
                time.sleep(0.003)

            except Exception as e:
                print(f"[OSTRZEŻENIE] Punkt ({x:.1f}, {y:.1f}, {z:.1f}) pominięty: {e}")

    def circle_xz(self, center_x, center_z, radius, start_angle_deg=0, end_angle_deg=360, y_fixed=0.0, steps=600, elbow_up=True):
        angles = np.radians(np.linspace(start_angle_deg, end_angle_deg, steps))
        xs = center_x + radius * np.cos(angles)
        zs = center_z + radius * np.sin(angles)
        ys = np.full_like(xs, y_fixed)

        for x, y, z in zip(xs, ys, zs):
            try:
                phi, t1, t2, t3 = self.kin.inverse(x, y, z, elbow_up)
                s1, s2, s3, s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)

                self.servo.sync_points({
                    base: s1,
                    schoulder: s2,
                    elbow: s3,
                    wrist: s4
                })
                time.sleep(0.01)

            except Exception as e:
                print(f"[OSTRZEŻENIE] Punkt ({x:.1f}, {y:.1f}, {z:.1f}) pominięty: {e}")

    def pad_ik(self, x, z, phi_deg, elbow_up=True, wrist_horizontal=True):
        try:
            # 1. Wylicz kąty (ustawiamy y=0 bo poruszamy się w XZ)
            phi, t1, t2, t3 = self.kin.inverse(x, 0.0, z, elbow_up)

            # 2. Oblicz kąty serw z uwzględnieniem trimów
            s1, s2, s3, s4 = self.kin.to_servo_angles(phi_deg, t1, t2, t3, wrist_horizontal=wrist_horizontal, apply_trim=True)

            # 3. Przypisz cele do serw
            angles = {
                base: s1,
                schoulder: s2,
                elbow: s3,
                wrist: s4
            }

            # 4. Odczyt aktualnych kątów
            current_angles = self.servo.get_all_servo_positions_deg(list(angles.keys()))
            for sid in angles:
                if sid not in current_angles:
                    print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}")
                    return False

            # 5. Obliczanie delty (różnicy) pomiędzy kątami z IK a aktualnymi kątami
            deltas = {}  # Słownik do przechowywania delty dla każdego serwa
            for sid in angles:
                current_angle = current_angles[sid]
                target_angle = angles[sid]
                delta = abs(target_angle - current_angle)  # Delta pomiędzy aktualnym a docelowym kątem
                deltas[sid] = delta
                # print(f"Serwo {sid}: Delta = {delta:.2f}°")

            # 6. Dynamiczne określenie liczby kroków na podstawie max delta
            max_delta = max(deltas.values())  # Największa zmiana kąta
            # print(f"max delta steps = {max_delta:.2f}°")
            steps = - round(-2.05 * math.exp(0.06 * max_delta))
            # print(f"steps  = {steps:.2f}°")

            # 7. Interpolacja pozycji
            import numpy as np
            interpolations = {
                sid: np.linspace(current_angles[sid], angles[sid], steps)
                for sid in angles
            }

            for i in range(steps):
                step_angles = {sid: interpolations[sid][i] for sid in angles}
                self.servo.sync_points(step_angles)
                time.sleep(0.001)

            # 8. Zapisz ostatnie pozycje
            self.servo.last_positions.update(angles)

            return True

        except Exception as e:
            print(f"[ERROR] Nie udało się wykonać ruchu: {e}")
            return False

    def move_to_point_dps(self, target_xyz, elbow_up=True, tempo_dps=60.0):
        # Odczytanie aktualnych kątów serw (4 serwa)
        current_angles = self.servo.get_all_servo_positions_deg([
            base, schoulder, elbow, wrist
        ])

        # Walidacja — brak pozycji z któregoś serwa
        for sid in [base, schoulder, elbow, wrist]:
            if sid not in current_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return

        # Obliczenie kątów serw na podstawie punktu docelowego
        try:
            phi, t1, t2, t3 = self.kin.inverse(*target_xyz, elbow_up)
            s1, s2, s3, s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Punkt docelowy nieosiągalny: {e}")
            return False

        # Synchronizacja kątów za pomocą sync_angles
        start_angles = current_angles
        end_angles = {
            base: s1,
            schoulder: s2,
            elbow: s3,
            wrist: s4
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