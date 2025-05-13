### robot_arm/controller.py

import logging
import math
import time
import numpy as np
from config import L1, L2, L3, base, shoulder, elbow, wrist, port
from kinematics_full import FullKinematics
from sc_controll import close_gripper
from servos import ServoController

servo_ctrl = ServoController(port)
fullkin = FullKinematics(L1, L2, L3)


class ArmController:
    def __init__(self, kinematics, servo_ctrl):
        self.kin = kinematics
        self.servo = servo_ctrl
        self.fullkin = FullKinematics(L1, L2, L3)

    def pad_ik(self, x, z, phi_deg, elbow_up=True, wrist_horizontal=True):
        # Inverse kinematics
        try:
            ik_angles = self.kin.inverse(x, 0.0, z, elbow_up)
            if ik_angles is None:
                print("[WARN] Solver nie znalazł rozwiązania IK (None).")
                return False
            phi, t1, t2, t3 = ik_angles
        except Exception as e:
            print(f"[ERROR] Błąd obliczeń IK: {e}")
            return False

        # Konwersja do serwo
        try:
            s1, s2, s3, s4 = self.kin.to_servo_angles(phi_deg, t1, t2, t3, wrist_horizontal=wrist_horizontal, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Błąd konwersji kątów IK do kątów serw: {e}")
            return False
        
        angles = {
            base: s1,
            shoulder: s2,
            elbow: s3,
            wrist: s4
        }

        current_angles = self.servo.get_all_servo_positions_deg(list(angles.keys()))
        for sid in angles:
            if sid not in current_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}")
                return False

        deltas = {}  
        for sid in angles:
            current_angle = current_angles[sid]
            target_angle = angles[sid]
            delta = abs(target_angle - current_angle)  
            deltas[sid] = delta
            # print(f"Serwo {sid}: Delta = {delta:.2f}°")

        max_delta = max(deltas.values())  
        # print(f"max delta steps = {max_delta:.2f}°")
        steps = - round(-2.05 * math.exp(0.06 * max_delta))
        # print(f"steps  = {steps:.2f}°")

        import numpy as np
        interpolations = {
            sid: np.linspace(current_angles[sid], angles[sid], steps)
            for sid in angles
        }

        for i in range(steps):
            step_angles = {sid: interpolations[sid][i] for sid in angles}
            self.servo.sync_points(step_angles)
            time.sleep(0.001)

        self.servo.last_positions.update(angles)

        return True

    def move_to_point_dps(self, target_xyz, elbow_up=True, tempo_dps=60.0):
        current_servo_angles = self.servo.get_all_servo_positions_deg([base, shoulder, elbow, wrist])

        for sid in [base, shoulder, elbow, wrist]:
            if sid not in current_servo_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return False

        # Inverse kinematics
        try:
            ik_angles = self.kin.inverse(*target_xyz, elbow_up)
            if ik_angles is None:
                print("[WARN] Solver nie znalazł rozwiązania IK (None).")
                return False
            phi, t1, t2, t3 = ik_angles
        except Exception as e:
            print(f"[ERROR] Błąd obliczeń IK: {e}")
            return False

        # Konwersja do serwo
        try:
            s1, s2, s3, s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Błąd konwersji kątów IK do kątów serw: {e}")
            return False

        end_servo_angles = {
            base: s1,
            shoulder: s2,
            elbow: s3,
            wrist: s4
        }

        # Obliczenie różnic kątów i czasu ruchu
        angle_deltas = {
            sid: abs(end_servo_angles[sid] - current_servo_angles[sid])
            for sid in end_servo_angles
        }

        max_delta = max(angle_deltas.values())
        time_to_move = max_delta / tempo_dps

        self.servo.sync_angles(current_servo_angles, end_servo_angles, tempo_dps)

        print(f"[INFO] Czekam {time_to_move:.2f} sekund na zakończenie ruchu...")
        time.sleep(time_to_move)
        print("[INFO] Ruch zakończony.")
        return True
    
    def homepos(self, tempo_dps=60):
        current_angles = servo_ctrl.get_all_servo_positions_deg([1, 2, 3, 4])
        print(current_angles)    

        start_angles = current_angles
        end_angles = {
            1: 90,
            2: 6,
            3: 179,
            4: 140
        }

        servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps)
        close_gripper()
        total_time = servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps)
        return total_time
    
    def start(self, tempo_dps=60):
        current_angles = servo_ctrl.get_all_servo_positions_deg([1, 2, 3, 4])
        print(current_angles)    

        start_angles = current_angles
        end_angles = {
            1: 90,
            2: 6,
            3: 179,
            4: 25
        }

        servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps)
        close_gripper()
        total_time = servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps)
        return total_time
    
    def pad_ik_full(self, x, z, phi_deg, cost_mode):
        current_angles = servo_ctrl.get_all_servo_positions_deg([base, shoulder, elbow, wrist])

        for sid in [base, shoulder, elbow, wrist]:
            if sid not in current_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return False

        try:
            ik_result = self.fullkin.solve_ik_3d(x, 0, z, cost_mode)
        except ValueError as e:
            print(f"[ERROR] Błąd obliczeń IK: {e}")
            return False

        if ik_result is None or ik_result[0] is None:
            print("[WARN] Solver nie znalazł rozwiązania IK (None).")
            return False

        ik_angles, positions = ik_result  

        try:
            target_servo_angles = self.fullkin.ik_3d_to_servo_angles(ik_angles)
        except Exception as e:
            print(f"[ERROR] Błąd konwersji kątów IK do kątów serw: {e}")
            return False

        angles = {
            base: target_servo_angles[base],
            shoulder: target_servo_angles[shoulder], 
            elbow: target_servo_angles[elbow], 
            wrist: target_servo_angles[wrist]
        }

        deltas = {}  
        for sid in angles:
            current_angle = current_angles[sid]
            target_angle = angles[sid]
            delta = abs(target_angle - current_angle)  
            deltas[sid] = delta
            # print(f"Serwo {sid}: Delta = {delta:.2f}°")

        max_delta = max(deltas.values())  
        # print(f"max delta steps = {max_delta:.2f}°")
        steps = - round(-2.05 * math.exp(0.06 * max_delta))
        # print(f"steps  = {steps:.2f}°")

        import numpy as np
        interpolations = {
            sid: np.linspace(current_angles[sid], angles[sid], steps)
            for sid in angles
        }

        for i in range(steps):
            step_angles = {sid: interpolations[sid][i] for sid in angles}
            self.servo.sync_points(step_angles)
            time.sleep(0.001)

        self.servo.last_positions.update(angles)

        return True

    def move_to_point_ik_full(self, x, y, z, tempo_dps=60, cost_mode="min_angle_sum"):
        current_servo_angles = servo_ctrl.get_all_servo_positions_deg([base, shoulder, elbow, wrist])

        for sid in [base, shoulder, elbow, wrist]:
            if sid not in current_servo_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return False

        try:
            ik_result = self.fullkin.solve_ik_3d(x, y, z, cost_mode)
        except ValueError as e:
            print(f"[ERROR] Błąd obliczeń IK: {e}")
            return False

        if ik_result is None or ik_result[0] is None:
            print("[WARN] Solver nie znalazł rozwiązania IK (None).")
            return False

        angles, angles_deg, positions = ik_result

        theta0, theta1, theta2, theta3 = angles_deg

        target_servo_angles = {
            base: theta0,
            shoulder: theta1,
            elbow: theta2,
            wrist: theta3
        }

        angle_deltas = {
            sid: abs(target_servo_angles[sid] - current_servo_angles[sid])
            for sid in target_servo_angles
        }

        max_delta = max(angle_deltas.values())  # Największa różnica kąta
        time_to_move = max_delta / tempo_dps  # Szacowany czas ruchu

        self.servo.sync_angles(current_servo_angles, target_servo_angles, tempo_dps)
        total_time = servo_ctrl.sync_angles(current_servo_angles, target_servo_angles, tempo_dps)

        target_servo_angles = {int(sid): float(angle) for sid, angle in target_servo_angles.items()}
        target_servo_angles = {sid: round(angle, 2) for sid, angle in target_servo_angles.items()}
        print(f"[INFO] Targets: {target_servo_angles}")

        print(f"[INFO] Czekam {time_to_move:.2f} sekund na zakończenie ruchu...")
        time.sleep(time_to_move)

        print("[INFO] Ruch zakończony.")
        return True

