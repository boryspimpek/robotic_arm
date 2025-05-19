import math
import time
import numpy as np
from config import base, shoulder, elbow, wrist
from sc_controll import close_gripper
from utilis import Utilis

class ArmController:
    def __init__(self, servo_ctrl, kinematics, fullkin):
        self.servo = servo_ctrl
        self.kin = kinematics
        self.fullkin = fullkin
        self.utilis = Utilis(servo_ctrl, kinematics, fullkin)

    def pad_ik_simple(self, x, z, phi_deg, elbow_up=True, wrist_horizontal=True):
        angles, current_angles = self.utilis.prepare_point_and_angles(
            (x, 0.0, z),
            elbow_up=elbow_up,
            wrist_horizontal=wrist_horizontal,
            phi_override=phi_deg
        )
        if angles is False:
            return False
        
        deltas = {}  
        for sid in angles:
            current_angle = current_angles[sid]
            target_angle = angles[sid]
            delta = abs(target_angle - current_angle)  
            deltas[sid] = delta

        max_delta = max(deltas.values())  
        steps = - round(-2.05 * math.exp(0.06 * max_delta))

        interpolations = {
            sid: np.linspace(current_angles[sid], angles[sid], steps)
            for sid in angles
        }

        for i in range(steps):
            step_angles = {sid: interpolations[sid][i] for sid in angles}
            self.servo.sync_points(step_angles)
            time.sleep(0.001)

        return True

    def move_to_point_simple(self, target_xyz, elbow_up=True, tempo_dps=60.0):
        servo_angles, current_servo_angles = self.utilis.prepare_point_and_angles(
            target_xyz,
            elbow_up=elbow_up,
            wrist_horizontal=True
        )
        if servo_angles is False:
            return False

        if not self.utilis.check_collision(servo_angles, current_servo_angles):
            print("[INFO] Ruch przerwany z powodu potencjalnej kolizji.")
            return False
            
        angle_deltas = {
            sid: abs(servo_angles[sid] - current_servo_angles[sid])
            for sid in servo_angles
        }

        max_delta = max(angle_deltas.values())
        time_to_move = max_delta / tempo_dps

        self.servo.sync_angles(current_servo_angles, servo_angles, tempo_dps)

        print(f"[INFO] Czekam {time_to_move:.2f} sekund na zakończenie ruchu...")
        time.sleep(time_to_move)
        print("[INFO] Ruch zakończony.")
        return True
    
    def move_to_angle(self, angle1, angle2, angle3, angle4, tempo_dps=60):
        current_angles = self.servo.get_positions([base, shoulder, elbow, wrist])
        print(current_angles)    

        start_angles = current_angles
        end_angles = {
            base: angle1, 
            shoulder: angle2,  
            elbow: angle3, 
            wrist: angle4 
        }

        self.servo.sync_angles(start_angles, end_angles, tempo_dps)
        total_time = self.servo.sync_angles(start_angles, end_angles, tempo_dps)
        return total_time
    
    def start_manual_mode(self, tempo_dps=60):
        current_angles = self.servo.get_positions([base, shoulder, elbow, wrist])
        print(current_angles)    

        start_angles = current_angles
        end_angles = {
            1: 90,
            2: 20,
            3: 290,
            4: 190
        }

        self.servo.sync_angles(start_angles, end_angles, tempo_dps)
        close_gripper()
        total_time = self.servo.sync_angles(start_angles, end_angles, tempo_dps)
        return total_time
    
    def pad_ik_full(self, x, z, phi_deg, cost_mode):
        current_angles = self.servo.get_positions([base, shoulder, elbow, wrist])

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
            s1, s2, s3, s4 = self.fullkin.ik_3d_to_servo_angles(ik_angles)
        except Exception as e:
            print(f"[ERROR] Błąd konwersji kątów IK do kątów serw: {e}")
            return False

        angles = {
            base: s1,
            shoulder: s2, 
            elbow: s3, 
            wrist: s4
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

    def move_to_point_full(self, x, y, z, tempo_dps=60, cost_mode="min_angle_sum"):
        current_servo_angles = self.servo.get_positions([base, shoulder, elbow, wrist])

        for sid in [base, shoulder, elbow, wrist]:
            if sid not in current_servo_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return False

        try:
            ik_result = self.fullkin.solve_ik_full(x, y, z, cost_mode)
        except ValueError as e:
            print(f"[ERROR] Błąd obliczeń IK: {e}")
            return False

        if ik_result is None or ik_result[0] is None:
            print("[WARN] Solver nie znalazł rozwiązania IK (None).")
            return False

        best_angles, positions = ik_result

        target_servo_angles = self.utilis.ik_to_servo_angles(best_angles)

        angle_deltas = {
            sid: abs(target_servo_angles[sid] - current_servo_angles[sid])
            for sid in target_servo_angles
        }

        max_delta = max(angle_deltas.values())  # Największa różnica kąta
        time_to_move = max_delta / tempo_dps  # Szacowany czas ruchu

        self.servo.sync_angles(current_servo_angles, target_servo_angles, tempo_dps)
        total_time = self.servo.sync_angles(current_servo_angles, target_servo_angles, tempo_dps)

        target_servo_angles = {int(sid): float(angle) for sid, angle in target_servo_angles.items()}
        target_servo_angles = {sid: round(angle, 2) for sid, angle in target_servo_angles.items()}
        print(f"[INFO] Targets: {target_servo_angles}")

        print(f"[INFO] Czekam {time_to_move:.2f} sekund na zakończenie ruchu...")
        time.sleep(time_to_move)

        print("[INFO] Ruch zakończony.")
        return total_time

