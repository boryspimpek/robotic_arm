### robot_arm/controller.py

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
        try:
            phi, t1, t2, t3 = self.kin.inverse(x, 0.0, z, elbow_up)
            s1, s2, s3, s4 = self.kin.to_servo_angles(phi_deg, t1, t2, t3, wrist_horizontal=wrist_horizontal, apply_trim=True)

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

        except Exception as e:
            print(f"[ERROR] Nie udało się wykonać ruchu: {e}")
            return False

    def move_to_point_dps(self, target_xyz, elbow_up=True, tempo_dps=60.0):
        current_angles = self.servo.get_all_servo_positions_deg([
            base, shoulder, elbow, wrist
        ])

        for sid in [base, shoulder, elbow, wrist]:
            if sid not in current_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return

        try:
            phi, t1, t2, t3 = self.kin.inverse(*target_xyz, elbow_up)
            s1, s2, s3, s4 = self.kin.to_servo_angles(phi, t1, t2, t3, apply_trim=True)
        except Exception as e:
            print(f"[ERROR] Punkt docelowy nieosiągalny: {e}")
            return False

        start_angles = current_angles
        end_angles = {
            base: s1,
            shoulder: s2,
            elbow: s3,
            wrist: s4
        }

        angle_deltas = {sid: abs(end_angles[sid] - start_angles[sid]) for sid in end_angles}

        max_delta = max(angle_deltas.values())  # Maksymalna różnica kątów (najdłuższy ruch)
        time_to_move = max_delta / tempo_dps  # Czas ruchu w sekundach

        self.servo.sync_angles(start_angles, end_angles, tempo_dps)

        print(f"[INFO] Czekam {time_to_move:.2f} sekund na zakończenie ruchu...")
        time.sleep(time_to_move)

        print("[INFO] Ruch zakończony.")
        return True
    
    def homepos(self):
        current_angles = servo_ctrl.get_all_servo_positions_deg([1, 2, 3, 4])
        print(current_angles)    

        start_angles = current_angles
        end_angles = {
            1: 90,
            2: 6,
            3: 179,
            4: 140
        }

        servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
        close_gripper()
        total_time = servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
        return total_time
    
    def start(self):
        current_angles = servo_ctrl.get_all_servo_positions_deg([1, 2, 3, 4])
        print(current_angles)    

        start_angles = current_angles
        end_angles = {
            1: 90,
            2: 6,
            3: 179,
            4: 25
        }

        servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
        close_gripper()
        total_time = servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
        return total_time
    
    def pad_ik_full(self, x, z, phi_deg, elbow_up=True, wrist_horizontal=True):
        try:
            end_angles, positions = self.fullkin.solve_ik_3d(x, 0.0, z)
            servo_angles = self.fullkin.ik_3d_to_servo_angles(end_angles)

            angles = {
                base: servo_angles[base],
                shoulder: servo_angles[shoulder],
                elbow: servo_angles[elbow],
                wrist: servo_angles[wrist]
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

        except Exception as e:
            print(f"[ERROR] Nie udało się wykonać ruchu: {e}")
            return False

    def move_to_point_ik_full(self, x, y, z):
        angles, positions = fullkin.solve_ik_3d(x, y, z)
        end_angles = fullkin.ik_3d_to_servo_angles(angles)
        start_angles = servo_ctrl.get_all_servo_positions_deg([base, shoulder, elbow, wrist])
        servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
        total_time = servo_ctrl.sync_angles(start_angles, end_angles, tempo_dps=30)
        return total_time

