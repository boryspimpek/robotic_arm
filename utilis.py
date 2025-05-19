import numpy as np
from config import base, shoulder, elbow, wrist
from config import base_angle_limits, shoulder_angle_limits, elbow_angle_limits, wrist_angle_limits

class Utilis:
    def __init__(self, servo_ctrl, kinematics, full_kinematics):
        self.servo = servo_ctrl
        self.kin = kinematics
        self.fullkin = full_kinematics
        

    def prepare_point_and_angles(self, target_xyz, elbow_up=True, wrist_horizontal=True, phi_override=None):
        try:
            ik_angles, current_servo_angles = self.prepare_to_move_ik(*target_xyz, elbow_up=elbow_up)
            if ik_angles is False:
                return False, None
        except Exception as e:
            print(f"[ERROR] Błąd przygotowania ruchu IK: {e}")
            return False, None

        try:
            servo_angles = self.ik_to_servo_angles(
                ik_angles,
                wrist_horizontal=wrist_horizontal,
                phi_override=phi_override
            )
        except Exception as e:
            print(f"[ERROR] Błąd konwersji kątów IK do kątów serw: {e}")
            return False, None

        return servo_angles, current_servo_angles

    @staticmethod
    def validate_and_clip_angles(angles):
        angle_limits_per_servo = {
            base: base_angle_limits,
            shoulder: shoulder_angle_limits,
            elbow: elbow_angle_limits,
            wrist: wrist_angle_limits
        }

        for sid, angle in angles.items():
            if sid not in angle_limits_per_servo:
                raise KeyError(f"Brak zdefiniowanego zakresu kątów dla serwa ID {sid}")
            
            min_angle, max_angle = angle_limits_per_servo[sid]

            if not (min_angle <= angle <= max_angle):
                if sid == wrist:
                    clipped = max(min(angle, max_angle), min_angle)
                    print(f"[WARN] Kąt nadgarstka poza zakresem ({angle:.2f}°), przycinam do {clipped:.2f}°")
                    angles[sid] = clipped
                else:
                    raise ValueError(
                        f"Kąt serwa ID {sid} poza zakresem: {angle:.2f}° (dozwolony: {min_angle}°–{max_angle}°)"
                    )
        return angles
    
    def apply_trims(angles, trims):
        for sid in angles:
            angles[sid] += trims.get(sid, 0.0)
        return angles
    
    def prepare_to_move_ik(self, *target_xyz, elbow_up):
        current_servo_angles = self.servo.get_positions([base, shoulder, elbow, wrist])
        for sid in [base, shoulder, elbow, wrist]:
            if sid not in current_servo_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return False

        try:
            ik_angles = self.kin.solve_ik_simple(*target_xyz, elbow_up)
            if ik_angles is None:
                print("[WARN] Solver nie znalazł rozwiązania IK (None).")
                return False
        except Exception as e:
            print(f"[ERROR] Błąd obliczeń IK: {e}")
            return False
        return ik_angles, current_servo_angles

    def ik_to_servo_angles(self, angles_tuple, wrist_horizontal=None, phi_override=None):
        phi, theta1, theta2, theta3 = angles_tuple

        if phi_override is not None:
            phi = phi_override

        if wrist_horizontal is True:
            wrist_angle = 90 + theta3
        elif wrist_horizontal is False:
            wrist_angle = 180 + theta3
        else:
            wrist_angle = 90 - theta3

        angles = {
            base: 90 - phi,
            shoulder: 180 - theta1,
            elbow: -theta2 + 140,
            wrist: wrist_angle + 40
        }

        angles = self.validate_and_clip_angles(angles)
        return angles

    def check_collision(self, servo_angles, current_servo_angles):
        steps = 30
        interpolated_keys = [shoulder, elbow, wrist]  

        try:
            traj = {
                k: np.linspace(current_servo_angles[k], servo_angles.get(k, current_servo_angles[k]), steps)
                for k in interpolated_keys
            }
        except KeyError as e:
            print(f"[ERROR] Brakuje klucza w kątach serw: {e}")
            return False

        for t1, t2, t3 in zip(traj[shoulder], traj[elbow], traj[wrist]):
            try:
                fk_angles = {
                    shoulder: t1,
                    elbow: t2,
                    wrist: t3
                }
                x2, z2, x3, z3 = self.fullkin.forward_ik_full(fk_angles)
                print(f"[INFO] FK: x2 = {x2:.1f} mm, z2 = {z2:.1f} mm")
                print(f"[INFO] FK: x3 = {x3:.1f} mm, z3 = {z3:.1f} mm")

            except Exception as e:
                print(f"[ERROR] Błąd FK: {e}")
                return False  # zakładamy kolizję w razie błędu

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

        return True
