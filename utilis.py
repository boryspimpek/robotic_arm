from config import base, shoulder, elbow, wrist
from config import base_angle_limits, shoulder_angle_limits, elbow_angle_limits, wrist_angle_limits

class Utilis:
    def __init__(self, servo_ctrl, kinematics):
        self.servo = servo_ctrl
        self.kin = kinematics

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
        current_servo_angles = self.servo.get_all_servo_positions_deg([base, shoulder, elbow, wrist])
        for sid in [base, shoulder, elbow, wrist]:
            if sid not in current_servo_angles:
                print(f"[ERROR] Nie udało się odczytać kąta serwa ID {sid}. Ruch przerwany.")
                return False

        try:
            ik_angles = self.kin.inverse(*target_xyz, elbow_up)
            if ik_angles is None:
                print("[WARN] Solver nie znalazł rozwiązania IK (None).")
                return False
        except Exception as e:
            print(f"[ERROR] Błąd obliczeń IK: {e}")
            return False
        return ik_angles, current_servo_angles

