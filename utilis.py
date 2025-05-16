from config import base, shoulder, elbow, wrist
from config import base_angle_limits, shoulder_angle_limits, elbow_angle_limits, wrist_angle_limits

class Utilis:
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

