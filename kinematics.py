### robot_arm/kinematics.py

import math
from config import SERVO_TRIMS, SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_ANGLE_LIMITS

class Kinematics:
    def __init__(self, l1, l2):
        self.l1 = l1
        self.l2 = l2

    def inverse(self, x, y, z, elbow_up=True):
        phi = math.atan2(y, x)
        r = math.hypot(x, y)
        px = r
        py = z

        d = math.hypot(px, py)
        if d > (self.l1 + self.l2) or d < abs(self.l1 - self.l2):
            raise ValueError("Punkt poza zasięgiem")

        cos_theta2 = (px**2 + py**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        theta2 = math.acos(cos_theta2)
        if elbow_up:
            theta2 = -theta2

        k1 = self.l1 + self.l2 * math.cos(theta2)
        k2 = self.l2 * math.sin(theta2)
        theta1 = math.atan2(py, px) - math.atan2(k2, k1)

        phi_deg = math.degrees(phi)
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)

        return phi_deg, theta1_deg, theta2_deg

    def to_servo_angles(self, phi, theta1, theta2, apply_trim=True):
        s_base = 90 - phi 
        s_shoulder = 180 - theta1 
        s_elbow = 180 - (90 + theta2)

        print("[Inverse Kinematics — Punkt docelowy]")
        print(f"  Obrót podstawy (phi):   {phi:.2f}°")
        print(f"  Ramię 1 (theta1):        {theta1:.2f}°")
        print(f"  Ramię 2 (theta2):        {theta2:.2f}°")
        print("  Kąty serw (0-180°):")
        print(f"  Serwo 1 (baza):          {s_base:.2f}°")
        print(f"  Serwo 2 (ramię 1):       {s_shoulder:.2f}°")
        print(f"  Serwo 3 (ramię 2):       {s_elbow:.2f}°")

        angles = {
            SERVO_BASE_ID: s_base,
            SERVO_SHOULDER_ID: s_shoulder,
            SERVO_ELBOW_ID: s_elbow
        }

        # Walidacja przed trimem
        for sid, angle in angles.items():
            min_angle, max_angle = SERVO_ANGLE_LIMITS
            if not (min_angle <= angle <= max_angle):
                raise ValueError(f"Kąt serwa ID {sid} poza zakresem: {angle:.2f}° (przed trimem)")

        # Trim dodajemy dopiero po walidacji
        if apply_trim:
            for sid in angles:
                angles[sid] += SERVO_TRIMS.get(sid, 0.0)

        return angles[SERVO_BASE_ID], angles[SERVO_SHOULDER_ID], angles[SERVO_ELBOW_ID]
    
    