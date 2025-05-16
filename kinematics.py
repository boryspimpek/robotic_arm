# robot_arm/kinematics.py
import numpy as np
import math
from config import base, shoulder, elbow, wrist, trims 
from utilis import Utilis

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

        theta3 = theta1 + theta2

        phi_deg = math.degrees(phi)
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        theta3_deg = math.degrees(theta3)
        print(f"[INFO] Inverse kinematics: phi={phi_deg:.2f}°, theta1={theta1_deg:.2f}°, theta2={theta2_deg:.2f}°, theta3={theta3_deg:.2f}°")
        return phi_deg, theta1_deg, theta2_deg, theta3_deg

    def forward(self, theta1_deg, theta2_deg):
        # Konwersja na radiany
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)

        # Pozycja końcówki ramienia
        x = self.l1 * math.cos(t1) + self.l2 * math.cos(t1 + t2)
        z = self.l1 * math.sin(t1) + self.l2 * math.sin(t1 + t2)

        # Zakładamy ramię w płaszczyźnie X/Z, y = 0
        return x, 0.0, z

    def to_servo_angles(self, phi, theta1, theta2, theta3, wrist_horizontal=True, apply_trim=True):
        angles = {
            base: 90 - phi,
            shoulder: 180 - theta1,
            elbow: -theta2 + 90,
            wrist: (90 if wrist_horizontal else 180) + theta3
        }

        angles = Utilis.validate_and_clip_angles(angles)

        if apply_trim:
            angles = Utilis.apply_trims(angles, trims)

        return angles[base], angles[shoulder], angles[elbow], angles[wrist]
