# robot_arm/kinematics.py
import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import time
import math
import matplotlib
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
import serial
import math
from config import trims, base, schoulder, elbow, angle_limits, wrist


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
        s_base = 90 - phi
        s_shoulder = 180 - theta1
        s_elbow = 180 - (90 + theta2) - 90
        s_wrist = (90 if wrist_horizontal else 180) + theta3

        angles = {
            base: s_base,
            schoulder: s_shoulder,
            elbow: s_elbow,
            wrist: s_wrist
        }

        # Walidacja przed trimem
        for sid, angle in angles.items():
            min_angle, max_angle = angle_limits
            if not (min_angle <= angle <= max_angle):
                if sid == wrist:
                    # Przycinamy tylko nadgarstek
                    clipped = max(min(angle, max_angle), min_angle)
                    print(f"[WARN] Kąt nadgarstka poza zakresem ({angle:.2f}°), przycinam do {clipped:.2f}°")
                    angles[sid] = clipped
                else:
                    raise ValueError(f"Kąt serwa ID {sid} poza zakresem: {angle:.2f}° (przed trimem)")

        # Trim dodajemy dopiero po walidacji
        if apply_trim:
            for sid in angles:
                angles[sid] += trims.get(sid, 0.0)

        return angles[base], angles[schoulder], angles[elbow], angles[wrist]

    def ikpy(self, target_position):
        my_chain = ikpy.chain.Chain.from_urdf_file(
            "robo.urdf", active_links_mask=[False, True, True, True, True]
        )
        target_orientation = [0, 0, 0]
        ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
        angles_deg = list(map(math.degrees, ik.tolist()))
        formatted_angles = [f"{angle:.2f}" for angle in angles_deg]
        selected_angles = formatted_angles[1:4]
        # print("Joint angles (in degrees):", selected_angles)
        return angles_deg[1:4]

    def to_servo_angles_ikpy(self, ik_angles_deg, wrist_horizontal=True, apply_trim=True):
        base_angle, shoulder_angle, elbow_angle = ik_angles_deg
        s_base = 90 - base_angle
        s_shoulder = 90 - shoulder_angle
        s_elbow = 90 - elbow_angle - 90
        s_wrist = 90 - (((s_shoulder-90) + s_elbow)-90)

        angles = {
            base: s_base,
            schoulder: s_shoulder,
            elbow: s_elbow,
            wrist: s_wrist
        }

        for sid, angle in angles.items():
            min_angle, max_angle = angle_limits
            if not (min_angle <= angle <= max_angle):
                clipped = max(min(angle, max_angle), min_angle)
                angles[sid] = clipped
                print(f"[WARN] Kąt serwa ID {sid} poza zakresem ({angle:.2f}°), przycinam do {clipped:.2f}°")

        # Trim dodajemy dopiero po walidacji
        if apply_trim:
            for sid in angles:
                angles[sid] += trims.get(sid, 0.0)
        return angles[base], angles[schoulder], angles[elbow], angles[wrist]

