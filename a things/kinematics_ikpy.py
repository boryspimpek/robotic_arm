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
from config import trims, base, shoulder, elbow, angle_limits, wrist

class IkpyKinematics:

    def ikpy_inverse(self, target_position):
        my_chain = ikpy.chain.Chain.from_urdf_file(
            "robo.urdf", active_links_mask=[False, True, True, True, True]
        )
        target_orientation = [0, 0, 0]
        ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
        angles_deg = list(map(math.degrees, ik.tolist()))
        formatted_angles = [f"{angle:.2f}" for angle in angles_deg]
        selected_angles = formatted_angles[1:5]
        # print("Joint angles (in degrees):", selected_angles)
        return angles_deg[1:5]

    def to_servo_angles_ikpy(self, ik_angles_deg, apply_trim=True):
        base_angle, shoulder_angle, elbow_angle, wrist_angle = ik_angles_deg
        s_base = 90 - base_angle
        s_shoulder = 90 - shoulder_angle
        s_elbow = - elbow_angle
        s_wrist = 90 - wrist_angle

        angles = {
            base: s_base,
            shoulder: s_shoulder,
            elbow: s_elbow,
            wrist: s_wrist
        }

        # Walidacja przed trimem
        for sid, angle in angles.items():
            min_angle, max_angle = angle_limits
            if not (min_angle <= angle <= max_angle):
                raise ValueError(f"Kąt serwa ID {sid} poza zakresem: {angle:.2f}° (przed trimem)")

        # Trim dodajemy dopiero po walidacji
        if apply_trim:
            for sid in angles:
                angles[sid] += trims.get(sid, 0.0)
        return angles[base], angles[shoulder], angles[elbow], angles[wrist]