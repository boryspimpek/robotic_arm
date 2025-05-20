import math
from matplotlib import pyplot as plt
import numpy as np
from config import L1, L2, L3, base, shoulder, elbow, wrist, base_angle_limits, shoulder_angle_limits, elbow_angle_limits, wrist_angle_limits 
from utilis import Utilis


class FullKinematics:
    def __init__(self, l1, l2, l3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.delta_deg = 1

    def calculate_positions_full(self, theta0, theta1, theta2, theta3):
        x0, y0, z0 = 0, 0, 0
        x1 = self.l1 * np.cos(theta1) * np.cos(theta0)
        y1 = self.l1 * np.cos(theta1) * np.sin(theta0)
        z1 = self.l1 * np.sin(theta1)

        x2 = x1 + self.l2 * np.cos(theta1 + theta2) * np.cos(theta0)
        y2 = y1 + self.l2 * np.cos(theta1 + theta2) * np.sin(theta0)
        z2 = z1 + self.l2 * np.sin(theta1 + theta2)

        x3 = x2 + self.l3 * np.cos(theta1 + theta2 + theta3) * np.cos(theta0)
        y3 = y2 + self.l3 * np.cos(theta1 + theta2 + theta3) * np.sin(theta0)
        z3 = z2 + self.l3 * np.sin(theta1 + theta2 + theta3)

        return [(x0, y0, z0), (x1, y1, z1), (x2, y2, z2), (x3, y3, z3)]

    def plot_positions_full(self, best_positions, x_target, y_target, z_target):
        if best_positions:
            xs, ys, zs = zip(*best_positions)
            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(xs, ys, zs, 'o-', linewidth=4, markersize=8, label='Arm')
            ax.scatter([x_target], [y_target], [z_target], color='r', s=100, label='Target')
            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")
            ax.set_zlabel("Z (mm)")
            ax.set_title("IK – 3D Arm with Base Rotation (Elbow Up)")
            ax.grid(True)
            ax.set_xlim(0, 300)
            ax.set_ylim(-300, 300)
            ax.set_zlim(0, 300)

    def solve_ik_full(self, x_target, y_target, z_target, cost_mode):
        delta_theta = np.radians(self.delta_deg)
        theta3_candidates = np.arange(-np.pi, np.pi, delta_theta)

        r_target = np.hypot(x_target, y_target)
        px = r_target
        py = z_target

        d = math.hypot(px, py)
        if d > (self.l1 + self.l2 + self.l3):
            raise ValueError("Punkt poza zasięgiem")

        theta0 = np.arctan2(y_target, x_target)

        min_cost = np.inf
        best_angles = None
        best_positions = None

        for theta3_c in theta3_candidates:
            wrist_r = r_target - self.l3 * np.cos(theta3_c)
            wrist_z = z_target - self.l3 * np.sin(theta3_c)

            D = np.hypot(wrist_r, wrist_z)
            if D > (self.l1 + self.l2) or D < abs(self.l1 - self.l2):
                continue

            cos_theta2 = (wrist_r**2 + wrist_z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            if not (-1 <= cos_theta2 <= 1):
                continue

            theta2 = -np.arccos(cos_theta2)  # Elbow-up
            k1 = self.l1 + self.l2 * np.cos(theta2)
            k2 = self.l2 * np.sin(theta2)
            theta1 = np.arctan2(wrist_z, wrist_r) - np.arctan2(k2, k1)
            theta3 = theta3_c - (theta1 + theta2)

            if cost_mode == "min_angle_sum":
                cost = theta0**2 + theta1**2 + theta2**2 + theta3**2
            elif cost_mode == "vertical_up":
                end_orientation = theta1 + theta2 + theta3
                cost = (end_orientation - np.pi/2)**2    
            elif cost_mode == "vertical_down":
                end_orientation = theta1 + theta2 + theta3
                cost = (end_orientation + np.pi/2)**2            
            elif cost_mode == "flat":
                end_orientation = theta1 + theta2 + theta3
                cost = (end_orientation)**2  
            elif cost_mode == "normal":
                cost = (theta2)**2 + (theta3)**2
            else:
                raise ValueError("Nieznany tryb kosztu")
            
            if cost < min_cost:
                min_cost = cost

                phi_deg = np.degrees(theta0)
                theta1_deg = np.degrees(theta1)
                theta2_deg = np.degrees(theta2)
                theta3_deg = np.degrees(theta3)

                best_angles = (phi_deg, theta1_deg, theta2_deg, theta3_deg)
                best_positions = self.calculate_positions_full(theta0, theta1, theta2, theta3)
            
        if best_positions:
            self.plot_positions_full(best_positions, x_target, y_target, z_target)

        return best_angles, best_positions

    def forward_ik_full(self, angles):
        
        theta1_deg = angles[shoulder]
        theta2_deg = angles[elbow]
        theta3_deg = angles[wrist]  

        # Convert angles to degrees        
        # print(f"Angles in degrees: theta1 = {theta1_deg:.2f}, theta2 = {theta2_deg:.2f}, theta3 = {theta3_deg:.2f}")

        # Convert angles to radians
        theta1 = np.radians(180 - theta1_deg)
        theta2 = np.radians(- theta2_deg + 140)
        theta3 = np.radians(- theta3_deg + 130)

        # Base joint position
        x0, z0 = 0, 0

        # First joint
        x1 = x0 + L1 * np.cos(theta1)
        z1 = z0 + L1 * np.sin(theta1)
        print(f"Joint 1 position: x1 = {x1:.2f}, z1 = {z1:.2f}")

        # Second joint
        x2 = x1 + L2 * np.cos(theta1 + theta2)
        z2 = z1 + L2 * np.sin(theta1 + theta2)
        print(f"Joint 2 position: x2 = {x2:.2f}, z2 = {z2:.2f}")

        # End effector
        x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
        z3 = z2 + L3 * np.sin(theta1 + theta2 + theta3)
        print(f"End Effector position: x3 = {x3:.2f}, z3 = {z3:.2f}")

        return x2, z2, x3, z3