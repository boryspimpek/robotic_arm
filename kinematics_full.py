import math
from matplotlib import pyplot as plt
import numpy as np
from config import base, shoulder, elbow, wrist
from utilis import Utilis


class FullKinematics:
    def __init__(self, l1, l2, l3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.delta_deg = 1

    def calculate_positions_2d(self, theta1, theta2, theta3):
        x0, z0 = 0, 0
        x1 = self.l1 * np.cos(theta1)
        z1 = self.l1 * np.sin(theta1)
        x2 = x1 + self.l2 * np.cos(theta1 + theta2)
        z2 = z1 + self.l2 * np.sin(theta1 + theta2)
        x3 = x2 + self.l3 * np.cos(theta1 + theta2 + theta3)
        z3 = z2 + self.l3 * np.sin(theta1 + theta2 + theta3)
        return [(x0, z0), (x1, z1), (x2, z2), (x3, z3)]

    def plot_positions_2d(self, best_positions, x_target, z_target):
        if best_positions:
            xs, zs = zip(*best_positions)
            plt.figure(figsize=(6, 6))
            plt.plot(xs, zs, 'o-', linewidth=4, markersize=8)
            plt.plot(x_target, z_target, 'rx', markersize=10, label='Target')
            plt.axis('equal')
            plt.grid(True)
            plt.title("IK Solution – Elbow Up Configuration (Servo Limits 0°–180°)")
            plt.xlabel("X (mm)")
            plt.ylabel("Z (mm)")
            plt.legend()

    def solve_ik_2d(self, x_target, z_target):
        delta_theta = np.radians(self.delta_deg)
        theta3_candidates = np.arange(-np.pi, np.pi, delta_theta)

        min_cost = np.inf
        best_angles = None
        best_positions = None

        for theta3_c in theta3_candidates:
            wrist_x = x_target - self.l3 * np.cos(theta3_c)
            wrist_z = z_target - self.l3 * np.sin(theta3_c)

            D = np.hypot(wrist_x, wrist_z)
            if D > (self.l1 + self.l2) or D < abs(self.l1 - self.l2):
                continue

            cos_theta2 = (wrist_x**2 + wrist_z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            if not (-1 <= cos_theta2 <= 1):
                continue

            theta2 = -np.arccos(cos_theta2)  # Elbow-up
            k1 = self.l1 + self.l2 * np.cos(theta2)
            k2 = self.l2 * np.sin(theta2)
            theta1 = np.arctan2(wrist_z, wrist_x) - np.arctan2(k2, k1)
            theta3 = theta3_c - (theta1 + theta2)

            # Konwersja do zakresu serwa
            servo_angles = {
                "shoulder": 180 - np.degrees(theta1),
                "elbow": -np.degrees(theta2),
                "wrist": 90 - np.degrees(theta3),
            }

            if any(not (0 <= angle <= 180) for angle in servo_angles.values()):
                continue

            cost = theta1**2 + theta2**2 + theta3**2            
            # cost = (theta2)**2 + (theta3)**2

            if cost < min_cost:
                min_cost = cost
                best_angles = (theta1, theta2, theta3)
                best_angles_deg = (180 - np.degrees(theta1), -np.degrees(theta2), 90 - np.degrees(theta3))
                best_positions = self.calculate_positions_2d(theta1, theta2, theta3)

        if best_positions:
            self.plot_positions_2d(best_positions, x_target, z_target)

        print(f"Best angles: {best_angles}")
        print(f"Best angles (degrees): {best_angles_deg}")
        return best_angles, best_angles_deg, best_positions

    def ik_2d_to_servo_angles(self, angles):
        if angles is None:
            return None

        theta1, theta2, theta3 = angles
        servo_angles = {
            shoulder: 180 - np.degrees(theta1),
            elbow: -1 * np.degrees(theta2),
            wrist: 90 - np.degrees(theta3),
        }

        for sid, angle in servo_angles.items():
            if not (0 <= angle <= 180):
                raise ValueError(f"Kąt serwa '{sid}' poza zakresem: {angle:.2f}°")
        return servo_angles
    
    def calculate_positions_3d(self, theta0, theta1, theta2, theta3):
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

    def plot_positions_3d(self, best_positions, x_target, y_target, z_target):
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

    def solve_ik_3d(self, x_target, y_target, z_target, cost_mode):
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

            # Konwersja do zakresu serwa
            servo_angles = {
                "base": np.degrees(theta0) + 90,
                "shoulder": 180 - np.degrees(theta1),
                "elbow": -np.degrees(theta2),
                "wrist": 90 - np.degrees(theta3),
            }

            if any(not (0 <= angle <= 180) for angle in servo_angles.values()):
                continue

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
                cost = (end_orientation)**2  # minimalizujemy odchylenie końcówki od 0°
            elif cost_mode == "standard":
                cost = (theta2)**2 + (theta3)**2
            else:
                raise ValueError("Nieznany tryb kosztu")
            
            if cost < min_cost:
                min_cost = cost
                best_angles = (theta0, theta1, theta2, theta3)
                best_positions = self.calculate_positions_3d(theta0, theta1, theta2, theta3)
            
        if best_positions:
            self.plot_positions_3d(best_positions, x_target, y_target, z_target)

        # print(f"Best angles: {best_angles}")
        # print(f"Best angles (degrees): {best_angles_deg}")
        return best_angles, best_positions

    def ik_3d_to_servo_angles(self, best_angles):
        theta0, theta1, theta2, theta3 = best_angles
        servo_angles = {
            base: np.degrees(theta0) + 90,
            shoulder: 180 - np.degrees(theta1),
            elbow: -1 * np.degrees(theta2),
            wrist: 90 - np.degrees(theta3)
        }

        servo_angles = Utilis.validate_and_clip_angles(servo_angles)
        return servo_angles