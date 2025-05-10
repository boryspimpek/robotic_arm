from matplotlib import pyplot as plt
from config import base, shoulder, elbow, wrist
import numpy as np


def calculate_positions_2d(theta1, theta2, theta3, L1, L2, L3):
    x0, z0 = 0, 0
    x1 = L1 * np.cos(theta1)
    z1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    z2 = z1 + L2 * np.sin(theta1 + theta2)
    x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    z3 = z2 + L3 * np.sin(theta1 + theta2 + theta3)
    return [(x0, z0), (x1, z1), (x2, z2), (x3, z3)]


def plot_positions_2d(best_positions, x_target, z_target):
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


def solve_ik_2d(x_target, z_target, L1=100, L2=100, L3=100, delta_deg=1):
    delta_theta = np.radians(delta_deg)
    theta3_candidates = np.arange(-np.pi, np.pi, delta_theta)

    min_cost = np.inf
    best_angles = None
    best_positions = None

    for theta3_c in theta3_candidates:
        wrist_x = x_target - L3 * np.cos(theta3_c)
        wrist_z = z_target - L3 * np.sin(theta3_c)

        D = np.hypot(wrist_x, wrist_z)
        if D > (L1 + L2) or D < abs(L1 - L2):
            continue

        cos_theta2 = (wrist_x**2 + wrist_z**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if not (-1 <= cos_theta2 <= 1):
            continue

        theta2 = -np.arccos(cos_theta2)  # Elbow-up
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(wrist_z, wrist_x) - np.arctan2(k2, k1)
        theta3 = theta3_c - (theta1 + theta2)


        cost = theta1**2 + theta2**2 + theta3**2
        if cost < min_cost:
            min_cost = cost
            best_angles = (theta1, theta2, theta3)
            best_positions = calculate_positions_2d(theta1, theta2, theta3, L1, L2, L3)

    if best_positions:
        plot_positions_2d(best_positions, x_target, z_target)

    return best_angles, best_positions


def calculate_positions_3d(theta0, theta1, theta2, theta3, L1, L2, L3):
    x0, y0, z0 = 0, 0, 0
    x1 = L1 * np.cos(theta1) * np.cos(theta0)
    y1 = L1 * np.cos(theta1) * np.sin(theta0)
    z1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2) * np.cos(theta0)
    y2 = y1 + L2 * np.cos(theta1 + theta2) * np.sin(theta0)
    z2 = z1 + L2 * np.sin(theta1 + theta2)

    x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3) * np.cos(theta0)
    y3 = y2 + L3 * np.cos(theta1 + theta2 + theta3) * np.sin(theta0)
    z3 = z2 + L3 * np.sin(theta1 + theta2 + theta3)

    return [(x0, y0, z0), (x1, y1, z1), (x2, y2, z2), (x3, y3, z3)]


def plot_positions_3d(best_positions, x_target, y_target, z_target):
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


def solve_ik_3d(x_target, y_target, z_target, L1=120, L2=120, L3=70, delta_deg=1):
    delta_theta = np.radians(delta_deg)
    theta3_candidates = np.arange(-np.pi, np.pi, delta_theta)

    r_target = np.hypot(x_target, y_target)
    theta0 = np.arctan2(y_target, x_target)

    min_cost = np.inf
    best_angles = None
    best_positions = None

    for theta3_c in theta3_candidates:
        wrist_r = r_target - L3 * np.cos(theta3_c)
        wrist_z = z_target - L3 * np.sin(theta3_c)

        D = np.hypot(wrist_r, wrist_z)
        if D > (L1 + L2) or D < abs(L1 - L2):
            continue

        cos_theta2 = (wrist_r**2 + wrist_z**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if not (-1 <= cos_theta2 <= 1):
            continue

        theta2 = -np.arccos(cos_theta2)  # Elbow-up
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(wrist_z, wrist_r) - np.arctan2(k2, k1)
        theta3 = theta3_c - (theta1 + theta2)

        # cost = theta0**2 + theta1**2 + theta2**2 + theta3**2
        cost = (theta2)**2 + (theta3)**2

        if cost < min_cost:
            min_cost = cost
            best_angles = (theta0, theta1, theta2, theta3)
            best_positions = calculate_positions_3d(theta0, theta1, theta2, theta3, L1, L2, L3)

    if best_positions:
        plot_positions_3d(best_positions, x_target, y_target, z_target)

    return best_angles, best_positions


def ik_to_servo_angles(angles):
    if angles is None:
        return None

    theta0, theta1, theta2, theta3 = angles
    servo_angles = {
        base: np.degrees(theta0) + 90,
        shoulder: 180 - np.degrees(theta1),
        elbow: -1 * np.degrees(theta2),
        wrist: 90 - np.degrees(theta3),
    }

    # Clamp to range 0–180°
    return {sid: min(max(angle, 0), 180) for sid, angle in servo_angles.items()}


if __name__ == "__main__":
    angles, positions = solve_ik_3d(150, 0, 200)
    end_angles = ik_to_servo_angles(angles)

    if end_angles:
        print("\nServo Angles:")
        for sid, angle in end_angles.items():
            print(f"  Servo {sid}: {angle:.2f}°")

    plt.show()
