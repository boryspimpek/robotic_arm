import math
import threading
import time
from pyPS4Controller.controller import Controller
from controller import ArmController
from kinematics import Kinematics
from servos import ServoController
from sc_controll import open_gripper, close_gripper
from config import L1, L2, port


def scaled_step(value, base_step, exponent):
    scaled = abs(value) ** exponent
    return math.copysign(base_step * scaled, value)


class ArmPS4Controller(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.kinematics = Kinematics(L1, L2)
        self.servo_controller = ServoController(port)
        self.arm = ArmController(self.kinematics, self.servo_controller)

        # Initial position
        self.x = 142.0
        self.z = 0.0
        self.phi = 0.0  # Base rotation

        # Input states
        self.joystick_rx = 0.0  # X-axis
        self.joystick_lz = 0.0  # Z-axis
        self.joystick_lphi = 0.0  # Base rotation
        self.deadzone = 0.5

        self.gripper_closed = True
        self.wrist_horizontal = True  # Default to horizontal

        self.running = True

        print("[INFO] Moving to start position...")
        self.arm.move_to_point_dps((self.x, 0.0, self.z), tempo_dps=60)

        self.control_thread = threading.Thread(target=self.update_loop)
        self.control_thread.start()

    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    # --- Joystick and button handlers ---
    def on_R3_up(self, val): self.joystick_lz = -self.apply_deadzone(val / 32767)
    def on_R3_down(self, val): self.joystick_lz = -self.apply_deadzone(val / 32767)
    def on_L3_up(self, val): self.joystick_rx = -self.apply_deadzone(val / 32767)
    def on_L3_down(self, val): self.joystick_rx = -self.apply_deadzone(val / 32767)
    def on_L3_left(self, val): self.joystick_lphi = -self.apply_deadzone(val / 32767)
    def on_L3_right(self, val): self.joystick_lphi = -self.apply_deadzone(val / 32767)
    def on_L3_y_at_rest(self): pass
    def on_L3_x_at_rest(self): pass
    def on_R3_right(self, value): pass
    def on_R3_left(self, value): pass
    def on_R3_x_at_rest(self): pass
    def on_R3_y_at_rest(self): pass
    def on_x_press(self): pass  # Zarezerwowane 

    def on_R1_press(self):
        self.gripper_closed = not self.gripper_closed
        action = "Closing" if self.gripper_closed else "Opening"
        print(f"[INFO] {action} gripper")
        close_gripper() if self.gripper_closed else open_gripper()

    def on_triangle_press(self):
        if self.wrist_horizontal and self.z < -10.0:
            print("[WARN] Too low to rotate wrist to vertical")
            return

        self.wrist_horizontal = not self.wrist_horizontal
        mode = "horizontal" if self.wrist_horizontal else "vertical"
        print(f"[INFO] Wrist mode changed to: {mode}")

    def on_circle_press(self):
        print("[INFO] Shutting down controller...")
        self.running = False
        self.control_thread.join()
        self.stop = True
        if hasattr(self, 'on_exit'):
            self.on_exit()

    # --- Helper functions ---
    def dynamic_base_step_x(self, x):
        return max(1, round(-0.0494 + 0.116 * x - 1.24E-03 * x**2 + 6.23E-06 * x**3 - 1.27E-08 * x**4))

    def dynamic_base_step_z(self, z):
        # return max(1, round(8 + 7.53e-3 * z - 3.67e-4 * z**2 + 4.7e-6 * z**3 - 2.1e-8 * z**4))
        return max(1, round(7.97 + -4.28E-03 * z + 1.21E-04 * z**2 + -9.23E-07 * z**3))    

    def calculate_z_thresholds(self):
        z_thresholds = []
        for i in range(1000):
            z = -100 + i * 0.1
            x = 6.59 - 3.9 * z - 0.0689 * z**2 - 4.81E-04 * z**3
            z_thresholds.append((z, x))
        return z_thresholds

    # --- Main control loop ---
    def update_loop(self):
        refresh_rate = 0.05
        z_thresholds = self.calculate_z_thresholds()

        while self.running:
            delta_x, delta_z, delta_phi = self.calculate_deltas()
            if delta_x or delta_z or delta_phi:
                self.update_position(delta_x, delta_z, delta_phi, z_thresholds)
            time.sleep(refresh_rate)

    def calculate_deltas(self):
        step_x = self.dynamic_base_step_x(self.x)
        step_z = self.dynamic_base_step_z(self.z)
        step_phi = 2  # Constant base step for rotation

        delta_x = scaled_step(self.joystick_rx, step_x, exponent=10.0)
        delta_z = scaled_step(self.joystick_lz, step_z, exponent=10.0)
        delta_phi = scaled_step(self.joystick_lphi, step_phi, exponent=5.0)

        return delta_x, delta_z, delta_phi

    def update_position(self, delta_x, delta_z, delta_phi, z_thresholds):
        new_x = self.calculate_new_x(delta_x, z_thresholds)
        new_z = self.calculate_new_z(delta_z)
        new_phi = self.calculate_new_phi(delta_phi)

        if self.arm.pad_ik_full(new_x, new_z, new_phi, cost_mode="inverted_vertical_end_effector"):
            self.x, self.z, self.phi = new_x, new_z, new_phi
            print(f"x={self.x:.1f}, z={self.z:.1f}, phi={self.phi:.1f}°")
            # print(f"[DEBUG] delta_x={delta_x:.1f}, delta_z={delta_z:.1f}, delta_phi={delta_phi:.1f}")
        else:
            print("[INFO] Movement not allowed.")

    def calculate_new_x(self, delta_x, z_thresholds):
        new_x = self.x + delta_x
        for z_limit, min_x in z_thresholds:
            if self.z <= z_limit:
                new_x = max(min_x, new_x)
                break
        else:
            new_x = max(10.0, new_x)
        return new_x

    def calculate_new_z(self, delta_z):
        z_limit = -50.0 if self.wrist_horizontal else -10.0
        return max(z_limit, self.z + delta_z)

    def calculate_new_phi(self, delta_phi):
        return max(-90, min(90, self.phi + delta_phi))

if __name__ == "__main__":
    pad = ArmPS4Controller(interface=port, connecting_using_ds4drv=False)
    pad.listen()
