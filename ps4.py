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

        self.wrist_horizontal = True  # Domyślnie poziomo

        # Pozycja początkowa
        self.x = 142.0
        self.z = 0.0
        self.phi = 0.0  # obrót podstawy
        self.deadzone = 0.5

        # Stany wejść
        self.joystick_rx = 0.0  # oś X
        self.joystick_lz = 0.0  # oś Z
        self.joystick_lphi = 0.0  # obrót podstawy

        self.gripper_closed = True
        self.running = True

        print("[INFO] Przechodzę do pozycji startowej...")
        self.arm.move_to_point_dps((self.x, 0.0, self.z), tempo_dps=60)

        self.control_thread = threading.Thread(target=self.update_loop)
        self.control_thread.start()

    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    # --- Obsługa przycisków i joysticków ---

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
    def on_x_press(self): pass  # Zarezerwowane na przyszłość

    def on_R1_press(self):
        self.gripper_closed = not self.gripper_closed
        if self.gripper_closed:
            print("[INFO] Zamykam chwytak")
            close_gripper()
        else:
            print("[INFO] Otwieram chwytak")
            open_gripper()

    def on_triangle_press(self):
        if self.wrist_horizontal and self.z < -10.0:
            print("[WARN] Za nisko, nie można obrócić nadgarstka do pionu")
            return

        self.wrist_horizontal = not self.wrist_horizontal
        mode = "poziomy" if self.wrist_horizontal else "pionowy"
        print(f"[INFO] Zmieniono tryb nadgarstka na: {mode}")

    def on_circle_press(self):
        print("[INFO] Zamykanie kontrolera...")
        self.running = False
        self.control_thread.join()
        # self.servo_controller.torque_off_all()
        self.stop = True
        if hasattr(self, 'on_exit'):
            self.on_exit()

    # --- Funkcje pomocnicze ---

    def dynamic_base_step_x(self, x):
        return max(1, round(-0.0494 + 0.116 * x - 1.24E-03* x**2 + 6.23E-06 * x**3 - 1.27E-08 * x**4))

    def dynamic_base_step_z(self, z):
        return max(1, round(8 + 7.53e-3 * z - 3.67e-4 * z**2 + 4.7e-6 * z**3 - 2.1e-8 * z**4))

    # --- Główna pętla sterowania ---

    def update_loop(self):
        refresh_rate = 0.05

        while self.running:
            step_x = self.dynamic_base_step_x(self.x)
            step_z = self.dynamic_base_step_z(self.z)
            step_phi = 2  # stała wartość bazowa dla obrotu podstawy

            delta_x = scaled_step(self.joystick_rx, step_x, exponent=5.0)
            delta_z = scaled_step(self.joystick_lz, step_z, exponent=5.0)
            delta_phi = scaled_step(self.joystick_lphi, step_phi, exponent=5.0)

            # Jeśli jest zmiana
            if delta_x or delta_z or delta_phi:
                # Definiujemy początkową wartość dla z oraz krok
                start_z = -50
                step_z = 0.1  # Zmiana w z_limit
                num_points = 500  # Liczba punktów do wygenerowania

                # Generujemy z_thresholds na podstawie wzoru dla x
                z_thresholds = []
                for i in range(num_points):
                    z = start_z + i * step_z  # Obliczamy wartość z w każdej iteracji
                    # Obliczamy x na podstawie wzoru
                    x = 6.59 - 3.9 * z - 0.0689 * z**2 - 4.81E-04 * z**3
                    z_thresholds.append((z, x))  # Dodajemy punkt (z, x) do listy

                # Obliczamy nową wartość x na podstawie delta_x
                new_x = self.x + delta_x

                # Iterujemy przez z_thresholds, aby dostosować nową wartość x
                for z_limit, min_x in z_thresholds:
                    if self.z <= z_limit:  # Jeśli aktualna wartość z jest mniejsza lub równa limitowi
                        new_x = max(min_x, new_x)  # Ustawiamy new_x na maksymalną z wartości: min_x i current new_x
                        break  # Przerywamy pętlę, ponieważ znaleźliśmy odpowiedni limit

                # Jeśli nie znaleziono żadnego limitu z_thresholds, ustawiamy minimalną wartość new_x
                else:
                    new_x = max(5.0, new_x)

                # Ustal maksymalne ograniczenie dla Z w zależności od trybu nadgarstka
                z_limit = -50.0 if self.wrist_horizontal else -10.0
                new_z = max(z_limit, self.z + delta_z)

                new_phi = max(-90, min(90, self.phi + delta_phi))

                if self.arm.pad_ik(new_x, new_z, new_phi, wrist_horizontal=self.wrist_horizontal):
                    self.x = new_x
                    self.z = new_z
                    self.phi = new_phi

                    print(f"x={self.x:.1f}, z={self.z:.1f}, phi={self.phi:.1f}°")
                    print(f"[DEBUG] delta_x={delta_x:.1f}, delta_z={delta_z:.1f}, delta_phi={delta_phi:.1f}")
                    # print(f"[DEBUG] step_x={step_x}, step_z={step_z}")
                    # print(f"[DEBUG] dynamic_base_step_z: {z} -> {step_z}")
                    # print(f"[DEBUG] dynamic_base_step_x: {x} -> {step_x}")
                else:
                    print("[INFO] Ruch niedozwolony.")

            time.sleep(refresh_rate)


if __name__ == "__main__":
    pad = ArmPS4Controller(interface=port, connecting_using_ds4drv=False)
    pad.listen()
