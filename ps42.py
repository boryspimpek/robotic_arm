from pyPS4Controller.controller import Controller
from controller import ArmController
from kinematics import Kinematics
from servos import ServoController
from config import (
    L1, L2,
    base, schoulder, elbow, wrist, port,
)

import threading
import time
import math

def scaled_step(value, base_step=5.0, exponent=2.0):
    scaled = abs(value) ** exponent
    return math.copysign(base_step * scaled, value)

class ArmPS4Controller(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Inicjalizacja komponentów
        self.kinematics = Kinematics(L1, L2)
        self.servo_controller = ServoController("/dev/ttyACM0")
        self.arm = ArmController(self.kinematics, self.servo_controller)

        # Stan kontrolera
        self.deadzone = 0.3
        self.running = True

        # Osie joysticka
        self.joystick_lx = 0.0  # obrót podstawy
        self.joystick_ly = 0.0  # ramię
        self.joystick_rz = 0.0  # łokieć

        # Pozycja startowa (dla czytelności, nie steruje kątami bezpośrednio)
        print("[INFO] Przechodzę do pozycji startowej...")
        self.arm.move_to_point_dps((142.0, 0.0, 0.0), tempo_dps=60)

        # Odczyt aktualnych kątów z serw
        angles = self.servo_controller.get_all_servo_positions_deg([
            base, schoulder, elbow, wrist
        ])
        self.base_angle = angles.get(base, 90)
        self.shoulder_angle = angles.get(schoulder, 90)
        self.elbow_angle = angles.get(elbow, 90)
        self.wrist_angle = 180 - self.shoulder_angle - self.elbow_angle + 90

        # Wątek sterowania
        self.control_thread = threading.Thread(target=self.update_loop)
        self.control_thread.start()

    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    # --- Mapa przycisków i joysticków ---

    def on_L3_left(self, val): self.joystick_lx = self.apply_deadzone(val / 32767)
    def on_L3_right(self, val): self.joystick_lx = self.apply_deadzone(val / 32767)
    def on_L3_up(self, val): self.joystick_ly = -self.apply_deadzone(val / 32767)
    def on_L3_down(self, val): self.joystick_ly = -self.apply_deadzone(val / 32767)
    def on_R3_up(self, val): self.joystick_rz = self.apply_deadzone(val / 32767)
    def on_R3_down(self, val): self.joystick_rz = self.apply_deadzone(val / 32767)

    def on_x_press(self):
        pass  # HybridController przypisuje tutaj switch_mode

    def on_circle_press(self):
        print("[INFO] Zamykanie...")
        self.running = False
        # self.servo_controller.torque_off_all()
        self.stop = True
        if hasattr(self, 'on_exit'):
            self.on_exit()

    # --- Główna pętla aktualizacji kątów ---

    def update_loop(self):
        refresh_rate = 0.05

        while self.running:
            # Aktualizacja kątów
            self.base_angle += scaled_step(self.joystick_lx, base_step=2.0, exponent=5.0)
            self.shoulder_angle += scaled_step(self.joystick_ly, base_step=2.0, exponent=5.0)
            self.elbow_angle += scaled_step(self.joystick_rz, base_step=2.0, exponent=5.0)
            
            # Wylicz nadgarstek (poziomy)
            self.wrist_angle = 180 - self.shoulder_angle - self.elbow_angle + 90

            # Ograniczenia mechaniczne
            self.base_angle = max(0, min(180, self.base_angle))
            self.shoulder_angle = max(0, min(180, self.shoulder_angle))
            self.elbow_angle = max(0, min(180, self.elbow_angle))

            # Wysłanie do serw
            self.servo_controller.move_to({
                base: self.base_angle,
                schoulder: self.shoulder_angle,
                elbow: self.elbow_angle,
                wrist: self.wrist_angle
            })

            time.sleep(refresh_rate)


if __name__ == "__main__":
    pad = ArmPS4Controller(interface=port, connecting_using_ds4drv=False)
    pad.listen()
