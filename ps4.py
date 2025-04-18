from pyPS4Controller.controller import Controller
from controller import ArmController
from kinematics import Kinematics
from servos import ServoController
from config import L1, L2, UART_PORT
import threading
import time
import math

def scaled_step(raw_val, base_step=5.0, expo=2.0):
    scaled = abs(raw_val) ** expo
    return math.copysign(base_step * scaled, raw_val)


class ArmPS4Controller(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Inicjalizacja ramienia
        self.kin = Kinematics(L1, L2)
        self.servo = ServoController(UART_PORT)
        self.arm = ArmController(self.kin, self.servo)

        # Pozycja startowa
        self.x = 142.0
        self.y = 0.0
        self.z = 0.0
        self.step = 5.0  # podstawowy krok
        self.deadzone = 0.3
        self.running = True

        # Odczyty joysticków
        self.lx = 0.0  # Lewy X
        self.ly = 0.0  # Lewy Y
        self.rz = 0.0  # Prawy Z

        self.lz = 0.0      # ruch góra/dół (Z)
        self.lphi = 0.0    # obrót podstawy
        self.rx = 0.0      # ruch w osi X


        print("[INFO] Przechodzę do pozycji startowej...")
        self.arm.move_to_point_dps((self.x, self.y, self.z), tempo_dps=30)

        # Uruchamiamy wątek sterowania ruchem
        self.control_thread = threading.Thread(target=self.update_loop)
        self.control_thread.start()

    def apply_deadzone(self, val):
        return val if abs(val) > self.deadzone else 0.0
    
    # L3 – pion (Z)
    def on_L3_up(self, val): self.lz = -self.apply_deadzone(val / 32767)
    def on_L3_down(self, val): self.lz = -self.apply_deadzone(val / 32767)

    # L3 – obrót bazy (phi)
    def on_L3_left(self, val): self.lphi = -self.apply_deadzone(val / 32767)
    def on_L3_right(self, val): self.lphi = -self.apply_deadzone(val / 32767)

    # R3 – poziom (X)
    def on_R3_up(self, val): self.rx = -self.apply_deadzone(val / 32767)
    def on_R3_down(self, val): self.rx = -self.apply_deadzone(val / 32767)

    def update_loop(self):
        refresh_rate = 0.05  # 50 ms
        current_phi = 0.0

        while self.running:
            dx = scaled_step(self.rx, base_step=3.0, expo=4.0)      # oś X z R3
            dz = scaled_step(self.lz, base_step=3.0, expo=4.0)      # oś Z z L3
            dphi = scaled_step(self.lphi, base_step=2.0, expo=3.0)  # obrót z L3

            if dx != 0.0 or dz != 0.0 or dphi != 0.0:
                new_x = self.x + dx
                new_z = self.z + dz
                current_phi += dphi
                current_phi = max(-90, min(90, current_phi))  # ogranicz obrót

                success = self.arm.pad_controll(
                    new_x, new_z, current_phi, steps=5
                )

                if success:
                    self.x = new_x
                    self.z = new_z
                    print(f"x={self.x:.1f}, z={self.z:.1f}, phi={current_phi:.1f}°")
                else:
                    print("[INFO] Ruch niedozwolony.")

            time.sleep(refresh_rate)
    def on_circle_press(self):
        print("[INFO] Zamykanie...")
        self.running = False
        self.control_thread.join()
        self.servo.torque_off_all()
        self.stop = True

# Start kontrolera
if __name__ == "__main__":
    pad = ArmPS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
    pad.listen()
