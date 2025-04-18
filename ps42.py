from pyPS4Controller.controller import Controller
from servos import ServoController
from config import SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, GLOBAL_SERVO_SPEED, GLOBAL_SERVO_ACC
import threading
import time
import math

def scaled_step(raw_val, base_step=5.0, expo=2.0):
    scaled = abs(raw_val) ** expo  # Zastosowanie wykładnika na wartości
    return math.copysign(base_step * scaled, raw_val)  # Zachowujemy znak oryginalnej wartości


class ArmPS4Controller(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Inicjalizacja sterowania serwami
        self.servo = ServoController("/dev/ttyACM0")  # Właściwy port do sterowania
        self.base_angle = 90  # Początkowy kąt podstawy (obrót)
        self.shoulder_angle = 90  # Początkowy kąt ramienia 1
        self.elbow_angle = 90  # Początkowy kąt ramienia 2
        self.deadzone = 0.3  # Strefa martwa dla analogu

        # Odczyty joysticków
        self.lx = 0.0  # Lewy X
        self.ly = 0.0  # Lewy Y
        self.rz = 0.0  # Prawy Z

        # Uruchamiamy wątek sterowania ruchem
        self.control_thread = threading.Thread(target=self.update_loop)
        self.control_thread.start()

    def apply_deadzone(self, val):
        return val if abs(val) > self.deadzone else 0.0
    
    # L3 – pion (Z)
    def on_L3_down(self, val): self.ly = self.apply_deadzone(val / 32767)
    def on_L3_up(self, val): self.ly = self.apply_deadzone(val / 32767)

    # L3 – obrót bazy (phi)
    def on_L3_left(self, val): self.lx = -self.apply_deadzone(val / 32767)
    def on_L3_right(self, val): self.lx = -self.apply_deadzone(val / 32767)

    # R3 – poziom (X)
    def on_R3_down(self, val): self.rz = self.apply_deadzone(val / 32767)
    def on_R3_up(self, val): self.rz = self.apply_deadzone(val / 32767)

    def update_loop(self):
        refresh_rate = 0.05  # 50 ms

        while True:
            # Sterowanie kątami serw za pomocą joysticków
            self.base_angle += scaled_step(self.lx, base_step=2.0, expo=5.0)  # Przesuwanie bazy (obrót)
            self.shoulder_angle += scaled_step(self.ly, base_step=2.0, expo=5.0)  # Ramię 1
            self.elbow_angle += scaled_step(self.rz, base_step=2.0, expo=5.0)  # Ramię 2

            # Ograniczenie kątów serw (zakres 0°-180°)
            self.base_angle = max(0, min(180, self.base_angle))
            self.shoulder_angle = max(0, min(180, self.shoulder_angle))
            self.elbow_angle = max(0, min(180, self.elbow_angle))

            # Przesyłanie nowych kątów do serw
            self.servo.move_to({
                SERVO_BASE_ID: self.base_angle,
                SERVO_SHOULDER_ID: self.shoulder_angle,
                SERVO_ELBOW_ID: self.elbow_angle
            })

            # Opóźnienie odświeżania
            time.sleep(refresh_rate)

    def on_circle_press(self):
        print("[INFO] Zamykanie...")
        self.servo.torque_off_all()
        self.stop = True

# Start kontrolera
if __name__ == "__main__":
    pad = ArmPS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
    pad.listen()
