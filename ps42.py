# ps42.py
from pyPS4Controller.controller import Controller
from controller import ArmController
from kinematics import Kinematics
from servos import ServoController
from config import L1, L2, SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, GLOBAL_SERVO_SPEED, GLOBAL_SERVO_ACC, SERVO_WRIST_ID
import threading
import time
import math

def scaled_step(raw_val, base_step=5.0, expo=2.0):
    scaled = abs(raw_val) ** expo
    return math.copysign(base_step * scaled, raw_val)

class ArmPS4Controller(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.kin = Kinematics(L1, L2)
        self.servo = ServoController("/dev/ttyACM0")
        self.arm = ArmController(self.kin, self.servo)

        # self.base_angle = 90
        # self.shoulder_angle = 90
        # self.elbow_angle = 90
        # self.wrist_angle = 0
        self.deadzone = 0.3
        self.running = True

        self.lx = 0.0
        self.ly = 0.0
        self.rz = 0.0
        
        x = 142.0
        y = 0.0
        z = 0.0
        print("[INFO] Przechodzę do pozycji startowej...")
        self.arm.move_to_point_dps((x, y, z), tempo_dps=30)

        angles = self.servo.get_all_servo_positions_deg([
            SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
        ])
        self.base_angle = angles.get(SERVO_BASE_ID, 90)
        self.shoulder_angle = angles.get(SERVO_SHOULDER_ID, 90)
        self.elbow_angle = angles.get(SERVO_ELBOW_ID, 90)
        self.wrist_angle = (180 - self.shoulder_angle) + (180 - self.elbow_angle)

        self.control_thread = threading.Thread(target=self.update_loop)
        self.control_thread.start()

    def apply_deadzone(self, val):
        return val if abs(val) > self.deadzone else 0.0

    def on_L3_down(self, val): self.ly = self.apply_deadzone(val / 32767)
    def on_L3_up(self, val): self.ly = self.apply_deadzone(val / 32767)
    def on_L3_left(self, val): self.lx = -self.apply_deadzone(val / 32767)
    def on_L3_right(self, val): self.lx = -self.apply_deadzone(val / 32767)
    def on_R3_down(self, val): self.rz = self.apply_deadzone(val / 32767)
    def on_R3_up(self, val): self.rz = self.apply_deadzone(val / 32767)

    def on_x_press(self):
        pass

    def update_loop(self):
        refresh_rate = 0.05

        while self.running:
            self.base_angle += scaled_step(self.lx, base_step=2.0, expo=5.0)
            self.shoulder_angle += scaled_step(self.ly, base_step=2.0, expo=5.0)
            self.elbow_angle += scaled_step(self.rz, base_step=2.0, expo=5.0)
            self.wrist_angle = (180 - self.shoulder_angle) + (180 - self.elbow_angle)

            self.base_angle = max(0, min(180, self.base_angle))
            self.shoulder_angle = max(0, min(180, self.shoulder_angle))
            self.elbow_angle = max(0, min(180, self.elbow_angle))

            self.servo.move_to({
                SERVO_BASE_ID: self.base_angle,
                SERVO_SHOULDER_ID: self.shoulder_angle,
                SERVO_ELBOW_ID: self.elbow_angle,
                SERVO_WRIST_ID: self.wrist_angle
            })

            time.sleep(refresh_rate)

    def on_circle_press(self):
        print("[INFO] Zamykanie...")
        self.running = False
        self.servo.torque_off_all()
        self.stop = True
        if hasattr(self, 'on_exit'):
            self.on_exit()

if __name__ == "__main__":
    pad = ArmPS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
    pad.listen()
