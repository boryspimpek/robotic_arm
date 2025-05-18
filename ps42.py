import math
import threading
import time
from pyPS4Controller.controller import Controller
from config import L1, L2, L3, base, shoulder, elbow, wrist, port_bus
from config import base_angle_limits, shoulder_angle_limits, elbow_angle_limits, wrist_angle_limits
from controller import ArmController
from kinematics import Kinematics
from kinematics_full import FullKinematics
from servos import ServoController
from utilis import Utilis

def scaled_step(value, base_step=5.0, exponent=2.0):
    scaled = abs(value) ** exponent
    return math.copysign(base_step * scaled, value)

class ArmPS4Controller(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._initialize_components()
        self._initialize_state()
        self._start_control_thread()

    def _initialize_components(self):
        self.kinematics = Kinematics(L1, L2)
        self.servo_controller = ServoController(port_bus)
        self.arm = ArmController(self.kinematics, self.servo_controller)
        self.utilis = Utilis(self.servo_controller, self.kinematics)
        self.fullkin = FullKinematics(L1, L2, L3)

    def _initialize_state(self):
        self.deadzone = 0.3
        self.running = True
        self.joystick_lx = 0.0  # Base rotation
        self.joystick_ly = 0.0  # Shoulder
        self.joystick_rz = 0.0  # Elbow
        self.joystick_rw = 0.0  # Wrist
        self._initialize_angles()

    def _initialize_angles(self):
        angles = self.servo_controller.get_positions([base, shoulder, elbow, wrist])
        self.base_angle = angles.get(base, 90)
        self.shoulder_angle = angles.get(shoulder, 90)
        self.elbow_angle = angles.get(elbow, 90)
        self.wrist_angle = angles.get(wrist, 90)

    def _start_control_thread(self):
        self.control_thread = threading.Thread(target=self._update_loop)
        self.control_thread.start()

    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    def on_L3_left(self, val): self.joystick_lx = self.apply_deadzone(val / 32767)
    def on_L3_right(self, val): self.joystick_lx = self.apply_deadzone(val / 32767)
    def on_R3_up(self, val): self.joystick_ly = self.apply_deadzone(val / 32767)
    def on_R3_down(self, val): self.joystick_ly = self.apply_deadzone(val / 32767)
    def on_L3_up(self, val): self.joystick_rz = self.apply_deadzone(val / 32767)
    def on_L3_down(self, val): self.joystick_rz = self.apply_deadzone(val / 32767)
    def on_L3_y_at_rest(self): pass
    def on_L3_x_at_rest(self): pass
    def on_R3_right(self, val): self.joystick_rw = self.apply_deadzone(val / 32767)
    def on_R3_left(self, val): self.joystick_rw = self.apply_deadzone(val / 32767)
    def on_R3_x_at_rest(self): pass
    def on_R3_y_at_rest(self): pass
    def on_x_press(self): pass  # Reserved for HybridController

    def on_circle_press(self):
        print("[INFO] Stopping...")
        self.running = False
        if hasattr(self, 'on_exit'):
            self.on_exit()

    def _update_loop(self):
        refresh_rate = 0.05

        while self.running:
            self._update_angles()
            self._apply_mechanical_limits()
            self._send_servo_commands()
            time.sleep(refresh_rate)

    def _update_angles(self):
        self.base_angle += scaled_step(self.joystick_lx, base_step=2.0, exponent=5.0)
        self.shoulder_angle += scaled_step(self.joystick_ly, base_step=2.0, exponent=5.0)
        self.elbow_angle += scaled_step(self.joystick_rz, base_step=2.0, exponent=5.0)
        self.wrist_angle += scaled_step(self.joystick_rw, base_step=2.0, exponent=5.0) 

    def _apply_mechanical_limits(self):
        angles = {
            base: self.base_angle,
            shoulder: self.shoulder_angle,
            elbow: self.elbow_angle,
            wrist: self.wrist_angle
        }

        angle_limits_per_servo = {
            base: base_angle_limits,
            shoulder: shoulder_angle_limits,
            elbow: elbow_angle_limits,
            wrist: wrist_angle_limits
        }

        for sid in angles:
            min_val, max_val = angle_limits_per_servo[sid]
            angles[sid] = min(max(angles[sid], min_val), max_val)

        # przypisanie z powrotem
        self.base_angle = angles[base]
        self.shoulder_angle = angles[shoulder]
        self.elbow_angle = angles[elbow]
        self.wrist_angle = angles[wrist]

    def _send_servo_commands(self):
        self.servo_controller.safe_move_to({
            base: self.base_angle,
            shoulder: self.shoulder_angle,
            elbow: self.elbow_angle,
            wrist: self.wrist_angle
        })

if __name__ == "__main__":
    pad = ArmPS4Controller(interface=port_bus, connecting_using_ds4drv=False)
    pad.listen()
