import threading
import time
from pyPS4Controller.controller import Controller
from controller import ArmController
from kinematics_ikpy import IkpyKinematics
from servos import ServoController
from config import L1, L2, port


class ArmPS4Controller(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.kinematics = IkpyKinematics()
        self.servo_controller = ServoController(port)
        self.arm = ArmController(self.kinematics, self.servo_controller)

        self.x = 0.0
        self.y = 0.142
        self.z = 0.100
        
        self.joystick_rx = 0.0  # X-axis
        self.joystick_ly = 0.0  # Z-axis
        self.joystick_lz = 0.0  # Base rotation

        self.deadzone = 0.2  # Adjust this value as needed to define the deadzone

        self.running = True

        self.control_thread = threading.Thread(target=self.update_loop)
        self.control_thread.start()

        self.prev_position = (self.x, self.y, self.z)  # Store initial position

    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    def on_R3_up(self, val): self.joystick_lz = -self.apply_deadzone(val / 32767.0)
    def on_R3_down(self, val): self.joystick_lz = -self.apply_deadzone(val / 32767.0)
    def on_L3_left(self, val): self.joystick_rx = -self.apply_deadzone(val / 32767.0)
    def on_L3_right(self, val): self.joystick_rx = -self.apply_deadzone(val / 32767.0)
    def on_L3_up(self, val): self.joystick_ly = -self.apply_deadzone(val / 32767.0)
    def on_L3_down(self, val): self.joystick_ly = -self.apply_deadzone(val / 32767.0)
    def on_L3_y_at_rest(self): pass
    def on_L3_x_at_rest(self): pass
    def on_R3_right(self, value): pass
    def on_R3_left(self, value): pass
    def on_R3_x_at_rest(self): pass
    def on_R3_y_at_rest(self): pass
    def on_x_press(self): pass  # Zarezerwowane 


    def update_loop(self):
        refresh_rate = 0.005

        while self.running:
            delta_x = self.joystick_rx / 100
            delta_z = self.joystick_lz / 100
            delta_y = self.joystick_ly / 100
            

            # Update the arm position directly
            new_x = self.x + delta_x
            new_y = self.y + delta_y
            new_z = self.z + delta_z

            if self.arm.pad_ik_ikpy(new_x, new_y, new_z):
                self.x, self.y, self.z = new_x, new_y, new_z

                # Check if position has changed, if so, print the new position
                if (self.x, self.y, self.z) != self.prev_position:
                    print(f"Position: x={self.x*100:.3f}, y={self.y*100:.3f}, z={self.z*100:.3f}")
                    print(f"Delta X: {delta_x:.4f}, Delta Y: {delta_y:.4f}, Delta Z: {delta_z:.4f}")
                    self.prev_position = (self.x, self.y, self.z)  # Update the previous position

            time.sleep(refresh_rate)


if __name__ == "__main__":
    pad = ArmPS4Controller(interface=port, connecting_using_ds4drv=False)
    pad.listen()