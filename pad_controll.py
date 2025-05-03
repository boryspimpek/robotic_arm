# hybrid_arm_controller.py

from ps4 import ArmPS4Controller as IKController
from ps42 import ArmPS4Controller as AngleController
import threading
import time
import sys

class HybridController:
    def __init__(self):
        self.mode = "ik"
        self.current_controller = None
        self.angle_controller = None
        self.ik_controller = None

    def start(self):
        print("[INFO] Uruchamianie kontrolera IK...")
        self.ik_controller = IKController(interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.ik_controller.on_x_press = self.switch_mode
        self.ik_controller.on_exit = self.exit_program
        self.current_controller = self.ik_controller
        self.ik_controller.listen()

    def switch_mode(self):
        if self.mode == "ik":
            print("[INFO] Przełączanie do trybu bezpośredniego sterowania kątami...")
            self.ik_controller.running = False
            self.ik_controller.control_thread.join()
            self.ik_controller.servo_controller.torque_off_all()

            self.mode = "angle"
            self.angle_controller = AngleController(interface="/dev/input/js0", connecting_using_ds4drv=False)
            # self.angle_controller.base_angle = 90
            # self.angle_controller.shoulder_angle = 90
            # self.angle_controller.elbow_angle = 90
            self.angle_controller.on_x_press = self.switch_mode
            self.angle_controller.on_exit = self.exit_program
            self.current_controller = self.angle_controller
            self.angle_controller.listen()

        else:
            print("[INFO] Przełączanie z powrotem do trybu IK...")
            self.angle_controller.running = False
            self.angle_controller.control_thread.join()
            self.angle_controller.servo_controller.torque_off_all()

            self.mode = "ik"
            self.ik_controller = IKController(interface="/dev/input/js0", connecting_using_ds4drv=False)
            self.ik_controller.on_x_press = self.switch_mode
            self.ik_controller.on_exit = self.exit_program
            self.current_controller = self.ik_controller
            self.ik_controller.listen()

    def exit_program(self):
        print("[INFO] Zamykanie programu (HybridController).")
        sys.exit(0)

if __name__ == "__main__":
    hybrid = HybridController()
    hybrid.start()
