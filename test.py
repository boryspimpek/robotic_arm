from servos import ServoController
from kinematics import Kinematics
from controller import ArmController
from config import L1, L2, UART_PORT
import time

kin = Kinematics(L1, L2)
servo = ServoController(UART_PORT)
arm = ArmController(kin, servo)

start = (142, 0, 0)
end = (190, 0, 0)

print("[TEST] Ruch 10mm przy tempo_dps = 200")
arm.move_xyz_sync(end, tempo_dps=200)
time.sleep(1)

print("[TEST] Ruch powrotny przy tempo_dps = 1000")
arm.move_xyz_sync(start, tempo_dps=1000)
time.sleep(1)
