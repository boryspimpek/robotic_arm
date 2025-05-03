import json
import time
import sys
from kinematics import Kinematics
from sc_controll import sc_servo
from servos import ServoController
from config import L1, L2, SC_SERVO_ACC, SC_SERVO_SPEED, SERVO_BASE_ID, SERVO_ELBOW_ID, SERVO_GRIPPER_ID, SERVO_SHOULDER_ID, SERVO_WRIST_ID, UART_PORT
from controller import ArmController
servo_ctrl = ServoController(UART_PORT)
kin = Kinematics(L1, L2)
controller = ArmController(kin, servo_ctrl)

TEMPO_DPS = 50.0  # prędkość ruchu serw w °/s

try:
    servo = ServoController(UART_PORT)

    with open('recorded_positions.json', 'r') as f:
        positions = json.load(f)

    last_pose = servo_ctrl.get_all_servo_positions_deg([
        SERVO_BASE_ID, SERVO_SHOULDER_ID, SERVO_ELBOW_ID, SERVO_WRIST_ID
    ])
    
    for key in sorted(positions.keys()):
        print(f"Odtwarzam: {key} → {positions[key]}")
        angles = {int(k): v for k, v in positions[key].items()}
        
        # Oddziel ST (ramię) od SC (chwytak)
        arm_angles = {
            SERVO_BASE_ID: angles[SERVO_BASE_ID],
            SERVO_SHOULDER_ID: angles[SERVO_SHOULDER_ID],
            SERVO_ELBOW_ID: angles[SERVO_ELBOW_ID],
            SERVO_WRIST_ID: angles[SERVO_WRIST_ID]
        }
        gripper_angle = angles[SERVO_GRIPPER_ID]

        servo_ctrl.sync_angles(last_pose, arm_angles, tempo_dps=TEMPO_DPS)
        sc_servo(SERVO_GRIPPER_ID, gripper_angle, SC_SERVO_SPEED, SC_SERVO_ACC)

        # oblicz czas ruchu
        delta = max(abs(arm_angles[sid] - last_pose.get(sid, arm_angles[sid])) for sid in arm_angles)
        move_time = delta / TEMPO_DPS

        time.sleep(move_time + 0.1)

        last_pose = arm_angles

    print("Wszystkie pozycje odtworzone.")
    sys.exit(0)

except Exception as e:
    print(f"Błąd odtwarzania: {e}")
    sys.exit(1)
