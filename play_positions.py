import os
import json
import time
import sys
from kinematics import Kinematics
from kinematics_full import FullKinematics
from sc_controll import sc_servo
from servos import ServoController
from config import L1, L2, L3, sc_acc, sc_speed, base, elbow, gripper, shoulder, wrist, port_bus
from controller import ArmController

fullkin = FullKinematics(L1, L2, L3)
servo_ctrl = ServoController(port_bus)
kin = Kinematics(L1, L2)
controller = ArmController(kin, servo_ctrl, fullkin)

TEMPO_DPS = 50.0  # prędkość ruchu serw w °/s

try:
    servo = ServoController(port_bus)

    with open('recorded_positions.json', 'r') as f:
        positions = json.load(f)
    
    if os.path.exists('play_done.txt'):
        os.remove('play_done.txt')


    last_pose = servo_ctrl.get_positions([
        base, shoulder, elbow, wrist
    ])
    
    for i, key in enumerate(sorted(positions.keys())):
        print(f"Odtwarzam: {key} → {positions[key]}")
        angles = {int(k): v for k, v in positions[key].items()}
        
        arm_angles = {
            base: angles[base],
            shoulder: angles[shoulder],
            elbow: angles[elbow],
            wrist: angles[wrist]
        }
        gripper_angle = angles[gripper]

        servo_ctrl.sync_angles(last_pose, arm_angles, tempo_dps=TEMPO_DPS)
        sc_servo(gripper, gripper_angle, sc_speed, sc_acc)

        delta = max(abs(arm_angles[sid] - last_pose.get(sid, arm_angles[sid])) for sid in arm_angles)
        move_time = delta / TEMPO_DPS

        time.sleep(move_time + 0.3)
        last_pose = arm_angles

    # ✅ Dopiero po całej pętli:
    with open('play_done.txt', 'w') as f:
        f.write('done')

    print("Wszystkie pozycje odtworzone.")
    sys.exit(0)

except Exception as e:
    print(f"Błąd odtwarzania: {e}")
    sys.exit(1)
