import json
import sys
from sc_controll import sc_servo_position
from servos import ServoController
from config import base, elbow, gripper, shoulder, wrist, port_bus

try:
    servo = ServoController(port_bus)
    current_angles = servo.get_all_servo_positions_deg([base, shoulder, elbow, wrist])
    current_angles = {str(k): int(round(v)) for k, v in current_angles.items()}
    
    # Pobierz pozycję serwa SC (chwytak)
    sc_angle = sc_servo_position(gripper)
    current_angles[str(gripper)] = int(round(sc_angle))    
    
    try:
        with open('recorded_positions.json', 'r') as f:
            positions = json.load(f)
    except FileNotFoundError:
        positions = {}

    new_key = f"pos_{len(positions) + 1:02}"
    positions[new_key] = current_angles

    with open('recorded_positions.json', 'w') as f:
        json.dump(positions, f, indent=4)

    print("Pozycja zapisana!")
    sys.exit(0)

except Exception as e:
    print(f"Błąd zapisu: {e}")
    sys.exit(1)
