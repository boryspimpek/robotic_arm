from flask import Flask, json, jsonify, render_template, request
from config import L1, L2, L3, port_bus
from config import base, elbow, shoulder, wrist
from controller import ArmController
from kinematics import Kinematics
from kinematics_full import FullKinematics
from servos import ServoController
from sc_controll import open_gripper, close_gripper
import subprocess
import signal
import os
import time

app = Flask(__name__)

# Initialize components
kin = Kinematics(L1, L2)
fullkin = FullKinematics(L1, L2, L3)
servo_ctrl = ServoController(port_bus)
arm = ArmController(kinematics=Kinematics(L1, L2), servo_ctrl=ServoController(port_bus), fullkin=FullKinematics(L1, L2, L3))
positions_process = None
pad_process = None

def run_script(script_name):
    """Run a Python script and return its output."""
    try:
        result = subprocess.run(['python3', script_name], capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            output_lines = result.stdout.strip().split('\n')
            return output_lines[-1] if output_lines else 'No response.', 200
        return result.stdout.strip(), 500
    except Exception as e:
        return f"Server error: {e}", 500

def load_positions():
    """Load saved positions from a JSON file."""
    try:
        with open('positions.json', 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return {}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/home_position', methods=['POST'])
def home_position():
    return handle_action(arm.homepos, "Home position")

@app.route('/move_servo', methods=['POST'])
def move_servo():
    data = request.json
    servo_id = int(data.get('id'))
    angle = int(data.get('angle'))
    servo_ctrl.safe_move_to({servo_id: angle})
    return '', 204

@app.route('/get_angles', methods=['GET'])
def get_angles():
    positions = servo_ctrl.get_positions([1, 2, 3, 4])
    positions_int = {sid: int(round(angle)) for sid, angle in positions.items()}
    return jsonify(positions_int)

@app.route('/move_preset/<preset_name>', methods=['POST'])
def move_preset(preset_name):
    def action():
        positions = load_positions()
        if preset_name in positions:
            angles = positions[preset_name]
            if len(angles) == 4:
                return arm.move_to_angle(*angles)
            else:
                raise ValueError("Invalid angle data")
        else:
            raise ValueError("Preset not found")
    
    return handle_action(action, "Preset moved")

@app.route('/save_preset/<preset_name>', methods=['POST'])
def save_preset(preset_name):
    def action():
        angles = servo_ctrl.get_current_angles()  # lub inna Twoja funkcja do pobrania kątów
        if len(angles) != 4:
            raise ValueError("Unexpected number of angles")
        
        positions = load_positions()
        positions[preset_name] = angles

        with open('positions.json', 'w') as f:
            json.dump(positions, f, indent=2)

        return "Zapisano preset"
    
    return handle_action(action, "Zapisano preset")

@app.route('/play_sequence', methods=['POST'])
def play_sequence():
    def action():
        positions = load_positions()
        sorted_keys = sorted(positions.keys(), key=lambda x: int(x))  # sortujemy po numerach
        for key in sorted_keys:
            angles = positions[key]
            if len(angles) != 4:
                continue  # pomiń błędne dane
            arm.move_to_angle(*angles)
            time.sleep(2)  # opóźnienie między pozycjami w sekundach (zmień wg potrzeb)
        return "Sekwencja zakończona"
    
    return handle_action(action, "Sekwencja zakończona")

@app.route('/start_pad', methods=['POST'])
def start_pad():
    global pad_process
    pad_process = start_subprocess(pad_process, 'pad_controll.py')
    return '', 204

@app.route('/stop_pad', methods=['POST'])
def stop_pad():
    global pad_process
    pad_process = stop_subprocess(pad_process)
    return '', 204

@app.route('/save_position', methods=['POST'])
def save_position():
    return run_script('save_position.py')

@app.route('/reset_positions', methods=['POST'])
def reset_positions():
    return run_script('reset_positions.py')

@app.route('/play_positions', methods=['POST'])
def play_positions():
    global positions_process
    positions_process = start_subprocess(positions_process, 'play_positions.py')
    return 'Odtwarzanie rozpoczęte.', 200 if positions_process else 'Sekwencja już trwa.', 409

@app.route('/play_status', methods=['GET'])
def play_status():
    if os.path.exists('play_done.txt'):
        os.remove('play_done.txt')
        return 'done', 200
    return 'pending', 200

@app.route('/stop_positions', methods=['POST'])
def stop_positions():
    global positions_process
    positions_process = stop_subprocess(positions_process)
    return 'Odtwarzanie zatrzymane.', 200 if positions_process is None else 'Brak aktywnej sekwencji.', 200

@app.route('/torque_on', methods=['POST'])
def torque_on():
    return handle_action(servo_ctrl.torque_on_all, "Torque ON")

@app.route('/torque_off', methods=['POST'])
def torque_off():
    return handle_action(servo_ctrl.torque_off_all, "Torque OFF")

@app.route('/open_gripper', methods=['POST'])
def gripper_open():
    open_gripper()
    return "Gripper otwarty", 200

@app.route('/close_gripper', methods=['POST'])
def gripper_close():
    close_gripper()
    return "Gripper zamknięty", 200

@app.route('/start_manual', methods=['POST'])
def start_manual():
    return handle_action(arm.start_manual_mode, "Manual start")

@app.route('/move_to_point', methods=['POST'])
def move_to_point():
    data = request.json
    x = float(data.get('x'))
    y = float(data.get('y'))
    z = float(data.get('z'))
    cost_mode = data.get('cost_mode', 'standard')  

    return handle_action(
        lambda: arm.move_to_point_full(x, y, z, cost_mode=cost_mode),
        f"Ruch do punktu ({cost_mode})"
    )

def handle_action(action, success_message):
    """Handle an action and return a response."""
    try:
        result = action()
        return str(result) if result else success_message, 200
    except Exception as e:
        import traceback
        traceback.print_exc()
        return f"Error: {e}", 500

def start_subprocess(process, script_name):
    """Start a subprocess if not already running."""
    if process is None or process.poll() is not None:
        return subprocess.Popen(['python3', script_name])
    return process

def stop_subprocess(process):
    """Stop a running subprocess."""
    if process and process.poll() is None:
        process.terminate()
        return None
    return process

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
