from flask import Flask, json, jsonify, render_template, request
from config import L1, L2, port
from controller import ArmController
from kinematics import Kinematics
from servos import ServoController
import subprocess
import signal

app = Flask(__name__)

# Initialize controllers
kin = Kinematics(L1, L2)
servo_ctrl = ServoController(port)
arm = ArmController(kin, servo_ctrl)

# Global variable for pad process
pad_process = None


# Utility functions
def run_script(script_name):
    try:
        result = subprocess.run(['python3', script_name], capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            output_lines = result.stdout.strip().split('\n')
            return output_lines[-1] if output_lines else 'No response.', 200
        return result.stdout.strip(), 500
    except Exception as e:
        return f"Server error: {e}", 500


def load_positions():
    try:
        with open('positions.json', 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


# Routes
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/move_servo', methods=['POST'])
def move_servo():
    data = request.json
    servo_id = int(data.get('id'))
    angle = int(data.get('angle'))
    servo_ctrl.move_to({servo_id: angle})
    return '', 204


@app.route('/get_angles', methods=['GET'])
def get_angles():
    positions = servo_ctrl.get_all_servo_positions_deg([1, 2, 3, 4])
    positions_int = {sid: int(round(angle)) for sid, angle in positions.items()}
    return jsonify(positions_int)


@app.route('/move_preset/<preset_name>', methods=['POST'])
def move_preset(preset_name):
    positions = load_positions()
    if preset_name in positions:
        target_xyz = positions[preset_name]
        arm.move_to_point_dps(target_xyz)
        return '', 204
    return 'Preset not found', 404


@app.route('/start_pad', methods=['POST'])
def start_pad():
    global pad_process
    if pad_process is None or pad_process.poll() is not None:
        pad_process = subprocess.Popen(['python3', 'pad_controll.py'])
        print("Pad control started.")
    else:
        print("Pad is already running.")
    return '', 204


@app.route('/stop_pad', methods=['POST'])
def stop_pad():
    global pad_process
    if pad_process and pad_process.poll() is None:
        pad_process.send_signal(signal.SIGINT)
        pad_process = None
        print("Pad stopped.")
    else:
        print("Pad is not running.")
    return '', 204


@app.route('/save_position', methods=['POST'])
def save_position():
    return run_script('save_position.py')


@app.route('/reset_positions', methods=['POST'])
def reset_positions():
    return run_script('reset_positions.py')


@app.route('/play_positions', methods=['POST'])
def play_positions():
    try:
        subprocess.Popen(['python3', 'play_positions.py'])
        return 'Playback started.', 200
    except Exception as e:
        return f'Error: {e}', 500


@app.route('/torque_on', methods=['POST'])
def torque_on():
    try:
        servo_ctrl.torque_on_all()
        return 'Torque ON', 200
    except Exception as e:
        return f'Error: {e}', 500


@app.route('/torque_off', methods=['POST'])
def torque_off():
    try:
        servo_ctrl.torque_off_all()
        return 'Torque OFF', 200
    except Exception as e:
        return f'Error: {e}', 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
