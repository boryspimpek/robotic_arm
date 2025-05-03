from flask import Flask, json, jsonify, render_template, request
from config import L1, L2, UART_PORT
from controller import ArmController
from kinematics import Kinematics
from servos import ServoController
import subprocess
import signal

pad_process = None

app = Flask(__name__)

kin = Kinematics(L1, L2)
servo_ctrl = ServoController(UART_PORT)
arm = ArmController(kin, servo_ctrl)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/move_servo', methods=['POST'])
def move_servo():
    data = request.json
    servo_id = int(data.get('id'))
    angle = int(data.get('angle'))
    servo_ctrl.move_to({servo_id: angle})  # <-- UWAGA: przekazujemy słownik!
    return '', 204

@app.route('/get_angles', methods=['GET'])
def get_angles():
    positions = servo_ctrl.get_all_servo_positions_deg([1, 2, 3, 4])
    # Zamień float na int, bo suwaki są całkowite
    positions_int = {sid: int(round(angle)) for sid, angle in positions.items()}
    return jsonify(positions_int)

@app.route('/move_preset/<preset_name>', methods=['POST'])
def move_preset(preset_name):
    # Wczytaj pozycje z pliku
    with open('positions.json', 'r') as f:
        positions = json.load(f)

    if preset_name in positions:
        target_xyz = positions[preset_name]
        arm.move_to_point_dps(target_xyz)  # Wywołanie Twojej funkcji
        return '', 204
    else:
        return 'Preset not found', 404

@app.route('/start_pad', methods=['POST'])
def start_pad():
    global pad_process
    if pad_process is None or pad_process.poll() is not None:
        pad_process = subprocess.Popen(['python3', 'pad_controll.py'])
        print("Sterowanie padem uruchomione.")
    else:
        print("Pad już działa.")
    return '', 204

@app.route('/stop_pad', methods=['POST'])
def stop_pad():
    global pad_process
    if pad_process and pad_process.poll() is None:
        pad_process.send_signal(signal.SIGINT)  # lub SIGTERM
        pad_process = None
        print("Pad zatrzymany.")
    else:
        print("Pad nie działa.")
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
        return 'Odtwarzanie rozpoczęte.', 200
    except Exception as e:
        return f'Błąd: {e}', 500
    
@app.route('/torque_on', methods=['POST'])
def torque_on():
    try:
        servo_ctrl.torque_on_all()
        return 'Torque ON', 200
    except Exception as e:
        return f'Błąd: {e}', 500

@app.route('/torque_off', methods=['POST'])
def torque_off():
    try:
        servo_ctrl.torque_off_all()
        return 'Torque OFF', 200
    except Exception as e:
        return f'Błąd: {e}', 500
    
def run_script(script_name):
    import subprocess
    try:
        result = subprocess.run(['python3', script_name], capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            output_lines = result.stdout.strip().split('\n')
            last_line = output_lines[-1] if output_lines else 'Brak odpowiedzi.'
            return last_line, 200 if result.returncode == 0 else 500
        else:
            return result.stdout.strip(), 500
    except Exception as e:
        return f"Błąd serwera: {e}", 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
