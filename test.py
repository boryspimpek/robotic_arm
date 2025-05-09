import json

def load_positions():
    with open('positions.json', 'r') as json_file:
        positions = json.load(json_file)
    
    # Konwersja wartości w JSON do liczb całkowitych, jeśli są zapisane jako stringi
    for preset_name, angles in positions.items():
        for key, value in angles.items():
            if isinstance(value, str):
                positions[preset_name][key] = int(value)  # Konwertuj string na int
                print(f"Converted {preset_name} {key} from string to int: {value}")
    return positions
