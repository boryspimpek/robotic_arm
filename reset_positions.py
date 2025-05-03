import json

with open('recorded_positions.json', 'w') as f:
    json.dump({}, f)

print("Wszystkie pozycje usunięte.")
