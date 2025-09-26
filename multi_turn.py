import time
import math
from utilis import servo_positions, move_to_point, trajectory, servo_to_rad
from ik import forward_kinematics
from st3215 import ST3215


servo = ST3215('/dev/ttyACM0')

class ServoPositionTracker:
    def __init__(self, servo, servo_id, servo_resolution=4096):
        self.servo = servo
        self.servo_id = servo_id
        self.servo_resolution = servo_resolution
        
        # Inicjalizacja
        self.raw_position = servo.ReadPosition(servo_id)
        self.extended_position = self.raw_position  # Pozycja w rozszerzonej skali
        self.last_raw_position = self.raw_position
        
    def update_position(self):
        """Aktualizuje pozycję w rozszerzonej skali"""
        current_raw = self.servo.ReadPosition(self.servo_id)
        
        # Oblicz różnicę z uwzględnieniem przejścia przez zero
        diff = current_raw - self.last_raw_position
        
        # Korekta dla przejścia przez zero
        if diff > self.servo_resolution // 2:  # Przejście z ~0 do ~4095 (ruch w tył)
            diff -= self.servo_resolution
        elif diff < -self.servo_resolution // 2:  # Przejście z ~4095 do ~0 (ruch w przód)
            diff += self.servo_resolution
        
        # Aktualizuj pozycję w rozszerzonej skali
        self.extended_position += diff
        self.last_raw_position = current_raw
        self.raw_position = current_raw
        
        return self.extended_position
    
    def get_extended_position(self):
        """Zwraca aktualną pozycję w rozszerzonej skali"""
        return self.update_position()
    
    def reset_position(self, new_extended_pos=None):
        """Resetuje licznik pozycji rozszerzonej"""
        self.raw_position = self.servo.ReadPosition(self.servo_id)
        self.last_raw_position = self.raw_position
        if new_extended_pos is not None:
            self.extended_position = new_extended_pos
        else:
            self.extended_position = self.raw_position

def convert_old_to_movement(old_position, gear_ratio=9):
    """
    Przelicza starą pozycję (0-4095) na wielkość ruchu w rozszerzonej skali
    """
    movement_units = old_position * gear_ratio
    return movement_units

def rotate_by_old_position(servo, servo_id, old_position_delta, speed=1000, tolerance=10):
    """
    Obraca serwo O określoną wartość (ruch względny)
    old_position_delta: ile jednostek ruchu w starym systemie (0-4095)
    Może być ujemne dla ruchu w tył
    """
    movement_extended = convert_old_to_movement(abs(old_position_delta))
    if old_position_delta < 0:
        movement_extended = -movement_extended
    
    # Inicjalizuj tracker pozycji
    tracker = ServoPositionTracker(servo, servo_id)
    start_extended = tracker.get_extended_position()
    target_extended = start_extended + movement_extended
    
    print(f"Pozycja startowa (rozszerzona): {start_extended}")
    print(f"Ruch o: {movement_extended} jednostek")
    print(f"Pozycja docelowa (rozszerzona): {target_extended}")
    
    # Określ kierunek i rozpocznij ruch
    if movement_extended > 0:
        servo.Rotate(servo_id, speed)  # W przód
        print("Kierunek: do przodu")
    elif movement_extended < 0:
        servo.Rotate(servo_id, -speed)  # W tył
        print("Kierunek: do tyłu")
    else:
        print("Brak ruchu - pozycja docelowa = pozycja startowa")
        return
    
    # Płynny ruch do celu
    while True:
        current_extended = tracker.get_extended_position()
        distance_to_target = abs(target_extended - current_extended)
        progress = abs(current_extended - start_extended)
        
        print(f"Postęp: {progress:6.0f}/{abs(movement_extended):6.0f}, Pozostało: {distance_to_target:6.0f}", end='\r')
        
        # Sprawdź czy dotarliśmy do celu
        if distance_to_target <= tolerance:
            break
            
        # Sprawdź czy minęliśmy cel (zabezpieczenie)
        if movement_extended > 0:  # Ruch w przód
            if current_extended >= target_extended:
                break
        else:  # Ruch w tył
            if current_extended <= target_extended:
                break
        
        time.sleep(0.005)
    
    # Zatrzymaj serwo
    servo.StopServo(servo_id)
    
    final_extended = tracker.get_extended_position()
    final_raw = tracker.raw_position
    actual_movement = final_extended - start_extended
    
    print(f"\nZADANIE ZAKOŃCZONE")
    print(f"Pozycja końcowa (rozszerzona): {final_extended}")
    print(f"Pozycja końcowa (surowa): {final_raw}")
    print(f"Planowany ruch: {movement_extended}")
    print(f"Rzeczywisty ruch: {actual_movement}")
    print(f"Błąd ruchu: {actual_movement - movement_extended}")


# Przykład użycia
if __name__ == "__main__":
    
    rotate_by_old_position(servo, 1, -2048)  