import time
from st3215 import ST3215

class ContinuousServoController:
    def __init__(self, servo, servo_id, gear_ratio=9, resolution=4096):
        self.servo = servo
        self.servo_id = servo_id
        self.gear_ratio = gear_ratio
        self.resolution = resolution
        
        # Inicjalizacja pozycji
        self._reset_position()
    
    def _reset_position(self):
        """Resetuje licznik pozycji"""
        current_raw = self.servo.ReadPosition(self.servo_id)
        self.raw_position = current_raw
        self.last_raw_position = current_raw
        self.extended_position = current_raw
    
    def _read_extended_position(self):
        """Odczytuje i aktualizuje pozycję w rozszerzonej skali"""
        current_raw = self.servo.ReadPosition(self.servo_id)
        
        # Oblicz różnicę z uwzględnieniem przejścia przez zero
        diff = current_raw - self.last_raw_position
        
        if diff > self.resolution // 2:    # Przejście z ~0 do ~4095
            diff -= self.resolution
        elif diff < -self.resolution // 2: # Przejście z ~4095 do ~0
            diff += self.resolution
        
        # Aktualizuj pozycję
        self.extended_position += diff
        self.last_raw_position = current_raw
        self.raw_position = current_raw
        
        return self.extended_position
    
    def _old_to_extended(self, old_units):
        """Konwertuje jednostki starego systemu (0-4095) na rozszerzone"""
        return old_units * self.gear_ratio
    
    def rotate_by(self, old_units_delta, speed=1000, tolerance=10):
        """
        Obraca serwo o określoną wartość w starych jednostkach (0-4095)
        
        Args:
            old_units_delta: ilość jednostek do obrócenia (może być ujemna)
            speed: prędkość ruchu
            tolerance: tolerancja zakończenia ruchu
        """
        # Konwersja na rozszerzone jednostki
        movement = self._old_to_extended(abs(old_units_delta))
        if old_units_delta < 0:
            movement = -movement
        
        start_position = self._read_extended_position()
        target_position = start_position + movement
        
        print(f"Start: {start_position}, Cel: {target_position}, Ruch: {movement}")
        
        # Sprawdź czy potrzebny jest ruch
        if movement == 0:
            print("Brak ruchu - pozycja docelowa = pozycja startowa")
            return
        
        # Uruchom ruch
        direction = speed if movement > 0 else -speed
        self.servo.Rotate(self.servo_id, direction)
        print("Kierunek: do przodu" if movement > 0 else "Kierunek: do tyłu")
        
        # Pętla kontroli ruchu
        try:
            while True:
                current_pos = self._read_extended_position()
                distance = abs(target_position - current_pos)
                
                print(f"Postęp: {abs(current_pos - start_position):6.0f}/"
                      f"{abs(movement):6.0f}, Pozostało: {distance:6.0f}", end='\r')
                
                # Warunki zakończenia
                if (distance <= tolerance or 
                    (movement > 0 and current_pos >= target_position) or
                    (movement < 0 and current_pos <= target_position)):
                    break
                    
                time.sleep(0.005)
                
        finally:
            # Zawsze zatrzymaj serwo
            self.servo.StopServo(self.servo_id)
        
        # Raport końcowy
        final_pos = self._read_extended_position()
        actual_movement = final_pos - start_position
        
        print(f"\nZakończono: Pozycja {final_pos}, Ruch {actual_movement}, "
              f"Błąd: {actual_movement - movement}")

    


# Przykład użycia
if __name__ == "__main__":
    servo = ST3215('/dev/ttyACM0')
    controller = ContinuousServoController(servo, servo_id=1)
    
    controller.rotate_by(-2048)
