import time
from st3215 import ST3215

class ContinuousServoController:
    def __init__(self, servo, gear_ratio=8.95, resolution=4096):
        self.servo = servo
        self.gear_ratio = gear_ratio
        self.resolution = resolution
        
        # Słownik do przechowywania stanów pozycji dla różnych serw
        self.servo_states = {}
    
    def _initialize_servo_state(self, servo_id):
        """Inicjalizuje stan pozycji dla danego serwa"""
        if servo_id not in self.servo_states:
            current_raw = self.servo.ReadPosition(servo_id)
            self.servo_states[servo_id] = {
                'raw_position': current_raw,
                'last_raw_position': current_raw,
                'extended_position': current_raw
            }
    
    def _reset_position(self, servo_id):
        """Resetuje licznik pozycji dla danego serwa"""
        current_raw = self.servo.ReadPosition(servo_id)
        self.servo_states[servo_id] = {
            'raw_position': current_raw,
            'last_raw_position': current_raw,
            'extended_position': current_raw
        }
    
    def _read_extended_position(self, servo_id):
        """Odczytuje i aktualizuje pozycję w rozszerzonej skali dla danego serwa"""
        if servo_id not in self.servo_states:
            self._initialize_servo_state(servo_id)
            
        state = self.servo_states[servo_id]
        current_raw = self.servo.ReadPosition(servo_id)
        
        # Oblicz różnicę z uwzględnieniem przejścia przez zero
        diff = current_raw - state['last_raw_position']
        
        if diff > self.resolution // 2:    # Przejście z ~0 do ~4095
            diff -= self.resolution
        elif diff < -self.resolution // 2: # Przejście z ~4095 do ~0
            diff += self.resolution
        
        # Aktualizuj pozycję
        state['extended_position'] += diff
        state['last_raw_position'] = current_raw
        state['raw_position'] = current_raw
        
        return state['extended_position']
    
    def _old_to_extended(self, old_units):
        """Konwertuje jednostki starego systemu (0-4095) na rozszerzone"""
        return old_units * self.gear_ratio
    
    def rotate_by(self, servo_id, old_units_delta, speed, tolerance=10):
        """
        Obraca serwo o określoną wartość w starych jednostkach (0-4095)
        
        Args:
            servo_id: ID serwa
            old_units_delta: ilość jednostek do obrócenia (może być ujemna)
            speed: prędkość ruchu
            tolerance: tolerancja zakończenia ruchu
        """
        # Inicjalizuj stan serwa jeśli nie istnieje
        self._initialize_servo_state(servo_id)
        
        # Konwersja na rozszerzone jednostki
        movement = self._old_to_extended(abs(old_units_delta))
        if old_units_delta < 0:
            movement = -movement
        
        start_position = self._read_extended_position(servo_id)
        target_position = start_position + movement
        
        print(f"Serwo {servo_id}: Start: {start_position}, Cel: {target_position}, Ruch: {movement}")
        
        # Sprawdź czy potrzebny jest ruch
        if movement == 0:
            print(f"Serwo {servo_id}: Brak ruchu - pozycja docelowa = pozycja startowa")
            return
        
        # Uruchom ruch
        direction = speed if movement > 0 else -speed
        self.servo.Rotate(servo_id, direction)
        print(f"Serwo {servo_id}: Kierunek: do przodu" if movement > 0 else f"Serwo {servo_id}: Kierunek: do tyłu")
        
        # Pętla kontroli ruchu
        try:
            while True:
                current_pos = self._read_extended_position(servo_id)
                distance = abs(target_position - current_pos)
                
                print(f"Serwo {servo_id}: Postęp: {abs(current_pos - start_position):6.0f}/"
                      f"{abs(movement):6.0f}, Pozostało: {distance:6.0f}", end='\r')
                
                # Warunki zakończenia
                if (distance <= tolerance or 
                    (movement > 0 and current_pos >= target_position) or
                    (movement < 0 and current_pos <= target_position)):
                    break
                    
                time.sleep(0.005)
                
        finally:
            # Zawsze zatrzymaj serwo
            self.servo.StopServo(servo_id)
        
        # Raport końcowy
        final_pos = self._read_extended_position(servo_id)
        actual_movement = final_pos - start_position
        
        print(f"\nSerwo {servo_id}: Zakończono: Pozycja {final_pos}, Ruch {actual_movement}, "
              f"Błąd: {actual_movement - movement}")
    
    def get_position(self, servo_id):
        """Zwraca aktualną pozycję serwa w rozszerzonych jednostkach"""
        return self._read_extended_position(servo_id)
    
    def reset_position(self, servo_id):
        """Resetuje pozycję serwa do zera"""
        self._reset_position(servo_id)
        print(f"Serwo {servo_id}: Pozycja zresetowana")
