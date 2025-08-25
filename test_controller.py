import pygame
import time

# Inicjalizacja Pygame
pygame.init()
pygame.joystick.init()

# Sprawdź liczbę podłączonych kontrolerów
num_joysticks = pygame.joystick.get_count()
print(f"Liczba podłączonych kontrolerów: {num_joysticks}")

if num_joysticks == 0:
    print("Nie wykryto żadnego kontrolera!")
    exit()

# Inicjalizacja pierwszego kontrolera
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Kontroler: {joystick.get_name()}")
print(f"Liczba osi: {joystick.get_numaxes()}")
print(f"Liczba przycisków: {joystick.get_numbuttons()}")
print(f"Liczba hatów (krzyżaków): {joystick.get_numhats()}")
print("\nNaciśnij przyciski i porusz drążkami...")
print("Naciśnij CTRL+C aby zakończyć\n")

# Słownik z nazwami przycisków PS4 (mogą się różnić w zależności od systemu)
button_names = {
    0: "X",
    1: "Kółko",
    2: "Kwadrat",
    3: "Trójkąt",
    4: "L1",
    5: "R1",
    6: "L2",
    7: "R2",
    8: "Share",
    9: "Options",
    10: "PS",
    11: "L3 (wciśnięcie lewego drążka)",
    12: "R3 (wciśnięcie prawego drążka)",
    13: "Touchpad"
}

try:
    while True:
        pygame.event.pump()  # Przetwarzaj zdarzenia
        
        # Wyświetl wartości wszystkich osi
        axis_values = []
        for i in range(joystick.get_numaxes()):
            value = joystick.get_axis(i)
            axis_values.append(f"{i}:{value:6.3f}")
        
        # Wyświetl stan wszystkich przycisków
        button_states = []
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                button_name = button_names.get(i, f"Przycisk {i}")
                button_states.append(button_name)
        
        # Wyświetl stan hatów (krzyżaków)
        hat_values = []
        for i in range(joystick.get_numhats()):
            hat_value = joystick.get_hat(i)
            hat_values.append(f"Hat {i}: {hat_value}")
        
        # Wyczyść ekran (działa w większości terminali)
        print("\033[H\033[J", end="")
        
        # Wyświetl informacje
        print("=== STAN KONTROLERA PS4 ===")
        print("Osi: " + " ".join(axis_values))
        
        if button_states:
            print("Wciśnięte przyciski: " + ", ".join(button_states))
        else:
            print("Wciśnięte przyciski: brak")
            
        if hat_values:
            print("Haty: " + " | ".join(hat_values))
        else:
            print("Haty: brak")
        
        print("\nNaciśnij CTRL+C aby zakończyć")
        
        time.sleep(0.1)  # Małe opóźnienie, aby nie przeciążyć konsoli

except KeyboardInterrupt:
    print("\nZakończono testowanie kontrolera")