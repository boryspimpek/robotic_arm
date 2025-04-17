import pygame
import time

# Inicjalizacja pygame i joysticka
pygame.init()
pygame.joystick.init()

# Sprawdź, czy joystick jest dostępny
if pygame.joystick.get_count() == 0:
    print("Nie wykryto żadnego kontrolera!")
    exit()

# Wybierz pierwszy wykryty kontroler
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Połączono z: {joystick.get_name()}")

# Pętla główna
try:
    while True:
        pygame.event.pump()  # odświeża zdarzenia

        # Osie analogów (0 = lewo/prawo lewej gałki, 1 = góra/dół lewej gałki itd.)
        for i in range(joystick.get_numaxes()):
            axis_val = joystick.get_axis(i)
            print(f"Oś {i}: {axis_val:.2f}", end="  ")
        print()

        # Przycisk (np. X, Kółko, Kwadrat, itd.)
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                print(f"Przycisk {i} wciśnięty")

        # Krzyżak (D-pad)
        hat = joystick.get_hat(0)
        if hat != (0, 0):
            print(f"D-Pad: {hat}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nZamykam program.")
finally:
    pygame.quit()
