import pygame

# Inicjalizacja Pygame i joysticków
pygame.init()
pygame.joystick.init()

# Sprawdzenie, czy jest podłączony joystick
if pygame.joystick.get_count() == 0:
    print("Brak podłączonego kontrolera!")
    exit()

# Pobranie pierwszego joysticka
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Podłączono: {joystick.get_name()}")

# Główna pętla
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYAXISMOTION:
            # Odczyt osi lewego drążka
            left_x = joystick.get_axis(0)
            left_y = joystick.get_axis(1)
            # Odczyt osi prawego drążka
            right_x = joystick.get_axis(3)
            right_y = joystick.get_axis(4)  

            print(f"Left stick:  X={left_x:.2f} Y={left_y:.2f} | Right stick: X={right_x:.2f} Y={right_y:.2f}")
