import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Brak podłączonego kontrolera!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Podłączono: {joystick.get_name()}")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Naciśnięto przycisk nr: {event.button}")

        elif event.type == pygame.JOYBUTTONUP:
            print(f"Puszczono przycisk nr: {event.button}")
