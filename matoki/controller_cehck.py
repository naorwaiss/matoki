import pygame

# Initialize Pygame and the joystick module
pygame.init()
pygame.joystick.init()

# Threshold for triggers to be considered pressed
TRIGGER_THRESHOLD = 0.5

# Check for joysticks and initialize the first one found
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Initialized Joystick: {joystick.get_name()}")
else:
    print("No joystick found.")
    quit()

# Variables to keep track of trigger states
lt_pressed = False
rt_pressed = False

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed")
        elif event.type == pygame.JOYAXISMOTION:
            # Check for LT and RT triggers (commonly axes 2 and 5)
            print(str(event.axis) + " pressed, at value: " + str(event.value))

pygame.quit()