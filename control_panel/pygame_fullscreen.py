import pygame
import os

# Define the path to your image file
image_path = 'test.jpg'

# Initialize Pygame
pygame.init()

# Set the display mode to fullscreen on the first screen
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

# Load the image
image = pygame.image.load(image_path)

# Scale the image to fit the screen resolution
screen_width, screen_height = screen.get_size()
image = pygame.transform.scale(image, (screen_width, screen_height))

# Clear the screen and draw the image
screen.fill((0, 0, 0))  # Fill the screen with black color
screen.blit(image, (0, 0))  # Draw the image at position (0, 0)

# Update the display
pygame.display.flip()

# Wait for the user to close the window
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

# Quit Pygame
pygame.quit()
