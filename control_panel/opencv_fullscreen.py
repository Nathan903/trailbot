import cv2

# Create a named window
cv2.namedWindow('Full Screen', cv2.WINDOW_NORMAL)
# Set the window properties to full screen
cv2.setWindowProperty('Full Screen', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

def changeEmoji(emoji_number):
    # Read the image
    image_path = f'{emoji_number}.jpg'
    image = cv2.imread(image_path)
    # Display the image
    cv2.imshow('Full Screen', image)
    cv2.waitKey(1)

import time

while True:
	changeEmoji(2)  # Display '1.jpg'
	time.sleep(0.5)
	changeEmoji(1)  # Display '2.jpg'
	time.sleep(0.5)

# Close the window
cv2.destroyAllWindows()
