import cv2

# Create a named window
window_name="TESTNAME"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
# Move window to the 2nd monitor
cv2.moveWindow(name, 900,-900)
# Set the window properties to full screen
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

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
