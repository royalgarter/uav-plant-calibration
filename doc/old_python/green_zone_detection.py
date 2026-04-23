import cv2
import numpy as np

# Load image
frame = cv2.imread('image.jpg')
# Convert BGR to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Define range of green color in HSV
lower_green = np.array([40, 50, 50])
upper_green = np.array([80, 255, 255])

# Threshold the HSV image to get only green colors
mask = cv2.inRange(hsv, lower_green, upper_green)

# Bitwise-AND mask and original image
green_zone = cv2.bitwise_and(frame, frame, mask=mask)

cv2.imshow('Green Zone', green_zone)
cv2.waitKey(0)
cv2.destroyAllWindows()
