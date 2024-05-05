# import the necessary packages
import numpy as np
import cv2

# load the image, convert it to grayscale, and blur it
image = cv2.imread("test_image.jpeg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (3, 3), 0)
cv2.imshow("Gray", gray)
cv2.waitKey(0)