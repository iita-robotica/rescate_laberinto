import cv2 as cv
import numpy as np
image=cv.imread("openCV/assets/images/abc_positionated.png")
horizontal_image = np.hstack((image, image))
vertical_image = np.vstack((horizontal_image, horizontal_image))
cv.imshow("windows",vertical_image)
cv.waitKey(0)

""" That was easy """