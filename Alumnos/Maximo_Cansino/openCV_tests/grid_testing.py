import cv2 as cv
import numpy as np

grid= np.zeros((500,500, 3), np.uint8)
grid [250,250] = 0,0,255 #Blue, red, green
cv.imshow("grid", grid)
cv.waitKey(0)
cv.destroyAllWindows()