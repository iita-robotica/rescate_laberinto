import cv2 as cv
import numpy as np
from time import sleep
gridY = 128
gridX = 128
def short_distance(source):
    cv.rectangle(source, (gridY//2 + 30, gridX//2 - 30), (gridY//2-30, gridX//2+30), (0,255,0), -1)
grid= np.zeros((gridY,gridX, 3), np.uint8)

img = cv.imread(r"C:\Users\Maxi\Documents\program_robots\Titan\rescate_laberinto\Alumnos\Maximo_Cansino\assets\images\red_square.png", 0)

while True:
    #short_distance(grid)
    #gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imshow("grid",img)

    if cv.waitKey(1) == ord("q"):
        break
cv.destroyAllWindows()