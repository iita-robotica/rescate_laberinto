""" import pytesseract 
import cv2 as cv import numpy as np 
import pytesseract

MIN_CONTOUR_AREA = 10
img = cv2.imread("H_victim.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

blured = cv2.blur(gray, (5,5), 0)

img_thresh = cv2.adaptiveThreshold(blured, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

imgContours, Contours, Hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

for contour in Contours: if cv2.contourArea(contour) > MIN_CONTOUR_AREA: [X, Y, W, H] = cv2.boundingRect(contour) cv2.rectangle(img, (X, Y), (X + W, Y + H), (0,0,255), 2) cv2.imshow('contour', img) 
 """
import cv2 as cv
import numpy as np

grid = np.zeros((200,200,3), np.uint8)

for i in range(100):
    grid[i,100] = 255
for i in range(100):
    grid[100,i]= 255
all_points= np.where(grid == 255)
pointsY = sorted(set(all_points[0]))
pointsX = sorted(set(all_points[1]))

minOfY = min(pointsY)
maxOfX = max(pointsX)

index_min_Y = pointsY.index(minOfY)
#index_min_Y.append(all_points.index(minOfY))
index_max_X = pointsX.index(maxOfX)
#index_max_X.append(all_points.index(maxOfX))

print(f"\nTHE MIN POINT OF Y -> {minOfY} AND THE MAX OF X -> {maxOfX}")
print(f"THE INDEXS ARE Y --> {index_min_Y} AND X -> {index_max_X}")
print(f"\nAPPYING A FILTER THE LARGEST COLUMN SHOULD BE --> Y-> {index_min_Y} X-> {pointsX[index_max_X]}")
#print(pointsX)
distance_poitns = index_max_X - index_min_Y
print(f"The distance height of the objetive is -> {distance_poitns}")
#print(f"ALL POINTS--> Y {all_points[0]}")
#print(f"ALL POINTS--> X {all_points[1]}")

#print(f"MAX INDEX --> {maxIndexY}")
cv.imshow("grid", grid)
cv.waitKey(0)
cv.destroyAllWindows()