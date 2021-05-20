import cv2 as cv
import numpy as np
faces_cascade = cv.CascadeClassifier("openCV/assets/xmlFiles/haarcascade_frontalface_default.xml")
img = cv.imread("openCV/assets/images/people.png")
img_to_draw = img.copy()
imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
faces= faces_cascade.detectMultiScale(imgGray,1.1, 4)

for (x, y, w, h) in faces:
    cv.rectangle(img_to_draw, (x,y), (x+w, y+h), (0,255,0), 2)

cv.imshow("image", img)
cv.imshow("result", img_to_draw)
cv.waitKey(0)