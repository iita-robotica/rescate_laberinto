import pytesseract 
import cv2 as cv import numpy as np 
import pytesseract

MIN_CONTOUR_AREA = 10
img = cv2.imread("H_victim.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

blured = cv2.blur(gray, (5,5), 0)

img_thresh = cv2.adaptiveThreshold(blured, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

imgContours, Contours, Hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

for contour in Contours: if cv2.contourArea(contour) > MIN_CONTOUR_AREA: [X, Y, W, H] = cv2.boundingRect(contour) cv2.rectangle(img, (X, Y), (X + W, Y + H), (0,0,255), 2) cv2.imshow('contour', img) 
