import cv2 as cv
import numpy as np

def getContours(img, cannyThreshold=[100,100], showCanny = False):
    imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    imgBlur = cv.GaussianBlur(imgGray, (5,5), 1) #<- you can change the last value if you are not getting good results
    imgCanny = cv.Canny(imgBlur, cannyThreshold[0], cannyThreshold[1])
    if showCanny: cv.imshow("canny", imgCanny)

def reorderPoint(points):
    print(points.shape)


def warpImage(img, points, width, hight):
    reorderPoint(points)