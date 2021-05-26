 """ IN PROGRES....  """

import cv2 as cv
import numpy as np
from numpy.lib.arraypad import pad

webcam = False
image_path= r'C:\Users\Maxi\Documents\program_robots\Titan\rescate_laberinto\Alumnos\Maximo_Cansino\assets\images\green_rectangle.png'
scalator = 3
a = 150
widthPerspective = a *scalator
heightPerspective = a *scalator
def getContours(img, cannyThreshold=[100,100], showCanny = False, minArea = 1000, filter= 0, draw = False):
    imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    imgBlur = cv.GaussianBlur(imgGray, (5,5), 1) #<- you can change the last value if you are not getting good results
    imgCanny = cv.Canny(imgBlur, cannyThreshold[0], cannyThreshold[1])
    kernel = np.ones((5,5))
    imgDial = cv.dilate(imgCanny, kernel, iterations=3)
    imgThreshold = cv.erode(imgDial, kernel, iterations=2)
    if showCanny: cv.imshow("canny", imgThreshold)
    contours, hiearchy = cv.findContours(imgThreshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    final_contours = []
    for i in contours:
        area= cv.contourArea(i)
        if area > minArea:
            perimetter = cv.arcLength(i, True)
            approx = cv.approxPolyDP(i, 0.02*perimetter, True)
            bounding_box = cv.boundingRect(approx)
            if filter > 0:
                if len(approx) == filter:
                    final_contours.append([len(approx), area, approx, bounding_box, i])
            else: 
                final_contours.append([len(approx), area, approx, bounding_box, i])
    final_contours = sorted(final_contours, key = lambda x:x[1] , reverse= True)
    if draw:
        for con in final_contours:
            cv.drawContours(img, con[4], -1, (0,0,255), 3)
    return img, final_contours


def reorderPoint(points):
    print(points.shape)
    myNewPoints = np.zeros_like(points)
    myPoints = points.reshape((4,2))
    add = myPoints.sum(1)
    myNewPoints[0] = myPoints[np.argmin(add)]
    myNewPoints[3] = myPoints[np.argmax(add)]
    diff = np.diff(myPoints, axis=1)
    myNewPoints[1]= myPoints[np.argmin(diff)]
    myNewPoints[2]= myPoints[np.argmax(diff)]
    return myNewPoints


def warpImage(img, points, width, hight, pad = 20):
    """ print(points)
    print(reorderPoint(points)) """
    points = reorderPoint(points)
    points_1 = np.float32(points)
    points_2 = np.float32([[0,0], [width, 0], [0, hight], [width, hight]])
    matrix = cv.getPerspectiveTransform(points_1, points_2)
    imgWarp = cv.warpPerspective(img, matrix, (width, hight))
    imgWarp = imgWarp[pad:imgWarp.shape[0]-pad, pad:imgWarp.shape[1]-pad]
    return imgWarp

img = cv.imread(image_path)
while True:
    img, final_contours = getContours(
    img,
    #showCanny=True,
    #draw=True,
    minArea=1000,
    filter=4,
    )

    if len(final_contours) != 0:
        biggest = final_contours[0][2]
        print(biggest)
        imgWarp = warpImage(img, biggest, widthPerspective,heightPerspective) #<-- harcoded
        img2, final_contours2 = getContours(
        imgWarp,
        #showCanny=True,
        cannyThreshold=[50, 50],
        draw=True,
        minArea=2000,
        filter=4,)
    cv.imshow("final_result", img2)
    cv.imshow("Warp image", imgWarp)
    cv.waitKey(0)
    break
cv.destroyAllWindows()