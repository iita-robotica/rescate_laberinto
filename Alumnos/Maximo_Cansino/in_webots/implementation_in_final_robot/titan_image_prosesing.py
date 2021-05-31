from controller import Robot, Camera
import cv2 as cv # Include OpenCV library
import numpy as np # For python, include numpy as well



def empty(a):
    pass

cv.namedWindow("trackBars")
cv.resizeWindow("trackBars", 640,240)
cv.createTrackbar("hue mini", "trackBars", 206, 255, empty),
cv.createTrackbar("hue max", "trackBars", 179, 255, empty),
cv.createTrackbar("saturation mini", "trackBars", 157, 255, empty),
cv.createTrackbar("saturation max", "trackBars", 255, 255, empty),
cv.createTrackbar("min value", "trackBars", 82 , 255, empty),
cv.createTrackbar("max value", "trackBars", 255, 255, empty),


robot = Robot()
camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
timestep = int(robot.getBasicTimeStep())

camera1.enable(timestep)
camera2.enable(timestep)

def redListener():
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGR2HSV)
    hue_min1= 73
    hue_max1=179
    saturation_min1=157
    saturation_max1=255
    min_value1=127
    max_value1=255
    lower1 = np.array([hue_min1, saturation_min1, min_value1])
    upper1 = np.array([hue_max1, saturation_max1, max_value1])
    mask1 = cv.inRange(hsv_image1, lower1, upper1)
    imgResult1 = cv.bitwise_and(image1, image1, mask=mask1)
    
    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
    hsv_image2 = cv.cvtColor(image2, cv.COLOR_BGR2HSV)
    hue_min2= 73
    hue_max2=179
    saturation_min2=157
    saturation_max2=255
    min_value2=127
    max_value2=255
    lower2 = np.array([hue_min2, saturation_min2, min_value2])
    upper2 = np.array([hue_max2, saturation_max2, max_value2])
    mask2 = cv.inRange(hsv_image2, lower2, upper2)
    imgResult2 = cv.bitwise_and(image2, image2, mask=mask2)

    panelRedListener = cv.hconcat([imgResult1, imgResult2])
    cv.imshow("RedPanel", panelRedListener)

def yellowListener():
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGR2HSV)
    hue_min1= 0
    hue_max1=40
    saturation_min1=157
    saturation_max1=255
    min_value1=82
    max_value1=255
    lower1 = np.array([hue_min1, saturation_min1, min_value1])
    upper1 = np.array([hue_max1, saturation_max1, max_value1])
    mask1 = cv.inRange(hsv_image1, lower1, upper1)
    imgResult1 = cv.bitwise_and(image1, image1, mask=mask1)
    
    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
    hsv_image2 = cv.cvtColor(image2, cv.COLOR_BGR2HSV)
    hue_min2= 0
    hue_max2=40
    saturation_min2= 157
    saturation_max2=255
    min_value2= 82
    max_value2=255
    lower2 = np.array([hue_min2, saturation_min2, min_value2])
    upper2 = np.array([hue_max2, saturation_max2, max_value2])
    mask2 = cv.inRange(hsv_image2, lower2, upper2)
    imgResult2 = cv.bitwise_and(image2, image2, mask=mask2)

    panelYellowListener = cv.hconcat([imgResult1, imgResult2])
    cv.imshow("yellowPanel", panelYellowListener)
    #return [imgResult1, imgResult2]

def whiteListener():
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGR2HSV)
    hue_min1 = cv.getTrackbarPos("hue mini", "trackBars")
    hue_max1= cv.getTrackbarPos("hue max", "trackBars")
    saturation_min1= cv.getTrackbarPos("saturation mini", "trackBars")
    saturation_max1= cv.getTrackbarPos("saturation max", "trackBars")
    min_value1= cv.getTrackbarPos("min value", "trackBars")
    max_value1= cv.getTrackbarPos("max value", "trackBars")
    lower1 = np.array([hue_min1, saturation_min1, min_value1])
    upper1 = np.array([hue_max1, saturation_max1, max_value1])
    mask1 = cv.inRange(hsv_image1, lower1, upper1)
    imgResult1 = cv.bitwise_and(image1, image1, mask=mask1)
      
    image2 = camera2.getImage()
    image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
    hsv_image2 = cv.cvtColor(image2, cv.COLOR_BGR2HSV)
    hue_min2= cv.getTrackbarPos("hue mini", "trackBars")
    hue_max2= cv.getTrackbarPos("hue max", "trackBars")
    saturation_min2= cv.getTrackbarPos("saturation mini", "trackBars")
    saturation_max2=cv.getTrackbarPos("saturation max", "trackBars")
    min_value2= cv.getTrackbarPos("min value", "trackBars")
    max_value2= cv.getTrackbarPos("max value", "trackBars")
    lower2 = np.array([hue_min2, saturation_min2, min_value2])
    upper2 = np.array([hue_max2, saturation_max2, max_value2])
    mask2 = cv.inRange(hsv_image2, lower2, upper2)
    imgResult2 = cv.bitwise_and(image2, image2, mask=mask2)

    panelWhiteListener = cv.hconcat([imgResult1, imgResult2])
    cv.imshow("whitePanel", panelWhiteListener)

    #return [imgResult1, imgResult2]

    

while robot.step(timestep) != -1:
    redListener()
    yellowListener()
    whiteListener()
    cv.waitKey(1)




""" def whiteListener():
    image1 = camera1.getImage()
    image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    # gray = cv.cvtColor(image1, cv.COLOR_BGR2GRAY)
    # #gray = 255-gray
    # #ret, thresh = cv.threshold(gray, 225, 255, cv.THRESH_BINARY_INV)
    # #image, contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    # #frame = cv.drawContours(image1, contours, -1,(0,0,255),3)
    
    # cv.imshow("whiteone", gray)

    
    grayImage = cv.cvtColor(image1, cv.COLOR_BGR2GRAY)
    (thresh, blackAndWhiteImage) = cv.threshold(grayImage, 127, 255, cv.THRESH_BINARY)
    cv.imshow('Black white image', blackAndWhiteImage) """