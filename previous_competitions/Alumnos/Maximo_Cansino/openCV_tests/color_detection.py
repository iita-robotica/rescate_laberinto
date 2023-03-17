import cv2 as cv
import numpy as np

def empty(a):
    pass
image_path='c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/warning_webots_icons.png'
#image_path="openCV/assets/images/picture.jpg"

cv.namedWindow("trackBars")
cv.resizeWindow("trackBars", 640,240)
cv.createTrackbar("hue mini", "trackBars", 206, 255, empty),
cv.createTrackbar("hue max", "trackBars", 179, 255, empty),
cv.createTrackbar("saturation mini", "trackBars", 157, 255, empty),
cv.createTrackbar("saturation max", "trackBars", 255, 255, empty),
cv.createTrackbar("min value", "trackBars", 82 , 255, empty),
cv.createTrackbar("max value", "trackBars", 255, 255, empty),

while True:
    image=cv.imread(image_path)
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hue_min=cv.getTrackbarPos("hue mini", "trackBars")
    hue_max=cv.getTrackbarPos("hue max", "trackBars")
    saturation_min=cv.getTrackbarPos("saturation mini", "trackBars")
    saturation_max=cv.getTrackbarPos("saturation max", "trackBars")
    min_value=cv.getTrackbarPos("min value", "trackBars")
    max_value=cv.getTrackbarPos("max value", "trackBars")
    #print(f"The min hue is -> {hue_min}\tThe max hue is -> {hue_max}\tThe min saturation is -> {saturation_min}\tThe max saturation is -> {saturation_max}\tThe min value is -> {min_value}\tThe max value is -> {max_value}")
    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])
    mask = cv.inRange(hsv_image, lower, upper)
    imgResult = cv.bitwise_and(image, image, mask=mask)
    cv.imshow("BGR", image)
    cv.imshow("HSV", hsv_image) 
    cv.imshow("Mask", mask)
    cv.imshow("Final Reslut", imgResult)
    cv.waitKey(1)