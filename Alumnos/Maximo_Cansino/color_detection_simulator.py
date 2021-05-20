""" from controller import Robot
import cv2 as cv
import numpy as np

myRobot = Robot()
timeStep = 32
camera = myRobot.getDevice("camera_centre")
camera.enable(timeStep)
#cap = cv.VideoCapture(camera)
image_data = camera.getImage()

while myRobot.step(timeStep) != -1:
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))

     """


from controller import Robot, Camera
import cv2  as cv
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera1 = robot.getDevice("camera1")
camera1.enable(timestep)
camera2 = robot.getDevice("camera2")
camera2.enable(timestep)


def empty(a):
    pass

cv.namedWindow("trackBars")
cv.resizeWindow("trackBars", 640,240)
cv.createTrackbar("hue mini", "trackBars", 0, 179, empty),
cv.createTrackbar("hue max", "trackBars", 179, 179, empty),
cv.createTrackbar("saturation mini", "trackBars", 0, 255, empty),
cv.createTrackbar("saturation max", "trackBars", 255, 255, empty),
cv.createTrackbar("min value", "trackBars", 0 , 255, empty),
cv.createTrackbar("max value", "trackBars", 255, 255, empty),


""" while robot.step(timestep) != -1:
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    image = cv.imrea
    image = cv.cvtColor(image, cv.COLOR_BGRA2BGR)

    cv.imshow("BGR", image)    
    cv.waitKey(1) # Render imshows on scre """


while robot.step(timestep) != -1:
    image = camera1.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
    image = np.array(image ,dtype=np.uint8)

    hsv_image = cv.cvtColor(image, cv.COLOR_BGRA2BGR)
    hue_min=cv.getTrackbarPos("hue mini", "trackBars")
    hue_max=cv.getTrackbarPos("hue max", "trackBars")

    saturation_min=cv.getTrackbarPos("saturation mini", "trackBars")
    saturation_max=cv.getTrackbarPos("saturation max", "trackBars")
    
    min_value=cv.getTrackbarPos("min value", "trackBars")
    max_value=cv.getTrackbarPos("max value", "trackBars")

    print(f"The min hue is -> {hue_min}\tThe max hue is -> {hue_max}\tThe min saturation is -> {saturation_min}\tThe max saturation is -> {saturation_max}\tThe min value is -> {min_value}\tThe max value is -> {max_value}")

    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])
    mask = cv.inRange(hsv_image, lower, upper)
    imgResult = cv.bitwise_and(image, image, mask=mask)
    
    cv.imshow("BGR", image)
    cv.imshow("HSV", hsv_image) 
    cv.imshow("Mask", mask)
    cv.imshow("Final Reslut", imgResult)
    cv.waitKey(1)