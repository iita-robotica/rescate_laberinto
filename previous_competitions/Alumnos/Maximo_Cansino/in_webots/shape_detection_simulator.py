from controller import Robot, Camera
import cv2  as cv# Include OpenCV library
import numpy as np # For python, include numpy as well
robot = Robot()
camera = robot.getDevice("camera1")
timestep = int(robot.getBasicTimeStep())
camera.enable(timestep)

def empty(a):
    pass

def controllPanel():
    cv.namedWindow("parameters")
    cv.resizeWindow("parameters", 640, 240)
    cv.createTrackbar("Threshold_1", "parameters", 500, 500, empty)
    cv.createTrackbar("Threshold_", "parameters", 500, 500, empty)
    cv.createTrackbar("area", "parameters", 10000, 10000, empty)
    print("area", "parameters")


def showAllImages(img1,img2,img3):
    cv.imshow("img1",img1)
    cv.imshow("img2",img2)
    cv.imshow("img3",img3)


def findContoursMethod(source, sourceToProcess, areaFinder):
    shapeType=""
    contours, hierarchy = cv.findContours(source, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    #cv.drawContours(sourceToProcess, contours, -1, (0,255,0), 3)
    for ctn in contours:
        area = cv.contourArea(ctn)
        if area > areaFinder:
            print(area)
            cv.drawContours(sourceToProcess, ctn, -1, (0,255,0), 3)
            perimetter = cv.arcLength(ctn, True)
            approx = cv.approxPolyDP(ctn, 0.02*perimetter, True)     
            pointsCounter = len(approx)
            x, y, w, h = cv.boundingRect(approx)
            cv.rectangle(sourceToProcess, (x, y), (x + w, y + h), (255,0,0) , 3)
            if pointsCounter == 4:
                if area > 40000:
                    shapeType = "Square"
                else:
                    shapeType = "Diamond"
            elif pointsCounter == 3:
                shapeType = "Triangle"
            elif pointsCounter > 4:
                if area < 100000:
                    shapeType = "circle"
                else:
                    shapeType = "noiseq"
            cv.putText(sourceToProcess, str(shapeType), (x + w + 20, y + 20), cv.FONT_HERSHEY_COMPLEX, .7, (255, 0, 0), 2) #TODO: PUT THIS INSIDE THE LOGIC SENTENCES   
            cv.putText(sourceToProcess, "Area " + str(int(area)), (x + w + 20, y ), cv.FONT_HERSHEY_COMPLEX, .6, (238,130,238), 2)

controllPanel()
while robot.step(timestep) != -1:
    frame = camera.getImage()
    frame = np.frombuffer(frame, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    frame = np.array(frame ,dtype=np.uint8)
    imageCopy = frame.copy()
    imgBlur=  cv.GaussianBlur(frame, (7,7), 1)
    imgGray = cv.cvtColor(imgBlur, cv.COLOR_BGR2GRAY)    

    threshold1 = cv.getTrackbarPos("Threshold_1", "parameters")
    threshold2 = cv.getTrackbarPos("Threshold_2", "parameters")
    areaValues = cv.getTrackbarPos("area", "parameters")

    canny = cv.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5,5))
    imgDilated = cv.dilate(canny, kernel, iterations= 1)

    findContoursMethod(imgDilated, imageCopy, areaValues)
    imageCopy = cv.resize(imageCopy, (500,500))
    scale_percent = 50
    #calculate the 50 percent of original dimensions
    width = int(imageCopy.shape[1] * scale_percent / 100)
    height = int(imageCopy.shape[0] * scale_percent / 100)
    # dsize
    dsize = (width, height)
    imageCopy = cv.resize(imageCopy, dsize)
    cv.imshow("img3",imageCopy)
    #cv.cvGetWindowHandle()
    cv.waitKey(1)
