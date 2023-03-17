from numpy.lib.utils import source
from controller import Robot, Camera
import cv2 as cv
import numpy as np
from time import sleep




class TitanVision:
    def __init__(self, camera1, camera2):
        self.camera1 = camera1
        self.camera2 = camera2
        self.redHazzardFlag = False;
        self.yellowHazzardFlag = False;
        self.whiteCharacterVictim = False;
        self.counter = 0


    def victimSpotCall(self, image, flagType, hazzardType):
        gridOfPixelsValues = image
        for i in image:
            for a in i:
                if a == 255:
                    if flagType == False:
                        print(f"\n\nI spot a {hazzardType}")
                        self.flagType = True
                    else:
                        pass


    def victimHeight(self, image):
        gridOfPixelsValues = image
        for i in image:
            for a in i:
                self.counter += 1
                if a == 255:
                    print(f"\n\tCOUNTER FOUND SOMETHING\n{self.counter}")
                    print(f"\tINDEX--> {i}||||{a}")
    def redListener(self, camera1, camera2):
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

        panelMask = cv.hconcat([mask1,mask2])
        panelRedListener = cv.hconcat([imgResult1, imgResult2])
        cv.imshow("RedPanel", panelRedListener)
        #cv.imshow("RedPanelMask", panelMask) #panel with white were are red.
        #self.victimSpotCall(panelMask, self.redHazzardFlag ,"red hazzard warning!" )
        self.victimHeight(panelMask)
        

    def yellowListener(self, camera1, camera2):
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

        panelMask = cv.hconcat([mask1, mask2])
        panelYellowListener = cv.hconcat([imgResult1, imgResult2])
        cv.imshow("yellowPanel", panelYellowListener)
        #self.victimSpotCall(panelMask, self.yellowHazzardFlag,"Yellow hazzard warning!" )

    def whiteListener(self, camera1, camera2):
        image1 = camera1.getImage()
        image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
        hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGR2HSV)
        hue_min1 = 0
        hue_max1 = 0
        saturation_min1 = 0
        saturation_max1= 255
        min_value1 = 206
        max_value1 = 255
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

        panelMask = cv.hconcat([mask1, mask2])
        panelWhiteListener = cv.hconcat([imgResult1, imgResult2])
        cv.imshow("whitePanel", panelWhiteListener)
        #self.victimSpotCall(panelMask, self.whiteCharacterVictim ,"white character victim!" )

                
robot = Robot()
camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
timestep = int(robot.getBasicTimeStep())

camera1.enable(timestep)
camera2.enable(timestep)
titanVision = TitanVision(camera1, camera2)
print("start in 5 seconds")
sleep(5)
while robot.step(timestep) != -1:
    titanVision.redListener(titanVision.camera1, titanVision.camera2)
"""     titanVision.yellowListener(titanVision.camera1, titanVision.camera2)
    titanVision.whiteListener(titanVision.camera1, titanVision.camera2) """