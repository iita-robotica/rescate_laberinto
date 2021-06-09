# test__3
# TODO: IMPLEMENT THIS LOGIC IN THE FINAL PROGRAM

from numpy.core.numeric import identity
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
        for i in image:
            for a in i:
                if a == 255:
                    print(a)

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
        #cv.imshow("RedPanel", panelRedListener)
        cv.imshow("RedPanelMask", panelMask) #panel with white were are red.
        return panelMask

        #self.victimSpotCall(panelMask, self.redHazzardFlag ,"red hazzard warning!" )
        
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
        cv.imshow("yellowPanel", panelMask)
        return panelMask

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
        hue_min2 = 0
        hue_max2 = 0
        saturation_min2 = 0
        saturation_max2= 255
        min_value2 = 206
        max_value2 = 255
        lower2 = np.array([hue_min2, saturation_min2, min_value2])
        upper2 = np.array([hue_max2, saturation_max2, max_value2])
        mask2 = cv.inRange(hsv_image2, lower2, upper2)
        imgResult2 = cv.bitwise_and(image2, image2, mask=mask2)

        panelMask = cv.hconcat([mask1, mask2])
        panelWhiteListener = cv.hconcat([imgResult1, imgResult2])
        cv.imshow("White panel", panelMask)
        return panelMask
        #self.victimSpotCall(panelMask, self.whiteCharacterVictim ,"white character victim!" )

    def blackListener(self, camera1, camera2):
        image1 = camera1.getImage()
        image1 = np.frombuffer(image1, np.uint8).reshape((camera1.getHeight(), camera1.getWidth(), 4))
        hsv_image1 = cv.cvtColor(image1, cv.COLOR_BGR2HSV)
        hue_min1 = 0
        hue_max1 = 0
        saturation_min1 = 0
        saturation_max1= 0
        min_value1 = 0
        max_value1 = 0
        lower1 = np.array([hue_min1, saturation_min1, min_value1])
        upper1 = np.array([hue_max1, saturation_max1, max_value1])
        mask1 = cv.inRange(hsv_image1, lower1, upper1)
        imgResult1 = cv.bitwise_and(image1, image1, mask=mask1)
        
        image2 = camera2.getImage()
        image2 = np.frombuffer(image2, np.uint8).reshape((camera2.getHeight(), camera2.getWidth(), 4))
        hsv_image2 = cv.cvtColor(image2, cv.COLOR_BGR2HSV)
        hue_min2 = 0
        hue_max2 = 0
        saturation_min2 = 0
        saturation_max2= 0
        min_value2 = 0
        max_value2 = 0
        lower2 = np.array([hue_min2, saturation_min2, min_value2])
        upper2 = np.array([hue_max2, saturation_max2, max_value2])
        mask2 = cv.inRange(hsv_image2, lower2, upper2)
        imgResult2 = cv.bitwise_and(image2, image2, mask=mask2)

        panelMask = cv.hconcat([mask1, mask2])
        panelWhiteListener = cv.hconcat([imgResult1, imgResult2])
        cv.imshow("Black panel", panelMask)
        return panelMask
        #self.victimSpotCall(panelMask, self.whiteCharacterVictim ,"white character victim!" )

                
robot = Robot()
camera1 = robot.getDevice("camera1")
camera2 = robot.getDevice("camera2")
timestep = int(robot.getBasicTimeStep())

camera1.enable(timestep)
camera2.enable(timestep)
titanVision = TitanVision(camera1, camera2)

def algorithm_start222(layers_panels):
    list_of_data = []
    for keys, value in layers_panels.items():
        all_points = np.where(value == 255)
        all_points = all_points[0]
        if len(all_points) != 0:
            list_of_data.append(keys)
    return list_of_data
    list_of_data = []

identity_red_yellow_hazzard = ['Red', 'Yellow']
identity_red_hazzard = ['Red', 'White']
identity_white_hazzard = ['White']

def blackAndWhiteCases(panels_of_values):
    blacks = 0
    whites = 0
    for keys, value in panels_of_values.items():
        all_points = np.where(value == 255)
        all_points = all_points[0]
        if len(all_points) == 0:
            print("\n_")
        if keys == "Black":
            blacks = len(all_points)
        if keys == "White":
            whites = len(all_points)
    
    if blacks <20 and whites > 700 and whites <1000:
    #print(f"balcks --> {blacks}")
        print("all white hazzard")
    elif whites < 200 and whites > 100 and blacks > 350 and blacks < 550:
        print("white and black hazzard detected")
    
    elif whites > 2000 and blacks< 510:
        print("VICTIM DETECTED --> APPLY TEAM LOGIC FOR VICTIMS")
    #print(f"We have {whites} whites and {blacks} blacks")




while robot.step(timestep) != -1:
    list_layers_names = {
        "Red": titanVision.redListener(titanVision.camera1, titanVision.camera2),
        "Yellow":titanVision.yellowListener(titanVision.camera1, titanVision.camera2),
        "White": titanVision.whiteListener(titanVision.camera1, titanVision.camera2),
        "Black": titanVision.blackListener(titanVision.camera1, titanVision.camera2)}
    list_layers_names2 = {
        "White": titanVision.whiteListener(titanVision.camera1, titanVision.camera2),
        "Black": titanVision.blackListener(titanVision.camera1, titanVision.camera2)}
    hazzard = algorithm_start222(list_layers_names)
    if  hazzard == identity_red_yellow_hazzard:
        print('I found the yellow and red hazzard')
    elif hazzard == identity_red_hazzard:
        print('I found the RED hazzard')
    else:
        print("\n")
        blackAndWhiteCases(list_layers_names2)
    
    


    cv.waitKey(1)
