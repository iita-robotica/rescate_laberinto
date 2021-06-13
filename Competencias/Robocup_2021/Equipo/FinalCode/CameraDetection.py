from numpy.core.numeric import identity
import cv2 as cv
import numpy as np


class Listener:
    def __init__(self, lowerHSV, upperHSV):
        self.lower = np.array(lowerHSV)
        self.upper = np.array(upperHSV)
    
    def getFiltered(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        #imgResult = cv.bitwise_and(img, img, mask=mask)
        return mask

class Classifier:
    def __init__(self):
        self.redListener = Listener(lowerHSV=(73, 157, 127), upperHSV=(179, 255, 255))
        self.yellowListener = Listener(lowerHSV=(0, 157, 82), upperHSV=(40, 255, 255))
        self.whiteListener = Listener(lowerHSV=(0, 0, 200), upperHSV=(0, 255, 255))
        self.blackListener = Listener(lowerHSV=(0, 0, 0), upperHSV=(0, 255, 10))

    def isClose(self, height):
        return height > 45
    
    def isInCenter(self, pos):
        return 15 < pos[1] < 70

    def getCloseVictims(self, victimPoses, victimImages):
        finalVictims = []
        for pos, img in zip(victimPoses, victimImages):
            height = img.shape[0]
            if self.isClose(height) and self.isInCenter(pos):
                finalVictims.append(img)
        return finalVictims



    def getSumedFilters(self, images):
        finalImg = images[0]
        for index, image in enumerate(images):
            finalImg += image
            cv.imshow(str(index), image)
        return finalImg


    def filterVictims(self, poses, images):
        finalPoses = []
        finalImages = []
        for pos, img in zip(poses, images):
            if 25 < pos[0] < 60:
                finalPoses.append(pos)
                finalImages.append(img)

        return finalPoses, finalImages

    def getVictimImagesAndPositions(self, image):
        binaryImages = [self.redListener.getFiltered(image), 
                        self.yellowListener.getFiltered(image), 
                        self.whiteListener.getFiltered(image), 
                        self.blackListener.getFiltered(image)]

        binaryImage = self.getSumedFilters(binaryImages)
        cv.imshow("binaryImage", binaryImage)
        
        # Encuentra los contornos, aunque se puede confundir con el contorno de la letra
        contours, _ = cv.findContours(binaryImage, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # Pra evitar la confusion dibuja rectangulos blancos donde estan los contornos en la imagen y despues vuelve a
        # sacar los contornos para obtener solo los del rectangulo, no los de las letras.
        for c0 in contours:
            x, y, w, h = cv.boundingRect(c0)
            cv.rectangle(binaryImage, (x, y), (x + w, y + h), (225, 255, 255), -1)
        cv.imshow("thresh2", binaryImage)
        contours, _ = cv.findContours(binaryImage, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # saca las medidas y la posicion de los contornos y agrega a la lista de imagenes la parte esa de la imagen original
        # Tambien anade la posicion de cada recuadro en la imagen original
        finalPoses = []
        finalImages = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            finalImages.append(image[y:y + h, x:x + w])
            finalPoses.append((y, x))
        
        return self.filterVictims(finalPoses, finalImages)

    def isPoison(self, blackPoints, whitePoints):
        return blackPoints < 80 and whitePoints > 700 and whitePoints < 4000
    
    def isVictim(self, blackPoints, whitePoints):
        return whitePoints > 5000 and 1500 > blackPoints > 450
    
    def isCorrosive(self, blackPoints, whitePoints):
        return 1000 < whitePoints < 2500 and 1000 < blackPoints < 2500
    
    def isFlammable(self, redPoints, whitePoints):
        return redPoints and whitePoints
    
    def isOrganicPeroxide(self, redPoints, yellowPoints):
        return redPoints and yellowPoints

    def classifyVictim(self, img):
        letter = "N"
        image = cv.resize(img, (100, 100), interpolation=cv.INTER_AREA)
        colorImgs = {
        "red" : self.redListener.getFiltered(image),
        "yellow" : self.yellowListener.getFiltered(image), 
        "white" : self.whiteListener.getFiltered(image),
        "black" : self.blackListener.getFiltered(image)}

        colorPointCounts = {}
        for key, img in colorImgs.items():
            print("Shpae idisjfdj:", img.shape)
            sought = 255
            all_points = np.where(img == 255)
            all_points = all_points[0]
            count = len(all_points)
            
            colorPointCounts[key] = count
        
        print(colorPointCounts)
        if self.isPoison(colorPointCounts["black"], colorPointCounts["white"]):
            print("Poison!")
            letter = "P"
        
        if self.isVictim(colorPointCounts["black"], colorPointCounts["white"]):
            print("Victim!")
            letter = "H"
        
        if self.isCorrosive(colorPointCounts["black"], colorPointCounts["white"]):
            print("Corroive!")
            letter = "C"
        
        if self.isOrganicPeroxide(colorPointCounts["red"], colorPointCounts["yellow"]):
            print("organic peroxide!")
            letter = "O"
        
        if self.isFlammable(colorPointCounts["red"], colorPointCounts["white"]):
            print("Flammable!")
            letter = "F"

        return letter

    

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