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
        self.blackListener = Listener(lowerHSV=(0, 0, 0), upperHSV=(0, 0, 0))

    def getVictims(self, VictimImage):
        return "H"


    def getSumedFilters(self, images):
        
        finalImg = images[0]
        for index, image in enumerate(images):
            finalImg += image
            cv.imshow(str(index), image)
        return finalImg



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
        return finalPoses, finalImages


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