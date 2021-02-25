from controller import Robot
import cv2 as cv
import numpy as np
import struct
import math


timeStep= 16*2


class Wheel:
    def __init__(self, wheel, maxVelocity):
        self.maxVelocity = maxVelocity
        self.wheel = wheel
        self.wheel.setPosition(float("inf"))
    # Moves the wheel at a ratio of the maximum speed
    def move(self, ratio):
        if ratio > 1:
            ratio = 1
        self.wheel.setVelocity(ratio * self.maxVelocity)
    


class Camera:
    def __init__(self, camera, tileRanges, timeStep):
        self.camera = camera
        self.camera.enable(timeStep)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()
        self.tileRanges = tileRanges
        self.classifyThresh = 20
    

    # Gets an image from the raw camera data
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.height, self.width, 4)))

    def getVictimImagesAndPositions(self):
        img = self.getImg()
        # Hace una copia de la imagen
        img1 = img.copy()
        # Filtra la copia para aislar su elemento azul
        img1[:, :, 2] = np.zeros([img1.shape[0], img1.shape[1]])
        # Hace una version es escala de grises
        gray = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
        # Hace un thershold para hacer la imagen binaria
        thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)[1]
        #cv.imshow("thresh", thresh)
        # Encuentra los contornos, aunque se puede confundir con el contorno de la letra
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # Pra evitar la confusion dibuja rectangulos blancos donde estan los contornos en la imagen y despues vuelve a
        # sacar los contornos para obtener solo los del rectangulo, no los de las letras.
        for c0 in contours:
            x, y, w, h = cv.boundingRect(c0)
            cv.rectangle(thresh, (x, y), (x + w, y + h), (225, 255, 255), -1)
        #cv.imshow("thresh2", thresh)
        contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # saca las medidas y la posicion de los contornos y agrega a la lista de imagenes la parte esa de la imagen original
        # Tambien anade la posicion de cada recuadro en la imagen original
        finalPoses = []
        finalImages = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            finalImages.append(img[y:y + h, x:x + w])
            finalPoses.append((y, x))
        return finalPoses, finalImages


# Reads the colour sensor
class ColourSensor:
    def __init__(self, sensor, distancefromCenter, timeStep):
        self.distance = distancefromCenter
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0
    
    def getPosition(self, robotGlobalPosition, robotGlobalRotation):
        relPosition = getCoords(robotGlobalRotation, self.distance)
        return [robotGlobalPosition[0] + relPosition[0], robotGlobalPosition[1] + relPosition[1]]
    
    def __update(self):
        colour = self.sensor.getImage()
        self.r = colour[0]
        self.g = colour[1]
        self.b = colour[2]
    
    def __isTrap(self):
        return (57 < self.r < 61 and 57 < self.g < 61) or (self.r == 111 and self.g == 111)
    def __isSwamp(self):
        return (144 > self.r > 140 and 225 > self.g > 220 and self.b == 246)
    def __isCheckpoint(self):
        return (self.r == 255 and self.g == 255 and self.b == 255)
    def __isNormal(self):
        return self.r == 252 and self.g == 252
    # Returns the type of tyle detected from the colour data
    def getTileType(self):
        self.__update()
        tileType = "undefined"
        if self.__isNormal():
            tileType = "normal"
        elif self.__isTrap():
            tileType = "trap"
        elif self.__isSwamp():
            tileType = "swamp"
        elif self.__isCheckpoint():
            tileType = "checkpoint"
        return tileType







maxVelocity = 6.28
myRobot = Robot()
# Wheels
leftWheel = Wheel(myRobot.getMotor("left wheel motor"), maxVelocity)
rightWheel = Wheel(myRobot.getMotor("right wheel motor"), maxVelocity)

def move(velocity):
        rightWheel.move(velocity)
        leftWheel.move(velocity)
#Cameras
cameras = {
    "centre":Camera(myRobot.getCamera("camera_centre"), ((50, 105), ), timeStep),
    "right":Camera(myRobot.getCamera("camera_right"), ("undefied", (13, 32)), timeStep),
    "left":Camera(myRobot.getCamera("camera_left"), ("undefied", (13, 32)), timeStep)
        }
        
        #Colour sensor
colourSensor = ColourSensor(myRobot.getCamera("colour_sensor"), robotDiameter / 2 + colourSensorOffset, timeStep)
while myRobot.step(timeStep) != -1:
    move(maxVelocity)
    