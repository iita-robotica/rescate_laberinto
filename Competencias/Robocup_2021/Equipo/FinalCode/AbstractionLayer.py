from controller import Robot
import sys
import numpy as np
import cv2 as cv
#REMEMBER TO COPY-PASTE THIS FUNCTIONS ON TO FINAL CODE
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *
from StateMachines import *
from RobotLayer import RobotLayer
import Analysis


class PlottingArray:
    def __init__(self, size, offsets, scale, tileSize):
        self.scale = scale
        self.size = size
        self.offsets = offsets
        self.scale = scale
        self.tileSize = tileSize
        self.gridPlottingArray = np.zeros(self.size, np.uint8)
        
        for y in range(0, len(self.gridPlottingArray), int(self.tileSize * scale)):
            for x in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50
        for x in range(0, len(self.gridPlottingArray), int(self.tileSize * scale)):
            for y in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50
        
    
    def plotPoint(self, point, value):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale * -1)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)
                
        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            self.gridPlottingArray[finalx][finaly] = value
    
    def getPoint(self, point):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale * -1)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)
                
        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            return self.gridPlottingArray[finalx][finaly]
    
    def reset(self):
        self.gridPlottingArray = np.zeros(self.size, np.uint8)

        for y in range(0, len(self.gridPlottingArray), int(self.tileSize * self.scale)):
            for x in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50
        for x in range(0, len(self.gridPlottingArray), int(self.tileSize * self.scale)):
            for y in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 50



class AbstractionLayer():

    def __init__(self):
        #Variables
        self.tileSize = 0.06
        self.timeStep = 32
        self.gridPlotter = PlottingArray((300, 300), [1500, 1500], 150, self.tileSize)
        self.doWallMapping = False
        self.actualTileType = "undefined"
        self.timeInRound = 8 * 60
        self.timeWithoutMoving = 0
        self.__timeWithoutMovingStart = 0

        # Components
        self.robot = RobotLayer(self.timeStep)
        self.seqMg = SequenceManager()
        self.analyst = Analysis.Analyst(self.tileSize)

        # -- Functions --
        self.seqPrint = self.seqMg.makeSimpleSeqEvent(print)
        self.seqDelaySec = self.seqMg.makeComplexSeqEvent(self.robot.delaySec)
        self.seqMoveWheels = self.seqMg.makeSimpleSeqEvent(self.robot.moveWheels)
        self.seqRotateToDegs = self.seqMg.makeComplexSeqEvent(self.robot.rotateToDegs)
        self.seqMoveToCoords = self.seqMg.makeComplexSeqEvent(self.robot.moveToCoords)

    def calibrate(self):
        self.seqMg.startSequence()
        self.seqDelaySec(0.5)
        if self.seqMg.simpleSeqEvent(): 
            actualTile = [self.position[0] // self.tileSize, self.position[1] // self.tileSize]
            self.robot.positionOffsets =  [round((actualTile[0] * self.tileSize) - self.position[0]) + self.tileSize // 2, round((actualTile[1] * self.tileSize) - self.position[1]) + self.tileSize // 2]
            self.robot.positionOffsets = [self.robot.positionOffsets[0] % self.tileSize, self.robot.positionOffsets[1] % self.tileSize]

            print("positionOffsets: ", self.robot.positionOffsets)
        if self.seqMg.simpleSeqEvent(): self.analyst.registerStart()
        self.seqDelaySec(0.5)
        
        if self.seqMg.simpleSeqEvent(): self.robot.rotationDetectionType = "gps"
        self.seqMoveWheels(1, 1)
        self.seqDelaySec(0.2)
        if self.seqMg.simpleSeqEvent(): self.robot.rotationDetectionType = "gyroscope"
        self.seqDelaySec(0.2)
        self.seqMoveWheels(0, 0)
        self.seqMoveWheels(-1, -1)
        self.seqDelaySec(0.4)
        self.seqMoveWheels(0, 0)
        if self.seqMg.simpleSeqEvent(): self.doWallMapping = True
        return self.seqMg.seqResetSequence()
    
    @property
    def rotation(self):
        return self.robot.rotation
    
    @property
    def position(self):
        return self.robot.globalPosition
    
    @property
    def prevPosition(self):
        return self.robot.prevGlobalPosition

    
    def getBestPos(self):
        return self.analyst.getBestPosToMove()
    
    def doLoop(self):
        return self.robot.doLoop()
    
    def recalculatePath(self):
        self.analyst.calculatePath = True
    
    def isVictims(self):
        victims = self.robot.getVictims()
        if len(victims) and not self.analyst.isRegisteredVictim():
            return True
        return False

    def reportVictims(self):
        victims = self.robot.getVictims()
        if len(victims):
            self.robot.reportVictims(victims[0])
        self.analyst.registerVictim()
    
    def endGame(self):
        self.sendFinalArray()
        self.robot.sendEnd()

    def sendFinalArray(self):
        self.robot.sendArray(self.analyst.getArrayRepresentation())

    def isEnded(self):
        return self.analyst.ended
    
    @property
    def timeLeft(self):
        return self.timeInRound - self.robot.time

    def update(self):
        self.robot.update()

        print("Time:", self.robot.time)
        print("time without moving: ", self.timeWithoutMoving)
        print("time left:", self.timeLeft)
        diff = [self.position[0] - self.prevPosition[0], self.position[1] - self.prevPosition[1]]
        if self.robot.getWheelDirection() < 0.1:
            self.timeWithoutMoving = 0
        elif -0.0001 < getDistance(diff) < 0.0001:
            if self.timeWithoutMoving == 0:
                self.__timeWithoutMovingStart = self.robot.time
                self.timeWithoutMoving = 0.000000001
            else:
                self.timeWithoutMoving = self.robot.time - self.__timeWithoutMovingStart
        else:
            self.timeWithoutMoving = 0

        if self.doWallMapping:
            print("Doing wall mapping")

            if self.timeWithoutMoving > 1:
                self.analyst.stoppedMoving = True
            else:
                self.analyst.stoppedMoving = False


            pointCloud = self.robot.getDetectionPointCloud()
            
            """
            for point in pointCloud:
                
                if self.gridPlotter.getPoint(point) < 250:
                    self.gridPlotter.plotPoint(point, self.gridPlotter.getPoint(point) + 5)
            """
            #tileType = self.robot.get
            self.analyst.loadPointCloud(pointCloud)
            
        colorPos, self.actualTileType = self.robot.getColorDetection()
        print("Tile type: ", self.actualTileType)
        self.analyst.loadColorDetection(colorPos, self.actualTileType)
        self.analyst.update(self.position, self.rotation)

            
        self.gridPlotter.reset()
        for point in self.analyst.converter.totalPointCloud:
            if point[2] > 20:
                ppoint = [point[0] / 100, point[1] / 100]
                self.gridPlotter.plotPoint(ppoint, 100)
            
        bestPos = self.analyst.getStartRawNodePos()
        if bestPos is not None:
            self.gridPlotter.plotPoint(bestPos, 255)
            
        bestPos = self.analyst.getBestPosToMove()
        if bestPos is not None:
            self.gridPlotter.plotPoint(bestPos, 200)        
            
        
        #self.gridPlotter.plotPoint(self.position, 150)

        
        bestPoses = self.analyst.getBestPoses()
        for bestPos in bestPoses:
            self.gridPlotter.plotPoint(bestPos, 255)
        
        

        self.analyst.showGrid()
        
        
        cv.imshow("raw detections", cv.resize(self.gridPlotter.gridPlottingArray, (400, 400), interpolation=cv.INTER_NEAREST))
        cv.waitKey(1)

        