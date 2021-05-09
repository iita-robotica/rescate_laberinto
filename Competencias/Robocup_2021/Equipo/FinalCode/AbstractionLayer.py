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


timeStep = 16 * 2

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
                self.gridPlottingArray[x][y] = 100
        for x in range(0, len(self.gridPlottingArray), int(self.tileSize * scale)):
            for y in range(len(self.gridPlottingArray[0])):
                self.gridPlottingArray[x][y] = 100
    
    def plotPoint(self, point, value):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)
                
        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            self.gridPlottingArray[finalx][finaly] = value
    
    def getPoint(self, point):
        procPoint = [int(point[0] * self.scale), int(point[1] * self.scale)]
        finalx = procPoint[0] + int(self.offsets[0] * self.tileSize)
        finaly = procPoint[1] + int(self.offsets[1] * self.tileSize)
                
        if self.size[0] * -1 < finalx < self.size[0] and self.size[0] * -1 < finaly < self.size[1]:
            return self.gridPlottingArray[finalx][finaly]



class AbstractionLayer():

    def __init__(self):
        #Variables
        self.tileSize = 0.06
        self.gridPlotter = PlottingArray((600, 600), [3000, 3000], 300, self.tileSize)
        self.doWallMapping = False

        # Components
        self.robot = RobotLayer(timeStep)
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
    
    def doLoop(self):
        return self.robot.doLoop()
    


    def update(self):
        self.robot.update()
        
        if self.doWallMapping:
            print("Doing wall mapping")
            pointCloud = self.robot.getDetectionPointCloud()
            
            
            for point in pointCloud:
                
                if self.gridPlotter.getPoint(point) < 250:
                    self.gridPlotter.plotPoint(point, self.gridPlotter.getPoint(point) + 5)
            
            
            self.analyst.loadPointCloud(pointCloud)
            self.analyst.setTileInGrid(self.position, "hole")

            """
            for point in self.analyst.converter.getTotalPointCloud():
                ppoint = [point.position[0] / 100, point.position[1] / 100]

                self.gridPlotter.plotPoint(ppoint, 100)
            """
            

        
        self.gridPlotter.plotPoint(self.position, 255) 

        self.analyst.showGrid()
        
        
        cv.imshow("raw detections", cv.resize(self.gridPlotter.gridPlottingArray, (400, 400), interpolation=cv.INTER_NEAREST))
        cv.waitKey(1)