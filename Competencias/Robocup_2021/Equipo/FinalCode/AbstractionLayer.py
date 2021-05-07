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

class AbstractionLayer():

    def __init__(self):
        #Variables
        self.tileSize = 0.06
        self.gridPlottingArray= np.zeros((400, 400), np.uint8)
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
        if self.seqMg.simpleSeqEvent(): self.robot.rotationDetectionType = "gps"
        self.seqMoveWheels(1, 1)
        self.seqDelaySec(0.2)
        if self.seqMg.simpleSeqEvent(): self.robot.rotationDetectionType = "gyroscope"
        self.seqDelaySec(0.2)
        self.seqMoveWheels(0, 0)
        self.seqMoveWheels(-1, -1)
        self.seqDelaySec(0.4)
        self.seqMoveWheels(0, 0)
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
        pointCloud = self.robot.getDetectionPointCloud()
        
        for point in pointCloud:
            procPoint = [int(point[0] * 100), int(point[1] * 100)]
            finalx = procPoint[0] - 100
            finaly = procPoint[1] - 100
            if self.gridPlottingArray[finalx][finaly] < 250:
                self.gridPlottingArray[finalx][finaly] += 1

        
        self.analyst.loadPointCloud(pointCloud)
        self.analyst.setTileInGrid(self.position, "hole")
        print(self.analyst.getGrid())
        self.analyst.showGrid()
        
        cv.imshow("raw detections", self.gridPlottingArray)
        cv.waitKey(1)