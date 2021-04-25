import numpy as np
import cv2 as cv
import sys
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *

class ConverterPoint:

    def __init__(self, startingTime, maxTime, definitiveThreshold):
        self.definitive = False
        self.count = 0
        self.maxTime = maxTime
        self.definitiveThreshold = definitiveThreshold
        self.startingTime = startingTime
        self.time = 0
    
    def update(self, time):
        self.time = time - self.startingTime
        if not self.definitive:
            if self.count >= self.definitiveThreshold:
                self.definitive = True
            elif self.time > self.maxTime:
                self.count = 0
                self.startingTime = self.time

    def increaseCount(self):
        self.count += 1

class PointCloudToTileConverter:

    __tilesDict = {
    
        "wallRight": np.array([[0, 0, 0, 0, 1, 1],
                            [0, 0, 0, 0, 1, 1],
                            [0, 0, 0, 0, 1, 1],
                            [0, 0, 0, 0, 1, 1],
                            [0, 0, 0, 0, 1, 1],
                            [0, 0, 0, 0, 1, 1]]),

        "wallLeft": np.array([[1, 1, 0, 0, 0, 0],
                            [1, 1, 0, 0, 0, 0],
                            [1, 1, 0, 0, 0, 0],
                            [1, 1, 0, 0, 0, 0],
                            [1, 1, 0, 0, 0, 0],
                            [1, 1, 0, 0, 0, 0]]),
                            
        "wallUp": np.array([[1, 1, 1, 1, 1, 1],
                            [1, 1, 1, 1, 1, 1],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0]]),
                            
        "wallDown": np.array([[0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [1, 1, 1, 1, 1, 1],
                            [1, 1, 1, 1, 1, 1]]),

        "curvedRightUp": np.array([[1, 1, 1, 1, 0, 0],
                                    [1, 1, 1, 1, 1, 0],
                                    [0, 0, 0, 1, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1]]),

        "curvedLeftUp": np.array([[0, 0, 1, 1, 1, 1],
                                [0, 1, 1, 1, 1, 1],
                                [1, 1, 1, 0, 0, 0],
                                [1, 1, 0, 0, 0, 0],
                                [1, 1, 0, 0, 0, 0],
                                [1, 1, 0, 0, 0, 0]]),
                            
        "curvedRightDown": np.array([[0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 1, 1, 1],
                                    [1, 1, 1, 1, 1, 0],
                                    [1, 1, 1, 1, 0, 0]]),
                            
        "curvedLeftDown": np.array([[1, 1, 0, 0, 0, 0],
                                    [1, 1, 0, 0, 0, 0],
                                    [1, 1, 0, 0, 0, 0],
                                    [1, 1, 1, 0, 0, 0],
                                    [0, 1, 1, 1, 1, 1],
                                    [0, 0, 1, 1, 1, 1]])
        }

    def __init__(self, size, initialTime, maxPointTime, pointPermanenceThreshold):

        self.maxPointTime = maxPointTime
        self.pointPermanenceThresh = pointPermanenceThreshold
        self.possibleGrid = self.makeGrid(initialTime)
        self.size = size

    def makeGrid(self, time):
        row = []
        for _ in self.size:
            row.append(ConverterPoint(time, self.maxPointTime, self.pointPermanenceThresh))
        grid = []
        for _ in self.size:
            grid.append(row.copy())
        return np.array(grid)


    def update(self, time, pointCloud):
        for point in pointCloud:
            self.possibleGrid[point[0], point[1]].increaseCount()
            self.possibleGrid[point[0], point[1]].update(time)

    def getPossiblePercenteges(self):
        accuracyDict = {}
        for key in self.__tilesDict.keys():
            accuracyDict[key] = 0
        for i in range(self.size):
            for j in range(self.size):
                isDefinitive = self.possibleGrid[j][i].isDefinitive
                for key in key in self.__tilesDict.keys():
                    if self.__tilesDict[key][j][i] == isDefinitive:
                        accuracyDict[key] += 1
        return accuracyDict


class PointCloudToNodeGridConverter:
    def __init__(self):
        self.pointCloud = []