import numpy as np
import cv2 as cv
import time
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

        "curvedRight-up": np.array([[1, 1, 1, 1, 0, 0],
                                    [1, 1, 1, 1, 1, 0],
                                    [0, 0, 0, 1, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1]]),

        "curvedLeft-up": np.array([[0, 0, 1, 1, 1, 1],
                                [0, 1, 1, 1, 1, 1],
                                [1, 1, 1, 0, 0, 0],
                                [1, 1, 0, 0, 0, 0],
                                [1, 1, 0, 0, 0, 0],
                                [1, 1, 0, 0, 0, 0]]),
                            
        "curvedRight-down": np.array([[0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 0, 1, 1],
                                    [0, 0, 0, 1, 1, 1],
                                    [1, 1, 1, 1, 1, 0],
                                    [1, 1, 1, 1, 0, 0]]),
                            
        "curvedLeft-down": np.array([[1, 1, 0, 0, 0, 0],
                                    [1, 1, 0, 0, 0, 0],
                                    [1, 1, 0, 0, 0, 0],
                                    [1, 1, 1, 0, 0, 0],
                                    [0, 1, 1, 1, 1, 1],
                                    [0, 0, 1, 1, 1, 1]])
        }

    def __init__(self, size, initialTime, maxPointTime, pointPermanenceThreshold):
        self.maxPointTime = maxPointTime
        self.pointPermanenceThresh = pointPermanenceThreshold
        self.size = size
        self.possibleGrid = self.makeGrid(initialTime)
        self.elements = []
        self.validTileDictKeys = list(self.__tilesDict.keys())
    
    @property
    def straightWalls(self):
        stW = []
        for el in self.elements:
            if el[:4] == "wall":
                stW.append(el[4:].lower())
        return stW
    
    @property
    def curvedWalls(self):
        crvW = []
        for el in self.elements:
            if el[:6] == "curved":
                crvW.append(el[6:].lower())
        return crvW


    def makeGrid(self, time):
        grid = []
        for _ in range(self.size): 
            row = []
            for _ in range(self.size):
                item = ConverterPoint(time, self.maxPointTime, self.pointPermanenceThresh)
                row.append(item)
            grid.append(row.copy())
        return np.array(grid)

    def update(self, time, pointCloud):
        if len(pointCloud):
            for point in pointCloud:
                self.possibleGrid[point[0]][point[1]].increaseCount()
                self.possibleGrid[point[0]][point[1]].update(time)
            elements = self.calculateElements()
            for el in elements:
                self.validTileDictKeys.remove(el)
            self.elements.extend(elements) 
            

    def getSimilarityWithTiles(self):
        accuracyDict = {}
        
        for key in self.validTileDictKeys:
            accuracyDict[key] = 0
        for i in range(self.size):
            for j in range(self.size):
                isDefinitive = self.possibleGrid[i][j].definitive
                for key in self.validTileDictKeys:
                    if isDefinitive:
                        if self.__tilesDict[key][i][j] == 1:
                            accuracyDict[key] += 1
        return accuracyDict
    
    def getPrintableArray(self):
        pGrid = []
        for j in range(self.size):
            row = []
            for i in range(self.size):
                row.append(int(self.possibleGrid[j][i].definitive))
            pGrid.append(row)
        return pGrid
    
    def calculateElements(self):
        curveThresh = 8
        wallThresh = 4
        sim = self.getSimilarityWithTiles()
        maxValue = 0
        maxKey = ""
        wallKeys = []
        for key, value in sim.items():
            if value > maxValue:
                maxValue = value
                maxKey = key
            if key[:4] == "wall" and value > wallThresh:
                wallKeys.append(key)
        if maxKey[:6] == "curved" and maxValue > curveThresh:
            return [maxKey]
        return wallKeys


class PointCloudToNodeGridConverter:
    def __init__(self, resolutionPerTile, similarityDict, ):
        self.pointCloud = []


conv = PointCloudToTileConverter(6, time.time(), 1, 3)

pointCloud = [        [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], 
              [1, 0], [1, 1], [1, 2], [1, 3], [1, 4], [1, 5],
              [2, 0], [2, 1],
              [3, 0], [3, 1],
              [4, 0], [4, 1],
              [5, 0], [5, 1]]

"""pointCloud = [        [0, 1],
              [1, 5], [1, 1],
              [2, 5], [2, 1],
              [3, 5], [3, 1],
              [4, 5], [4, 1],
              [5, 5], [5, 1]]"""

#pointCloud = [[1, 0],]

for _ in range(3):
    conv.update(time.time(), pointCloud)

print(conv.getSimilarityWithTiles())
print(conv.getPrintableArray())
print("Elements: " + str(conv.elements))
print("Straight walls: "  + str(conv.straightWalls))
print("Curved walls: " + str(conv.curvedWalls))