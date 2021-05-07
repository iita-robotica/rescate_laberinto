import numpy as np
import cv2 as cv
import time
import sys
sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
from UtilityFunctions import *


# Point class for point clou to grid converter
class PointCloudConverterPoint:

    def __init__(self, position):
        self.position = position
        self.count = 1

    # Defines the "==" operator
    def __eq__(self, other):
        return self.position == other.position

    # Defines what to print if I ask to print it
    def __repr__(self):
        return str(self.position + [self.count])
    def __str__(self):
        return str([self.position + [self.count]])


# Converts a point cloud in to tiles with positions
class PointCloudToGridConverter:

    def __init__(self, queSize, pointMultiplier, tileSize, pointPermanenceThresh=1):
        self.queSize = queSize # Defines the size of the point cloud que
        self.que = [] # A que of point clouds
        # Defines the number to multiply the coordinates to convert them in to ints
        self.pointMultiplier = pointMultiplier
        # Defines the number of times a point has to repeat to be considered definitive
        self.pointPermanenceThresh = pointPermanenceThresh
        # Defines the size of a tie in the scale of the original coordinates
        self.tileSize = tileSize
        self.realTileSize = self.tileSize * self.pointMultiplier
        for _ in range(queSize):
            self.que.append([])
    
    # Converts the point coords in to ints and multiplies it with the point multiplier
    def processPointCloud(self, pointCloud):
        processedPointCloud = []
        for point in pointCloud:
            p = [int(point[0] * self.pointMultiplier), int(point[1] * self.pointMultiplier)]
            procPoint = PointCloudConverterPoint(p)
            processedPointCloud.append(procPoint)

        finalPointCloud = []
        for point in processedPointCloud:
            alreadyInFinal = False
            for finalPoint in finalPointCloud:
                if point == finalPoint:
                    finalPoint.count += 1
                    alreadyInFinal = True
            if not alreadyInFinal:
                finalPointCloud.append(point)
        return finalPointCloud
    
    # Merges all the point clouds in the que and returns them
    def getTotalPointCloud(self):
        totalPointCloud = []
        isInFinal = False
        for pointCloud in self.que:
            for item in pointCloud:
                for totalItem in totalPointCloud:
                    if item == totalItem:
                        isInFinal = True
                        totalItem.count += item.count
                if not isInFinal:
                    totalPointCloud.append(item)
        return totalPointCloud

    # Adds a new point cloud to the que and removes the last element
    def update(self, pointCloud):
        self.que.pop(0)
        self.que.append(self.processPointCloud(pointCloud))
    
    def getTile(self, position):
        return [int(position[0] // self.realTileSize), int(position[1] // self.realTileSize)]
    
    def getPosInTile(self, position):
        return [int(position[0] % self.realTileSize), int(position[1] % self.realTileSize)]
    
    # Returns a list with dictionarys containing the tile number and the position iside of said tile
    def getTiles(self):
        tiles = []
        
        realTileSize = self.tileSize * self.pointMultiplier
        totalPointCloud = self.getTotalPointCloud()
        print("Total Point Cloud: ", totalPointCloud)
        for item in totalPointCloud:
            inTiles = False
            if item.count >= self.pointPermanenceThresh:
                itemTile = self.getTile(item.position)
                itemPosInTile = self.getPosInTile(item.position)
                for tile in tiles:
                    if tile["tile"] == itemTile:
                        inTiles = True
                        tile["posInTile"].append(itemPosInTile)

                if not inTiles:
                    tiles.append({"tile":itemTile, "posInTile":[itemPosInTile]})
        print("Tiles: ", tiles)
        return tiles

# Uses a dict of tile templates to return the elements present in a tile given points inside that tile
class Classifier:
    def __init__(self, tilesDict):
        self.tilesDict = tilesDict
        self.validTileDictKeys = list(self.tilesDict.keys())
        self.tilesDictPositivesCount = {}
        for key, value in self.tilesDict.items():
            count = 0
            for y in range(len(value)):
                for x in range(len(value[y])):
                    count += value[x][y]
            self.tilesDictPositivesCount[key] = count

    # Returns the similarity of the points imputted to the templates in percentages
    def getCalsificationPercentages(self, pointList):
        elementsDict = {}
        for key in self.validTileDictKeys:
            elementsDict[key] = 0

        for point in pointList:
            for key, modelGrid in self.tilesDict.items():
    
                elementsDict[key] += (modelGrid[point[0]][point[1]] * 100 // self.tilesDictPositivesCount[key])
                    
        return elementsDict


if __name__ == "__main__":
    tilesDict = {
        
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


    conv = PointCloudToGridConverter(5, 100, 0.06, 2)
    classifier = Classifier(tilesDict)

    #conv.update([[0.9, 0.6], [0.9, 0.21], [0.2, 0.3]])
    for _ in range(3):
        conv.update([[0.1, 0.1], [0.1, 0.09], [0.2, 0.3]])
    print(conv.que)
    print(conv.getTotalPointCloud())
    print(conv.getTiles())
        
    print(classifier.getCalsificationPercentages([[0,0], [0,1], [0,2], [0,3], [0,4], [0,5]]))


