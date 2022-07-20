import numpy as np
import cv2 as cv
import time
from functools import lru_cache
import sys
#sys.path.append(r"C:\\Users\\ANA\\Desktop\\Webots - Erebus\\rescate_laberinto\\Competencias\\Robocup_2021\\Equipo\\FinalCode")
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
class PointCloudQueManager:

    def __init__(self, queSize, pointMultiplier, queStep = 1):
        self.queSize = queSize # Defines the size of the point cloud que
        self.que = [] # A que of point clouds
        self.queStep = queStep
        self.queActualStep = 0
        self.pointMultiplier = pointMultiplier
        for _ in range(queSize):
            self.que.append([])
    
    # Converts the point coords in to ints and multiplies it with the point multiplier
    def processPointCloud(self, pointCloud):

        finalPointCloud = []
        for point in pointCloud:
            fpoint = [round(point[0] * self.pointMultiplier), round(point[1] * self.pointMultiplier), 1]
            alreadyInFinal = False
            for finalPoint in finalPointCloud:
                if fpoint[:2] == finalPoint[:2]:
                    finalPoint[2] += 1
                    alreadyInFinal = True
            if not alreadyInFinal:
                finalPointCloud.append(fpoint)
        return finalPointCloud
    
    # Merges all the point clouds in the que and returns them
    def getTotalPointCloud(self):
        totalPointCloud = []
        isInFinal = False
        for pointCloud in self.que:
            for item in pointCloud:
                for totalItem in totalPointCloud:
                    if item[:2] == totalItem[:2]:
                        isInFinal = True
                        totalItem[2] += item[2]
                if not isInFinal:
                    totalPointCloud.append(item)
        #print("total point cloud: ", totalPointCloud)
        return totalPointCloud

    # Adds a new point cloud to the que and removes the last element
    def update(self, pointCloud):
        if self.queActualStep >= self.queStep:
            self.que.pop(0)
            self.que.append(self.processPointCloud(pointCloud))
            self.queActualStep = 0
        else:
            self.queActualStep += 1
    
    

class PointCloudDivider:
    def __init__(self, tileSize, pointMultiplier, pointPermanenceThresh):
        # Defines the size of a tie in the scale of the original coordinates
        self.tileSize = tileSize
         # Defines the number to multiply the coordinates to convert them in to ints
        self.pointMultiplier = pointMultiplier
        # Defines the number of times a point has to repeat to be considered definitive
        self.pointPermanenceThresh = pointPermanenceThresh
        self.realTileSize = self.tileSize * self.pointMultiplier
    
    

    def getTile(self, position):
        return (int(position[0] // self.realTileSize), int(position[1] // self.realTileSize))
    
    def getPosInTile(self, position):
        tile = self.getTile(position)
        tilePosition = multiplyLists(tile, [self.realTileSize, self.realTileSize])
        posInTile = [round(round(position[0]) - tilePosition[0]), round(round(position[1]) - tilePosition[1])]
        return posInTile
    
    # Returns a list with dictionarys containing the tile number and the position inside of said tile
    def getTiles(self, totalPointCloud):
        tiles = []
        #print("Total Point Cloud: ", totalPointCloud)
        for item in totalPointCloud:
            inTiles = False
            if item[2] >= self.pointPermanenceThresh:
                #print(item[:2])
                itemTile = self.getTile(item[:2])
                itemPosInTile = self.getPosInTile(item[:2])
                for tile in tiles:
                    if tile["tile"] == itemTile:
                        inTiles = True
                        tile["posInTile"].append(itemPosInTile)

                if not inTiles:
                    tiles.append({"tile":itemTile, "posInTile":[itemPosInTile]})
        #print("Tiles: ", tiles)
        return tiles

class PointCloudConverter:

    def __init__(self, tileSize, pointMultiplier):
        self.queManager = PointCloudQueManager(queSize=5, pointMultiplier=pointMultiplier)
        self.divider = PointCloudDivider(tileSize, pointMultiplier, pointPermanenceThresh=30)
        self.totalPointCloud = []
    

    def loadPointCloud(self, pointCloud):
        self.queManager.update(pointCloud)
        
    
    def getTilesWithPoints(self):
        self.totalPointCloud = self.queManager.getTotalPointCloud()
        return (self.divider.getTiles(self.totalPointCloud))
    

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
                elementsDict[key] += modelGrid[point[0]][point[1]]
                    
        return elementsDict