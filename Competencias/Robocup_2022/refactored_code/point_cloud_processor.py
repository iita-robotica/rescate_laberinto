import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np

from bresenham import bresenham

MAP_SCALE = 850

def processPointCloud(pc, robotPos):
    return [[round((pos[0] + robotPos[0]) * MAP_SCALE), round((pos[1]  + robotPos[1]) * MAP_SCALE)] for pos in pc]

    "For each point in the poincloud, sum its position with the position of the robot"
    """
    newPC = []
    for point in pc:
        newPC.append([point[0] + robotPos[0], point[1] + robotPos[1]])
    return newPC
    """



def get_intermediate_points(point_cloud, center_point=[0, 0]):
    seen_points = list()
    for point in point_cloud:
        intermediates = list(bresenham(center_point[0], center_point[1], point[0], point[1]))
        #intermediates.remove(point)
        seen_points += intermediates
    return seen_points




if __name__ == '__main__':
    pass

