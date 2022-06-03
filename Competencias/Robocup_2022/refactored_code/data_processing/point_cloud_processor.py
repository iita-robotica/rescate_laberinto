import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import copy

from bresenham import bresenham

MAP_SCALE = 850

def processPointCloud(pc, robotPos):
    return [[pcv + rpv for pcv, rpv in zip(pos, robotPos)] for pos in pc]

def processPointCloudForCamera(pc, robotPos):
    if len(pc) == 0:
        return np.empty((0, 2), dtype=np.int)
    return np.array([[int(pos[0]  * MAP_SCALE + 350), int(pos[1] * MAP_SCALE + 350)] for pos in pc], dtype=np.int)



def get_intermediate_points(point_cloud, center_point=[0, 0]):
    seen_points = list()
    for point in point_cloud:
        intermediates = list(bresenham(center_point[0], center_point[1], point[0], point[1]))
        #intermediates.remove(point)
        seen_points += intermediates
    return seen_points

straight_template = np.zeros((6, 6), dtype=np.int)
straight_template[:][0:2] = 1

templates = {}

for i, name in enumerate(["u", "l", "d", "r"]):
    templates[name] = np.rot90(straight_template, i)

def get_tile_status(min_x, min_y, max_x, max_y, point_cloud):
    threshold = 5

    counts = {}
    for name in templates:
        counts[name] = 0
    square = point_cloud[min_x:max_x, min_y:max_y]
    if square.shape != (6, 6):
        return []
    for name, template in templates.items():
        for i in range(6):
            for j in range(6):
                if square[i][j] == 1:
                    counts[name] += template[i][j]

    for name, count in counts.items():
        if count >= threshold:
            yield name

def transform_to_grid(point_cloud):
    offsets = point_cloud.offsets
    offsets = [o % 6 for o in offsets]
    grid = []
    bool_array_copy = point_cloud.get_bool_array()
    bool_array_copy = bool_array_copy.astype(np.uint8) * 100
    for x in range(offsets[0], bool_array_copy.shape[0] - 6, 6):
        row = []
        for y in range(offsets[1], bool_array_copy.shape[1] - 6, 6):
            min_x = x
            min_y = y
            max_x = x + 6
            max_y = y + 6
            #print(min_x, min_y, max_x, max_y)
            bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)
            
            val = get_tile_status(min_x, min_y, max_x, max_y, point_cloud.get_bool_array())
            row.append(list(val))
        grid.append(row)
    factor = 10
    cv.imshow("bool_array_copy", cv.resize(bool_array_copy.astype(np.uint8), (point_cloud.shape[1] * factor, point_cloud.shape[0] * factor), interpolation=cv.INTER_NEAREST))
    return grid, [o // 6 for o in offsets]
    




if __name__ == '__main__':
    pass

