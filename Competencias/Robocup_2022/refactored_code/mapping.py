import numpy as np
import copy
import cv2 as cv

import utilities
from data_structures.node_grid import NodeGrid
from data_processing import camera_processing
from data_processing import point_cloud_processor

class Grid(NodeGrid):
    def __init__(self):
        pass

    def load_floor(tiles:dict, robot_vortex:list):
        """
        tiles = {
            (x,y): "color",
        }
        robot_vortex = [x,y]
        """
        pass