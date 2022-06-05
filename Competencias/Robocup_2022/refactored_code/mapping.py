import numpy as np
import copy
import cv2 as cv

import utilities
from data_processing import camera_processor, data_extractor, point_cloud_processor
from data_structures import lidar_persistent_grid, expandable_node_grid

class Mapper:
    def __init__(self, tile_size):
        # Data structures
        self.node_grid = expandable_node_grid.Grid((1, 1))
        self.lidar_grid = lidar_persistent_grid.LidarGrid(tile_size, 6, 100)

        # Data processors
        self.point_cloud_processor = point_cloud_processor.PointCloudProcessor(350, 850)

        # Data extractors
        self.point_cloud_extractor = data_extractor.PointCloudExtarctor(6, 5)
        self.floor_color_extractor = data_extractor.FloorColorExtractor()
    
    def load_point_cloud(self, point_cloud, robot_position):
        point_cloud = self.point_cloud_processor.processPointCloud(point_cloud, robot_position)
        self.lidar_grid.update(point_cloud)
    
    def lidar_to_node_grid(self):
        grid, offsets = self.point_cloud_extractor.transform_to_grid(self.lidar_grid)
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                xx = (x - offsets[0]) * 2 + 1
                yy = (y - offsets[1]) * 2 + 1
                for direction in value:
                    self.node_grid.load_straight_wall((xx, yy),  direction)

    
        
    def update(self, point_cloud, robot_position):
        in_bounds_point_cloud, out_of_bounds_point_cloud = point_cloud
        self.load_point_cloud(in_bounds_point_cloud, robot_position)
        self.lidar_to_node_grid()

        self.lidar_grid.print_grid((600, 600))
        self.lidar_grid.print_bool((600, 600))  

        self.node_grid.print_grid()
    
    def get_node_grid(self):
        return copy.deepcopy(self.node_grid)