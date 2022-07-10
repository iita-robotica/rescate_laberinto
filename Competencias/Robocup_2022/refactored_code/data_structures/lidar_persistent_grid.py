import numpy as np
import cv2 as cv

import utilities

from data_structures import resizable_pixel_grid

class LidarGrid(resizable_pixel_grid.Grid):
    def __init__(self, input_resolution, resolution, threshold=0):
        self.input_res = input_resolution
        self.res = resolution
        self.multiplier = self.res / self.input_res
        self.frame = 0
        
        self.shape = (self.res, self.res)
        super().__init__(self.shape, self.res)
        self.threshold = 100
        self.delete_threshold = 2

    def get_bool_array(self):
        return self.grid > self.threshold
    
    def clean_up(self):
        print("Cleaning up lidar grid")
        self.grid = self.grid * (self.grid > self.delete_threshold).astype(np.int)

    
    def sum_detection(self, point):
        point = [round(p * self.multiplier) for p in point]
        self.sum_to_point(point, 1)
    
    def print_bool(self, max_size=(600, 600)):
        grid1 = utilities.resize_image_to_fixed_size(self.get_bool_array(), max_size)
        cv.imshow("bool_grid", grid1 * 255)
        cv.waitKey(1)
    
    def update(self, point_cloud):
        self.clean_up()
        for point in point_cloud:
            self.sum_detection(point)
        self.frame += 1

        
    