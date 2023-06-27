import numpy as np
import cv2 as cv
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.angle import Angle
import imutils
from copy import copy, deepcopy
from robot.devices.camera import CameraImage
from typing import List

class ColorFilter:
    def __init__(self, lower_hsv, upper_hsv):
        self.lower = np.array(lower_hsv)
        self.upper = np.array(upper_hsv)
    
    def filter(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask

class FloorMapper:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid, tile_resolution, tile_size, camera_distance_from_center) -> None:
        self.pixel_grid = pixel_grid
        self.tile_resolution = tile_resolution
        self.tile_size = tile_size
        self.pixel_per_m = tile_resolution / tile_size
        self.pov_distance_from_center = round(0.079 * self.pixel_per_m) 
        #self.hole_color_filter = ColorFilter((0, 0, 10), (0, 0, 106))
        self.hole_color_filter = ColorFilter((0, 0, 10), (0, 0, 27))
        self.swamp_color_filter = ColorFilter((19, 112, 32), (19, 141, 166))
        self.checkpoint_color_filter = ColorFilter((95, 0, 65), (128, 122, 198))
        self.wall_color_filter = ColorFilter((90, 61,  0), (100, 150, 255))

        tiles_up = 0
        tiles_down = 1
        tiles_sides = 1

        min_x = self.tile_resolution * tiles_sides
        max_x = self.tile_resolution * (tiles_sides + 1)
        min_y = self.tile_resolution * tiles_down
        max_y = self.tile_resolution * (tiles_down + 1)

        self.center_tile_points_in_final_image = np.array(((min_x, min_y),
                                                           (max_x, min_y),
                                                           (max_x, max_y),
                                                           (min_x, max_y),), dtype=np.float32)
        
        self.center_tile_points_in_input_image = np.array(([0, 24],  [39, 24], [32, 16], [7, 16]), dtype=np.float32)

        self.flattened_image_shape = (self.tile_resolution * (tiles_sides * 2 + 1),
                                      self.tile_resolution * (tiles_up + tiles_down + 1))
        
        self.final_povs_shape = (120, 120)
        self.distance_to_center_gradient = self.__get_distance_to_center_gradient(self.final_povs_shape)

    def flatten_camera_pov(self, camera_pov: np.ndarray):
        ipm_matrix = cv.getPerspectiveTransform(self.center_tile_points_in_input_image, 
                                                self.center_tile_points_in_final_image, 
                                                solveMethod=cv.DECOMP_SVD)
        
        ipm = cv.warpPerspective(camera_pov, ipm_matrix, self.flattened_image_shape, flags=cv.INTER_NEAREST)

        ipm = cv.resize(ipm, self.flattened_image_shape, interpolation=cv.INTER_CUBIC)

        blank_space = np.zeros((self.pov_distance_from_center, self.flattened_image_shape[0], 4), dtype=np.uint8)
        ipm = np.vstack((blank_space, ipm))

        return ipm
    
    def set_in_background(self, pov: np.ndarray, background=None):
        #cv.imshow('pov', pov)
        max_dim = max(pov.shape)
        if background  is None: background = np.zeros((max_dim * 2, max_dim * 2, 4), dtype=np.uint8)

        start = (max_dim, max_dim - round(pov.shape[1] / 2))
        end =  (start[0] + pov.shape[0], start[1] + pov.shape[1])
        
        background[start[0]:end[0], start[1]:end[1], :] = pov[:,:,:]

        #cv.imshow("pov in background", background)

        return background
    

    def get_global_camera_orientations(self, robot_orientation: Angle):
        global_camera_orientations = []
        for camera_orientation in self.pixel_grid.camera_orientations:
            o = camera_orientation + robot_orientation
            o.normalize()
            global_camera_orientations.append(o)
        
        return global_camera_orientations
    
    def rotate_image_to_angle(self, image: np.ndarray, angle: Angle):
        return imutils.rotate(image, angle.degrees, (image.shape[0] // 2, image.shape[1] // 2))
    

    def get_unified_povs(self, camera_images: List[CameraImage]):
        povs_list = []
        for camera_image in camera_images:
            pov = self.flatten_camera_pov(np.rot90(copy(camera_image.image), k=3))
            pov = np.flip(pov, 1)
            pov = self.set_in_background(pov)
            pov = self.rotate_image_to_angle(pov, camera_image.data.horizontal_orientation)
            povs_list.append(pov)

        return sum(povs_list)
    
    def map_floor(self, camera_images, robot_grid_index):
        povs = self.get_unified_povs(camera_images)

        #cv.imshow("final_pov", povs[:, :, 3])

        self.load_povs_to_grid(robot_grid_index, povs)

    def load_povs_to_grid(self, robot_grid_index, povs):
        
        start = np.array((robot_grid_index[0] - (povs.shape[0] // 2), robot_grid_index[1] - (povs.shape[1] // 2)))
        end = np.array((robot_grid_index[0] + (povs.shape[0] // 2), robot_grid_index[1] + (povs.shape[1] // 2)))

        self.pixel_grid.expand_to_grid_index(start)
        self.pixel_grid.expand_to_grid_index(end)

        start = self.pixel_grid.grid_index_to_array_index(start)
        end = self.pixel_grid.grid_index_to_array_index(end)

        mask = povs[:,:,3] > 254

        povs_gradient = np.zeros_like(self.distance_to_center_gradient)
        povs_gradient[mask] = self.distance_to_center_gradient[mask]


        detection_distance_mask = self.pixel_grid.arrays["floor_color_detection_distance"][start[0]:end[0], start[1]:end[1]] < povs_gradient

        seen_by_camera_mask = self.pixel_grid.arrays["seen_by_camera"][start[0]:end[0], start[1]:end[1]]

        not_walls_mask = self.wall_color_filter.filter(povs) == False

        final_mask = seen_by_camera_mask * detection_distance_mask * not_walls_mask

        self.pixel_grid.arrays["floor_color_detection_distance"][start[0]:end[0], start[1]:end[1]][final_mask] = povs_gradient[final_mask]


        self.pixel_grid.arrays["floor_color"][start[0]:end[0], start[1]:end[1]][final_mask] = povs[:,:,:3][final_mask]

        self.detect_holes()
        self.detect_swamps()
        self.detect_checkpoints()
        
    
    def __get_distance_to_center_gradient(self, shape):
        gradient = np.zeros(shape, dtype=np.float32)
        for x in range(shape[0]):
            for y in range(shape[1]):
                gradient[x, y] = (x - shape[0] // 2) ** 2 + (y - shape[1] // 2) ** 2
        
        gradient = 1 - gradient / gradient.max()

        return (gradient * 255).astype(np.uint8)
    
    def __get_offsets(self, tile_size):
        x_offset = int(self.pixel_grid.offsets[0] % tile_size + tile_size / 2) 
        y_offset = int(self.pixel_grid.offsets[1] % tile_size + tile_size / 2)

        return (x_offset, y_offset)
    
    def offset_array(self, array, offsets):
        return array[offsets[0]:, offsets[1]:]
    
    def get_color_average_kernel(self):
        tile_size = round(self.tile_size * self.pixel_per_m)
        square_propotion = 0.8
        square_size = round(tile_size * square_propotion)

        kernel = np.ones((square_size, square_size), dtype=np.float32)

        kernel = kernel / kernel.sum()

        return kernel
    
    def detect_swamps(self):
        swamp_array = self.swamp_color_filter.filter(self.pixel_grid.arrays["floor_color"]).astype(np.bool_)

        self.pixel_grid.arrays["swamps"] = self.get_squares_from_raw_array(swamp_array, self.pixel_grid.offsets - self.tile_resolution // 2, self.tile_resolution, margin=3, detection_proportion=0.3)

    def detect_holes(self):
        #tile_size = self.tile_size * self.pixel_per_m
        #offsets = self.__get_offsets(tile_size)

        self.pixel_grid.arrays["hole_detections"] += self.hole_color_filter.filter(self.pixel_grid.arrays["floor_color"]).astype(np.bool_)
        #cv.imshow("holes", self.pixel_grid.arrays["hole_detections"].astype(np.uint8) * 255)
        self.pixel_grid.arrays["holes"] = self.get_squares_from_raw_array(self.pixel_grid.arrays["hole_detections"], self.pixel_grid.offsets - self.tile_resolution // 2, self.tile_resolution, detection_proportion=0.2)

    def detect_checkpoints(self):
        checkpoint_array = self.checkpoint_color_filter.filter(self.pixel_grid.arrays["floor_color"]).astype(np.bool_)

        self.pixel_grid.arrays["checkpoints"] = self.get_tile_centers_from_raw_array(checkpoint_array, self.pixel_grid.offsets - self.tile_resolution // 2, self.tile_resolution)


    def get_squares_from_raw_array(self, hole_array: np.ndarray, raw_offsets: np.ndarray, square_size, margin=0, detection_proportion=0.5) -> np.ndarray:

        offsets = np.round(raw_offsets % square_size).astype(int)

        final_hole_array = np.zeros_like(hole_array)

        for x in range(offsets[0], hole_array.shape[0] - square_size, square_size):
            for y in range(offsets[1], hole_array.shape[1] - square_size, square_size):
                min_x = x
                min_y = y
                max_x = x + square_size
                max_y = y + square_size

                square = hole_array[min_x:max_x, min_y:max_y]

                count = np.count_nonzero(square)

                if count / (np.max(square.shape) ** 2) > detection_proportion:
                    final_hole_array[min_x - margin:max_x + margin, min_y -margin:max_y+margin] = True

        
        return final_hole_array
    
    def get_tile_centers_from_raw_array(self, raw_array: np.ndarray, raw_offsets: np.ndarray, square_size) -> np.ndarray:

        offsets = np.round(raw_offsets % square_size).astype(int)

        final_array = np.zeros_like(raw_array)

        for x in range(offsets[0], raw_array.shape[0] - square_size, square_size):
            for y in range(offsets[1], raw_array.shape[1] - square_size, square_size):
                min_x = x
                min_y = y
                max_x = x + square_size
                max_y = y + square_size

                square = raw_array[min_x:max_x, min_y:max_y]

                count = np.count_nonzero(square)

                if count / (np.max(square.shape) ** 2) > 0.3:
                    final_array[min_x + square_size // 2, min_y + square_size // 2] = True

        
        return final_array


    def load_average_tile_color(self):
        tile_size = self.tile_size * self.pixel_per_m
        offsets = self.__get_offsets(tile_size)
        floor_color = deepcopy(self.pixel_grid.arrays["floor_color"])

        kernel = self.get_color_average_kernel()

        floor_color = cv.filter2D(floor_color, -1, kernel)
        #print("offsets", offsets)
        image = []

        for x in range(round(offsets[0] + tile_size / 2), floor_color.shape[0], round(tile_size)):
            row = []
            for y in range(round(offsets[1] + tile_size / 2), floor_color.shape[1], round(tile_size)):
                row.append(floor_color[x, y, :])
            image.append(row)

        image = np.array(image, dtype=np.uint8)

        
        image = cv.resize(image, (0, 0), fx=tile_size, fy=tile_size, interpolation=cv.INTER_NEAREST)

                    
        final_x = image.shape[0] if image.shape[0] + offsets[0] < self.pixel_grid.array_shape[0] else self.pixel_grid.array_shape[0] - offsets[0]
        final_y = image.shape[1] if image.shape[1] + offsets[1] < self.pixel_grid.array_shape[1] else self.pixel_grid.array_shape[1] - offsets[1]

        #self.pixel_grid.arrays["average_floor_color"] = np.zeros((final_x, final_y, 3), dtype=np.uint8)

        self.pixel_grid.arrays["average_floor_color"][offsets[0]:offsets[0] + final_x:, offsets[1]:offsets[1] + final_y, :] = image[:final_x,:final_y, :]
        