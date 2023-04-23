import numpy as np
import cv2 as cv
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.angle import Angle
import imutils
from copy import copy, deepcopy

class FloorMapper:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid, tile_resolution, tile_size, camera_distance_from_center) -> None:
        self.pixel_grid = pixel_grid
        self.tile_resolution = tile_resolution
        self.tile_size = tile_size
        self.pixel_per_m = tile_resolution / tile_size
        self.pov_distance_from_center = round(0.064 * self.pixel_per_m) 

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
        cv.imshow('pov', pov)
        max_dim = max(pov.shape)
        if background  is None: background = np.zeros((max_dim * 2, max_dim * 2, 4), dtype=np.uint8)

        start = (max_dim, max_dim - round(pov.shape[1] / 2))
        end =  (start[0] + pov.shape[0], start[1] + pov.shape[1])
        
        background[start[0]:end[0], start[1]:end[1], :] = pov[:,:,:]

        cv.imshow("pov in background", background)

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
    

    def get_unified_povs(self, camera_images):
        povs_list = []
        for camera_image in camera_images:
            pov = self.flatten_camera_pov(np.rot90(copy(camera_image.image), k=3))
            pov = np.flip(pov, 1)
            pov = self.set_in_background(pov)
            pov = self.rotate_image_to_angle(pov, camera_image.orientation)
            povs_list.append(pov)

        return sum(povs_list)
    
    def map_floor(self, camera_images, robot_grid_index):
        povs = self.get_unified_povs(camera_images)

        cv.imshow("final_pov", povs[:, :, 3])

        self.load_povs_to_grid(robot_grid_index, povs)

    def load_povs_to_grid(self, robot_grid_index, povs):
        
        start = np.array((robot_grid_index[0] - (povs.shape[0] // 2), robot_grid_index[1] - (povs.shape[1] // 2)))
        end = np.array((robot_grid_index[0] + (povs.shape[0] // 2), robot_grid_index[1] + (povs.shape[1] // 2)))

        self.pixel_grid.expand_to_grid_index(start)
        self.pixel_grid.expand_to_grid_index(end)

        start = self.pixel_grid.grid_index_to_array_index(start)
        end = self.pixel_grid.grid_index_to_array_index(end)

        mask = povs[:,:,3] > 254

        gradient = self.__get_distance_to_center_gradient(povs.shape[:2])

        povs_gradient = np.zeros_like(gradient)
        povs_gradient[mask] = gradient[mask]

        cv.imshow("gradient", povs_gradient)

        

        detection_distance_mask = self.pixel_grid.arrays["floor_color_detection_distance"][start[0]:end[0], start[1]:end[1]] < povs_gradient

        seen_by_camera_mask = self.pixel_grid.arrays["seen_by_camera"][start[0]:end[0], start[1]:end[1]]

        final_mask = seen_by_camera_mask * detection_distance_mask

        self.pixel_grid.arrays["floor_color_detection_distance"][start[0]:end[0], start[1]:end[1]][final_mask] = povs_gradient[final_mask]


        self.pixel_grid.arrays["floor_color"][start[0]:end[0], start[1]:end[1]][final_mask] = povs[:,:,:3][final_mask]

        blured = self.blur_floor_color(robot_grid_index)
        cv.imshow("blured", blured)

    
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
        tile_size = self.tile_size * self.pixel_per_m
        square_propotion = 0.8
        square_size = round(tile_size * square_propotion)

        kernel = np.ones((square_size, square_size), dtype=np.float32)

        kernel = kernel / kernel.sum()

        return kernel

    
    def blur_floor_color(self, robot_position):
        tile_size = self.tile_size * self.pixel_per_m
        offsets = self.__get_offsets(tile_size)
        floor_color = deepcopy(self.pixel_grid.arrays["floor_color"])

        kernel = self.get_color_average_kernel()

        floor_color = cv.filter2D(floor_color, -1, kernel)
        print("offsets", offsets)

        image = []

        for x in range(offsets[0], floor_color.shape[0], round(tile_size)):
            row = []
            for y in range(offsets[1], floor_color.shape[1], round(tile_size)):
                row.append(floor_color[x, y, :])
            image.append(row)

        image = np.array(image, dtype=np.uint8)
            
        cv.imshow("floor_color", image)
            
            
        return floor_color