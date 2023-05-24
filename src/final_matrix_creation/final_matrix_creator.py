from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D
import copy
import math
import skimage

import numpy as np
import cv2 as cv


class FinalMatrixCreator:
    def __init__(self, tile_size: float, resolution: float):
        self.threshold = 8
        self.__square_size_px = round(tile_size / 2 * resolution)

        straight = [
            [0, 0, 1, 2, 2, 2, 2, 1, 0, 0],
            [0, 0, 1, 2, 2, 2, 2, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.straight_template = np.array(straight)

        
        vortex = [
            [3, 3, 3, 0, 0, 0, 0, 0, 0, 0],
            [3, 3, 3, 0, 0, 0, 0, 0, 0, 0],
            [3, 3, 3, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.vortex_template = np.array(vortex)
        


        self.templates = {}

        for i, name in enumerate([(-1, 0), (0,-1), (1,0), (0,1)]):
            self.templates[name] = np.rot90(self.straight_template, i)
        
        for i, name in enumerate([(-1,-1), (1, -1), (1, 1), (-1, 1)]):
           self.templates[name] = np.rot90(self.vortex_template, i)

    def __get_tile_status(self, min_x, min_y, max_x, max_y, wall_array: np.ndarray) -> list:
        counts = {name: 0 for name in self.templates}
        square = wall_array[min_x:max_x, min_y:max_y]
        if square.shape != (self.__square_size_px, self.__square_size_px):
            return []

        non_zero_indices = np.where(square != 0)
        for orientation, template in self.templates.items():
            counts[orientation] = np.sum(template[non_zero_indices])

        status = []
        for orientation, count in counts.items():
            if count >= self.threshold:
                status.append(orientation)

        return status

    def __transform_wall_array_to_bool_node_array(self, wall_array: np.ndarray, offsets: np.ndarray) -> np.ndarray:
        

        grid = []
        bool_array_copy = wall_array.astype(np.uint8) * 100
        for x in range(offsets[0], wall_array.shape[0] - self.__square_size_px, self.__square_size_px):
            row = []
            for y in range(offsets[1], wall_array.shape[1] - self.__square_size_px, self.__square_size_px):
                min_x = x
                min_y = y
                max_x = x + self.__square_size_px
                max_y = y + self.__square_size_px
                #print(min_x, min_y, max_x, max_y)
                
                bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)
                
                val = self.__get_tile_status(min_x, min_y, max_x, max_y, wall_array)
                
                row.append(list(val))
            grid.append(row)
        
        cv.imshow("point_cloud_with_squares", cv.resize(bool_array_copy, (0, 0), fx=1, fy=1, interpolation=cv.INTER_AREA))

        grid = self.__orientation_grid_to_final_wall_grid(grid)

        return grid
    
    def __orientation_grid_to_final_wall_grid(self, orientation_grid: list) -> np.ndarray:
        shape = np.array([len(orientation_grid), len(orientation_grid[0])])
        shape *= 2

        final_wall_grid = np.zeros(shape, dtype=np.bool_)
        
        for y, row in enumerate(orientation_grid):
            for x, value in enumerate(row):
                x1 = x * 2
                y1 = y * 2

                for orientation in value:
                    final_x = x1 + orientation[1]
                    final_y = y1 + orientation[0]

                    final_wall_grid[final_y, final_x] = True
        
        return final_wall_grid


    def pixel_grid_to_final_grid(self, pixel_grid: CompoundExpandablePixelGrid, robot_start_position: np.ndarray) -> np.ndarray:
        
        wall_array = pixel_grid.arrays["walls"]
        color_array = pixel_grid.arrays["floor_color"]

        offsets = self.__get_offsets(self.__square_size_px, pixel_grid.offsets)
        
        wall_node_array = self.__transform_wall_array_to_bool_node_array(wall_array, offsets)

        if robot_start_position is None:
            return np.array([])
        
        start_array_index = pixel_grid.coordinates_to_array_index(robot_start_position)

        start_array_index -= offsets

        robot_node = np.round((start_array_index / self.__square_size_px) * 2).astype(int) - 1

        text_grid = self.__get_final_text_grid(wall_node_array, robot_node)

        return np.array(text_grid)

        #wall_array = self.offset_array(wall_array, self.square_size_px, pixel_grid.offsets)
        #color_array = self.offset_array(color_array, self.square_size_px, pixel_grid.offsets)

    def __get_final_text_grid(self, wall_node_array: np.ndarray, robot_node: np.ndarray) -> list:
        
        #wall_node_array[robot_node[0], robot_node[1]] = True

        cv.imshow("final_grid", cv.resize(wall_node_array.astype(np.uint8) * 255, (0, 0), fx=10, fy=10, interpolation=cv.INTER_AREA))


        
        final_text_grid = []

        for row in wall_node_array:
            f_row = []
            for val in row:
                if val:
                    f_row.append("1")
                else:
                    f_row.append("0")
            final_text_grid.append(f_row)
        
        self.__set_node_as_character(final_text_grid, robot_node, "5")

        return final_text_grid
        
    
    def __get_offsets(self, square_size: float, raw_offsets: np.array) -> np.ndarray:
        return np.round(raw_offsets % square_size).astype(int)
    

    def __set_node_as_character(self, final_text_grid: list, node: np.ndarray, character: str) -> list:
        for diagonal in np.array(((1, 1), (-1, 1), (-1, -1), (1, -1))):
            n = node + diagonal
            final_text_grid[n[0]][n[1]] = character

        return final_text_grid



"""
class FinalMatrixCreator:
    def __init__(self, tile_size, resolution) -> None:
        self.square_size_px = round(tile_size / 2 * resolution)

        straight = [
            [0, 0,-1,-4,-8,-8,-4,-1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 2, 2, 4, 4, 2, 2, 1, 0],
            [0, 1, 2, 2, 4, 4, 2, 2, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 2, 2, 4, 4, 2, 2, 1, 0],
            [0, 1, 2, 2, 4, 4, 2, 2, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0,-1,-4,-8,-8,-4,-1, 0, 0],
                ]
        
        self.horizontal_template = np.array(straight, dtype=np.int8)

        self.template_match_threshold = 0.4

    def pixel_grid_to_final_grid(self, pixel_grid: CompoundExpandablePixelGrid) -> np.ndarray:
        
        
        wall_array = pixel_grid.arrays["walls"]
        color_array = pixel_grid.arrays["floor_color"]
        wall_array = self.offset_array(wall_array, self.square_size_px, pixel_grid.offsets)
        color_array = self.offset_array(color_array, self.square_size_px, pixel_grid.offsets)
        
        shape = np.ceil(np.array(wall_array.shape) / self.square_size_px * 2).astype(int)

        final_grid = np.zeros(shape, np.bool_)

        
        #debug_array = wall_array.astype(np.uint8) * 255
        #for x in range(0, shape[1] -1, 2):
        #    for y in range(1, shape[0] -1, 2):
        #       final_grid[y][x] = self.analyze_wall_square(x // 2, y // 2 + 0.5, wall_array, final_grid, debug_array)
        

        final_grid = cv.filter2D(wall_array.astype(np.float32), -1, self.horizontal_template)
        
        bool_final = (final_grid * (1 / np.sum(self.horizontal_template[self.horizontal_template > 0])) > self.template_match_threshold).astype(np.uint8) * 255

        cv.imshow("final_grid", bool_final[::self.square_size_px, ::self.square_size_px])#cv.resize(final_grid.astype(np.uint8) * 255, (0, 0), fx=10, fy=10, interpolation=cv.INTER_AREA))

        cv.imshow("template", self.horizontal_template.astype(np.float32)  * (1 / np.max(self.horizontal_template)))
        
        #cv.imshow("wall_array_tile_debug", debug_array)

    
    def analyze_wall_square(self, x: int, y: int, wall_array: np.ndarray, final_grid: list, debug_array = None):
        min_x = round(x * self.square_size_px)
        min_y = round(y * self.square_size_px)
        max_x = round(min_x + self.square_size_px)
        max_y = round(min_y + self.square_size_px)
        #print(min_x, min_y, max_x, max_y)

        cv.rectangle(debug_array, (min_x, min_y), (max_x, max_y), (255,), 1)
        
        print("x:", min_x, max_x, "y:", min_y, max_y)
        square_array = wall_array[min_y:max_y, min_x:max_x]
        
        val = self.get_tile_value(square_array)

        if np.any(square_array) and val:
            cv.imshow("example wall", square_array.astype(np.uint8) * 255)

        return val


    def get_tile_value(self, square_array: np.ndarray) -> bool:
        try:    
            count = np.sum(self.horizontal_template[square_array])
        except IndexError:
            return False
        
        ratio = count / np.sum(self.horizontal_template)

        return ratio > self.template_match_threshold
    
    def offset_array(self, array: np.ndarray, square_size: float, raw_offsets: Position2D) -> np.ndarray:
        offsets = self.get_offsets(square_size, raw_offsets)
        return array[offsets[0]:, offsets[1]:]
    
    def get_offsets(self, square_size: float, raw_offsets: np.array) -> np.ndarray:
        return np.round(raw_offsets % square_size).astype(int)
    

    def generate_horizontal_wall_template(self, square_size_px: np.ndarray) -> np.ndarray:
        array = np.zeros([square_size_px]*2, np.uint8)
        line_width = round(self.horizontal_line_width_ratio * square_size_px)
        vertical_margin = round((square_size_px - line_width) / 2)

        line_lenght = round(self.horizontal_line_lenght_ratio * square_size_px)
        horizontal_margin = round((square_size_px - line_lenght) / 2)

        array[vertical_margin:-vertical_margin, horizontal_margin:-horizontal_margin] = 1

        horizontal_gap_size = self.horizontal_line_middle_gap_ratio * square_size_px // 2
        middle = array.shape[0] // 2
        array[int(middle - horizontal_gap_size): int(middle + horizontal_gap_size), 0:-1] = 0

        return array

"""

