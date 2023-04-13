import numpy as np
import cv2 as cv
import copy

import utilities

from flags import SHOW_DEBUG

class FloorColorExtractor:
    def __init__(self, tile_resolution) -> None:
        self.tile_resolution = tile_resolution
        self.floor_color_ranges = {
                    "normal":
                        {   
                            "range":   ((0, 0, 37), (0, 0, 192)), 
                            "threshold":0.2},

                    "nothing":
                        {
                            "range":((100, 0, 0), (101, 1, 1)),
                            "threshold":0.9},
                    
                    "checkpoint":
                        {
                            "range":((95, 0, 65), (128, 122, 198)),
                            "threshold":0.2},
                    "hole":
                        {
                            "range":((0, 0, 10), (0, 0, 30)),
                            "threshold":0.2},
                    
                    "swamp":
                        {
                            "range":((19, 112, 32), (19, 141, 166)),
                            "threshold":0.2},

                    "connection1-2":
                        {
                            "range":((120, 182, 49), (120, 204, 232)),
                            "threshold":0.2},

                    "connection1-3":
                        {
                            "range":((132, 156, 36), (133, 192, 185)),
                            "threshold":0.2},

                    "connection2-3":
                        {
                            "range":((0, 182, 49), (0, 204, 232)),
                            "threshold":0.2},
                    }
        self.final_image = np.zeros((700, 700, 3), np.uint8)
        
    def get_square_color(self, image, square_points):
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        square = cv.cvtColor(square, cv.COLOR_BGR2HSV)
        if np.count_nonzero(square) == 0:
            return "nothing"
        color_counts = {}
        for color_key, color_range in self.floor_color_ranges.items():
            colour_count = np.count_nonzero(cv.inRange(square, color_range["range"][0], color_range["range"][1]))
            if colour_count > color_range["threshold"] * square.shape[0] * square.shape[1]:
                color_counts[color_key] = colour_count
        
        if len(color_counts) == 0:
            return "nothing"
        else:
            return max(color_counts, key=color_counts.get)
    
    def get_sq_color(self, image, square_points):
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        # remove pixels with value 0, 0, 0
        white_count = np.count_nonzero(cv.inRange(square, (180, 180, 180), (255, 255, 255)))
        black_count = np.count_nonzero(cv.inRange(square, (20, 20, 20), (180, 180, 180)))

        if white_count > black_count and white_count > square.shape[0] * square.shape[1] / 8:
            return (255, 255, 255)
        else:
            return (100, 100, 100)

    def get_floor_colors(self, floor_image, robot_position):

        grid_offsets = [(((p + 0) % 0.06) / 0.06) * 50 for p in robot_position]
        
        grid_offsets = [int(o) for o in grid_offsets]

        offsets = [((((p + 0.03) % 0.06) - 0.03) / 0.06) * 50 for p in robot_position]
        
        offsets = [int(o) for o in offsets]

        
        utilities.save_image(floor_image, "floor_image.png")

        squares_grid = utilities.get_squares(floor_image, self.tile_resolution, offsets)

        color_tiles = []
        for row in squares_grid:
            for square in row:
                color_key = self.get_square_color(floor_image, square)
                if color_key == "normal":
                    color = (255, 255, 255)
                elif color_key == "checkpoint":
                    color = (100, 100, 100)
                else:
                    color = (0, 0, 0)
                #if color != (0, 0, 0):
                #cv.rectangle(self.final_image, [square[2], square[0]], [square[3], square[1]], color, -1)

                tile = [square[2], square[0]]
                tile = utilities.substractLists(tile, (350 - offsets[0], 350 - offsets[1]))
                tile = utilities.divideLists(tile, [self.tile_resolution, self.tile_resolution])
                tile = [int(t) for t in tile]
                if color_key != "nothing":
                    if SHOW_DEBUG:
                        print(tile, color_key)
                    color_tiles.append((tile, color_key))

        if SHOW_DEBUG:
            drawing_image = floor_image.copy() #self.final_image.copy()
            utilities.draw_grid(drawing_image, self.tile_resolution, offset=grid_offsets)
            cv.circle(drawing_image, (350 - offsets[0], 350 - offsets[1]), 10, (255, 0, 0), -1)
            cv.imshow("final_floor_image", utilities.resize_image_to_fixed_size(drawing_image, (600, 600)))        
        return color_tiles


        
        

class PointCloudExtarctor:
    def __init__(self, resolution):
        self.threshold = 8
        self.resolution = resolution
        self.straight_template = np.zeros((self.resolution + 1, self.resolution + 1), dtype=int)
        self.straight_template[:][0:2] = 1
        #self.straight_template[3:-3][0:2] = 2
        self.straight_template[0][0:2] = 0
        self.straight_template[-1][0:2] = 0

        straight = [
            [0, 1, 2, 2, 2, 1, 0],
            [0, 1, 2, 2, 2, 1, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.straight_template = np.array(straight)

        curved = [
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 1, 1, 0],
            [0, 0, 3, 1, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [1, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.curved_template = np.array(curved)


        self.templates = {}

        for i, name in enumerate([("u",), ("l",), ("d",), ("r",)]):
            self.templates[name] = np.rot90(self.straight_template, i)
        
        for i, name in enumerate([("u", "l"), ("d", "l"), ("d", "r"),  ("u", "r")]):
            self.templates[name] = np.rot90(self.curved_template, i)

    def get_tile_status(self, min_x, min_y, max_x, max_y, point_cloud):
        counts = {name: 0 for name in self.templates}
        square = point_cloud[min_x:max_x+1, min_y:max_y+1]
        if square.shape != (self.resolution+1, self.resolution+1):
            return []

        non_zero_indices = np.where(square != 0)
        for name, template in self.templates.items():
            counts[name] = np.sum(template[non_zero_indices])

        names = [name for name, count in counts.items() if count >= self.threshold]

        return [i for sub in names for i in sub]

    def transform_to_grid(self, point_cloud):
        offsets = point_cloud.offsets
        offsets = [o % self.resolution for o in offsets]
        offsets.reverse()
        grid = []
        bool_array_copy = point_cloud.get_bool_array()
        if SHOW_DEBUG:
            bool_array_copy = bool_array_copy.astype(np.uint8) * 100
        for x in range(offsets[0], bool_array_copy.shape[0] - self.resolution, self.resolution):
            row = []
            for y in range(offsets[1], bool_array_copy.shape[1] - self.resolution, self.resolution):
                min_x = x
                min_y = y
                max_x = x + self.resolution
                max_y = y + self.resolution
                #print(min_x, min_y, max_x, max_y)

                if SHOW_DEBUG:
                    bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)
                
                val = self.get_tile_status(min_x, min_y, max_x, max_y, point_cloud.get_bool_array())
                
                row.append(list(val))
            grid.append(row)
        factor = 10

        if SHOW_DEBUG:
            cv.imshow("point_cloud_with_squares", utilities.resize_image_to_fixed_size(bool_array_copy, (600, 600)))
        offsets = point_cloud.offsets
        return grid, [o // self.resolution for o in offsets]