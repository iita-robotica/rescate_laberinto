import numpy as np
import cv2 as cv

import utilities
from data_processing import camera_processor
from data_processing import point_cloud_processor

class FloorColorExtractor:
    def __init__(self, tile_resolution) -> None:
        self.tile_resolution = tile_resolution
        self.floor_color_ranges = {
                    "normal":   
                        {   
                            "range":   ((0, 0, 100), (1, 1, 255)), 
                            "threshold":0.1},

                    "nothing":
                        {
                            "range":((100, 0, 0), (101, 1, 1)),
                            "threshold":0.9},
                    
                    "checkpoint":
                        {
                            "range":((100, 0, 70), (150, 100, 100)),
                            "threshold":0.1},
                    }
        
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

        final_image = floor_image.copy()
        utilities.save_image(final_image, "floor_image.png")

        squares_grid = utilities.get_squares(final_image, self.tile_resolution, offsets)

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

                cv.rectangle(final_image, [square[2], square[0]], [square[3], square[1]], color, -1)

                tile = [square[2], square[0]]
                tile = utilities.substractLists(tile, (350 - offsets[0], 350 - offsets[1]))
                tile = utilities.divideLists(tile, [self.tile_resolution, self.tile_resolution])
                tile = [int(t) for t in tile]
                if color_key != "nothing":
                    print(tile, color_key)
                    color_tiles.append((tile, color_key))

        utilities.draw_grid(final_image, self.tile_resolution, offset=grid_offsets)
        cv.circle(final_image, (350 - offsets[0], 350 - offsets[1]), 10, (255, 0, 0), -1)
        cv.imshow("final_floor_image", final_image)        
        return color_tiles


        
        

class PointCloudExtarctor:
    def __init__(self, resolution, threshold):
        self.threshold = threshold
        self.resolution = resolution
        self.straight_template = np.zeros((self.resolution, self.resolution), dtype=np.int)
        self.straight_template[:][0:2] = 1

        self.templates = {}

        for i, name in enumerate(["u", "l", "d", "r"]):
            self.templates[name] = np.rot90(self.straight_template, i)

    def get_tile_status(self, min_x, min_y, max_x, max_y, point_cloud):
        counts = {}
        for name in self.templates:
            counts[name] = 0
        square = point_cloud[min_x:max_x, min_y:max_y]
        if square.shape != (self.resolution, self.resolution):
            return []
        for name, template in self.templates.items():
            for i in range(self.resolution):
                for j in range(self.resolution):
                    if square[i][j] == 1:
                        counts[name] += template[i][j]

        for name, count in counts.items():
            if count >= self.threshold:
                yield name

    def transform_to_grid(self, point_cloud):
        offsets = point_cloud.offsets
        offsets = [o % self.resolution for o in offsets]
        grid = []
        bool_array_copy = point_cloud.get_bool_array()
        bool_array_copy = bool_array_copy.astype(np.uint8) * 100
        for x in range(offsets[0], bool_array_copy.shape[0] - self.resolution, self.resolution):
            row = []
            for y in range(offsets[1], bool_array_copy.shape[1] - self.resolution, self.resolution):
                min_x = x
                min_y = y
                max_x = x + self.resolution
                max_y = y + self.resolution
                #print(min_x, min_y, max_x, max_y)
                bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)
                
                val = self.get_tile_status(min_x, min_y, max_x, max_y, point_cloud.get_bool_array())
                row.append(list(val))
            grid.append(row)
        factor = 10
        cv.imshow("bool_array_copy", cv.resize(bool_array_copy.astype(np.uint8), (point_cloud.shape[1] * factor, point_cloud.shape[0] * factor), interpolation=cv.INTER_NEAREST))
        return grid, [o // self.resolution for o in offsets]