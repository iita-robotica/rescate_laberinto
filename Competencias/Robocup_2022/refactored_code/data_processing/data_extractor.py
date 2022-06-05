import numpy as np
import cv2 as cv

import utilities
from data_processing import camera_processor
from data_processing import point_cloud_processor

class FloorColorExtractor:
    def __init__(self) -> None:
        self.floor_color_ranges = {
                    "normal":   
                        {   
                            "range":   ((0, 0, 0), (255, 255, 255)), 
                            "threshold":0.5},

                    "swamp":
                        {
                            "range":((0, 0, 0), (255, 255, 255)),
                            "threshold":0.5},
                    }
        
    def get_square_color(self, square):
        color_counts = {}
        for row in square:
            for color in row:
                for key, value in self.floor_color_ranges.items():
                    if utilities.is_color_in_range(color, value["range"]):
                        if key in color_counts:
                            color_counts[key] += 1
                        else:
                            color_counts[key] = 1
        
        for key, value in color_counts.items():
            if value < self.floor_color_ranges[key]["threshold"] * len(square) ** 2:
                del color_counts[key]
        
        if len(color_counts) == 0:
            return None
        else:
            return max(color_counts, key=color_counts.get)



    def get_floor_colors(self, camera_images:dict, lidar_point_cloud, robot_rotation:float, robot_position:list):
        # Flatten, correct and join camera images
        camera_final_image = camera_processor.get_floor_image(camera_images, robot_rotation)

        #offsets = (int((robot.position[0] * 850) % 50), int((robot.position[1] * 850) % 50))
        #utilities.draw_grid(camera_final_image, 50, offsets)
        #cv.imshow("camera_final_image", camera_final_image)

        # Get point clouds
        point_cloud, out_of_bounds_point_cloud = lidar_point_cloud

        # Scale point clouds to camera image resolution
        camera_out_of_bounds_point_cloud = point_cloud_processor.processPointCloudForCamera(out_of_bounds_point_cloud, robotPos=robot_position)
        camera_point_cloud = point_cloud_processor.processPointCloudForCamera(point_cloud, robotPos=robot_position)

        # Join point clouds and delete duplicates
        total_camera_point_cloud = np.vstack((camera_point_cloud, camera_out_of_bounds_point_cloud))
        #total_camera_point_cloud = np.unique(total_camera_point_cloud)

        # Get lines from robot to point cloud
        seen_points = point_cloud_processor.get_intermediate_points(total_camera_point_cloud, (350, 350))
        
        # Draw the floor image where the floor is visible
        #final_image = np.zeros(camera_final_image.shape, np.uint8)
        final_image = np.zeros((700, 700, 3), np.uint8)
        utilities.draw_poses(final_image, seen_points, back_image=camera_final_image)

        # Draw the grid on the final image
        offsets = (int((robot_position[0] * 850) % 50), int((robot_position[1] * 850) % 50))

        utilities.draw_squares_where_not_zero(final_image, 50, offsets)
        utilities.draw_poses(final_image, camera_point_cloud, 255)
        utilities.draw_grid(final_image, 50, offsets)
        
        print("FINAL IMG SHAPE", final_image.shape)
        
        cv.imshow("final_image", final_image.astype(np.uint8))
        cv.waitKey(1)

        """
        squares = utilities.get_squares_where_not_zero(final_image, 50, offsets)
        for square in squares:
            print(get_square_color(square))

        return squares
        """

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