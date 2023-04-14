import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D
from data_structures.angle import Angle

from data_structures import compound_pixel_grid

from mapping.camera_processor import CameraProcessor
from mapping.point_cloud_processor import PointCloudProcessor

from mapping.data_extractor import PointCloudExtarctor, FloorColorExtractor

from flags import SHOW_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID


class Mapper:
    def __init__(self, tile_size):
        self.ADJACENTS_NO_DIAGONALS =  (Position2D(1, 1), Position2D(1, -1), Position2D(-1, 1), Position2D(-1, -1))

        self.tile_size = tile_size

        self.robot_position = None
        self.robot_rotation = None

        # Data structures
        pixels_per_tile = 10
        self.pixel_grid = compound_pixel_grid.Grid(np.array([1, 1]), pixels_per_tile / 0.06, (0.074 / 2) - 0.005)#+ 0.008)

        #Data processors
        self.point_cloud_processor = PointCloudProcessor(center_point=350, map_scale=50 / tile_size)
        self.camera_processor = CameraProcessor(tile_resolution=100)

        # Data extractors
        self.point_cloud_extractor = PointCloudExtarctor(resolution=6)
        self.floor_color_extractor = FloorColorExtractor(tile_resolution=50)

    def update(self, in_bounds_point_cloud: list = None, 
               out_of_bounds_point_cloud: list = None, 
               camera_images: list = None, 
               robot_position: Position2D = None, 
               robot_rotation: Angle = None):
        
        if robot_position is None or robot_rotation is None:
            return
        
        self.robot_position = robot_position
        self.robot_rotation = robot_rotation

        self.pixel_grid.load_robot(np.array(self.robot_position), self.robot_rotation)
        
        # Load walls and obstacles (Lidar detections)
        if in_bounds_point_cloud is not None:
            self.pixel_grid.load_point_cloud(in_bounds_point_cloud, out_of_bounds_point_cloud, np.array(robot_position))
            
        # Load floor colors
        if in_bounds_point_cloud is not None and camera_images is not None:
            pass
        
        #DEBUG
        
        if SHOW_GRANULAR_NAVIGATION_GRID or SHOW_DEBUG:
            cv.waitKey(1) 
        
    def register_start(self, robot_position):
        pass # TODO

    def set_robot_node(self, robot_position):
        pass # TODO

    def block_front_vortex(self, robot_rotation):
        pass # TODO

    # Fixtures
    def load_fixture_to_wall(self, letter, image_angle):
        pass # TODO

    def load_fixture_to_tile(self, letter, camera_angle, robot_rotation):
        pass # TODO
    
    def get_fixture_from_tile(self):
        pass # TODO
    
    # Grids
    def get_grid_for_bonus(self):
        """
        final_grid = []
        for row in self.get_node_grid().grid:
            final_row = []
            for node in row:
                final_row.append(node.get_representation())
            final_grid.append(final_row)
        return np.array(final_grid)
        """
        pass # TODO
    

    def __lidar_to_node_grid(self):
        """
        grid, offsets = self.point_cloud_extractor.transform_to_grid(self.lidar_grid)
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                xx = (x - offsets[0]) * 2 + 1
                yy = (y - offsets[1]) * 2 + 1
                #print(value)
                for direction in value:
                    self.node_grid.load_straight_wall((xx, yy),  direction)
        """

    def __process_floor(self, camera_images, total_point_cloud, robot_position, robot_rotation):
        """
        floor_image = self.camera_processor.get_floor_image(camera_images, robot_rotation.degrees)
        final_image = np.zeros(floor_image.shape, dtype=np.uint8)

        #cv.imshow("floor_image", floor_image)

        self.point_cloud_processor.center_point = (floor_image.shape[1] // 2, floor_image.shape[0] // 2)

        camera_point_cloud = self.point_cloud_processor.processPointCloudForCamera(total_point_cloud)
        seen_points = self.point_cloud_processor.get_intermediate_points(camera_point_cloud)
        #print(seen_points)

        utilities.draw_poses(final_image, seen_points, back_image=floor_image, xx_yy_format=True)
        
        floor_colors = self.floor_color_extractor.get_floor_colors(final_image, robot_position)

        #robot_tile = utilities.divideLists(robot_position, [self.tile_size, self.tile_size])
        robot_tile = [round((x + 0.03) / self.tile_size - 0.5) for x in robot_position]
        robot_node = [t * 2 for t in robot_tile]
       
        for floor_color in floor_colors:
            tile = floor_color[0]
            color = floor_color[1]
            tile = utilities.multiplyLists(tile, [2, 2])
            #tile.reverse()
            tile = utilities.sumLists(tile, [1, 1])
            tile = utilities.sumLists(tile, robot_node)
            if SHOW_DEBUG:
                print(self.node_grid.get_node(tile).node_type)
            if self.node_grid.get_node(tile).tile_type != "start":
                self.node_grid.get_node(tile).tile_type = color   
        #cv.imshow('final_image', utilities.resize_image_to_fixed_size(final_image, (600, 600)))    

        """