from copy import copy, deepcopy

import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D
from data_structures.angle import Angle

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.tile_color_grid import TileColorExpandableGrid

from mapping.camera_processor import CameraProcessor
from mapping.wall_mapper import WallMapper
from mapping.floor_mapper import FloorMapper

from mapping.data_extractor import PointCloudExtarctor, FloorColorExtractor

from flags import SHOW_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID, DO_WAIT_KEY


class Mapper:
    def __init__(self, tile_size, robot_diameter, camera_distance_from_center):
        self.tile_size = tile_size
        self.robot_diameter = robot_diameter

        self.robot_position = None
        self.robot_orientation = None
        self.start_position = None

        # Data structures
        pixels_per_tile = 10
        self.pixel_grid = CompoundExpandablePixelGrid(initial_shape=np.array([1, 1]), 
                                                      pixel_per_m=pixels_per_tile / 0.06, 
                                                      robot_radius_m=(self.robot_diameter / 2) -0.008)
        
        self.tile_color_grid = TileColorExpandableGrid(initial_shape=np.array((1, 1)),
                                                       tile_size=0.12)

        #Data processors
        self.wall_mapper = WallMapper(self.pixel_grid, robot_diameter)
        self.floor_mapper = FloorMapper(pixel_grid=self.pixel_grid, 
                                        tile_resolution=pixels_per_tile * 2,
                                        tile_size=0.12,
                                        camera_distance_from_center=camera_distance_from_center)
        
        self.camera_processor = CameraProcessor(tile_resolution=100)

        # Data extractors
        self.point_cloud_extractor = PointCloudExtarctor(resolution=6)
        self.floor_color_extractor = FloorColorExtractor(tile_resolution=50)

    def update(self, in_bounds_point_cloud: list = None, 
               out_of_bounds_point_cloud: list = None, 
               camera_images: list = None, 
               robot_position: Position2D = None, 
               robot_orientation: Angle = None):
        
        if robot_position is None or robot_orientation is None:
            return
        
        self.robot_position = robot_position
        self.robot_orientation = robot_orientation

        # Load walls and obstacles (Lidar detections)
        if in_bounds_point_cloud is not None and out_of_bounds_point_cloud is not None:
            self.wall_mapper.load_point_cloud(in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position)
        
        self.pixel_grid.load_robot(np.array(self.robot_position), self.robot_orientation)
        
        # Load floor colors
        if camera_images is not None:
            self.floor_mapper.map_floor(camera_images, self.pixel_grid.coordinates_to_grid_index(self.robot_position))
        
        #DEBUG
        if DO_WAIT_KEY:
            cv.waitKey(1)

    
        
    def register_start(self, robot_position):
        self.start_position = deepcopy(robot_position)
        print("registered start position:", self.start_position)

    
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