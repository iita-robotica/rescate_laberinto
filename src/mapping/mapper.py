from copy import copy, deepcopy

import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D
from data_structures.angle import Angle

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.tile_color_grid import TileColorExpandableGrid

from mapping.wall_mapper import WallMapper
from mapping.floor_mapper import FloorMapper

from mapping.array_filtering import ArrayFilterer

from mapping.robot_mapper import RobotMapper
from mapping.fixture_mapper import FixtureMapper

from mapping.data_extractor import PointCloudExtarctor, FloorColorExtractor
from fixture_detection.fixture_detection import FixtureDetector

from flags import SHOW_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID, DO_WAIT_KEY


class Mapper:
    def __init__(self, tile_size, robot_diameter, camera_distance_from_center):
        self.tile_size = tile_size
        self.quarter_tile_size = tile_size / 2
        self.robot_diameter = robot_diameter

        self.robot_position = None
        self.robot_orientation = None
        self.start_position = None

        self.robot_grid_index = None

        # Data structures
        pixels_per_tile = 10
        self.pixel_grid = CompoundExpandablePixelGrid(initial_shape=np.array([1, 1]), 
                                                      pixel_per_m=pixels_per_tile / self.quarter_tile_size, 
                                                      robot_radius_m=(self.robot_diameter / 2) -0.008)
        
        self.tile_color_grid = TileColorExpandableGrid(initial_shape=np.array((1, 1)),
                                                       tile_size=self.tile_size)

        #Data processors
        self.wall_mapper = WallMapper(self.pixel_grid, robot_diameter)
        self.floor_mapper = FloorMapper(pixel_grid=self.pixel_grid, 
                                        tile_resolution=pixels_per_tile * 2,
                                        tile_size=self.tile_size,
                                        camera_distance_from_center=camera_distance_from_center)
        
        self.filterer = ArrayFilterer()
        
        self.robot_mapper = RobotMapper(pixel_grid=self.pixel_grid,
                                        robot_diameter=self.robot_diameter,
                                        pixels_per_m=pixels_per_tile / self.quarter_tile_size)

        self.fixture_mapper = FixtureMapper(pixel_grid=self.pixel_grid,
                                            tile_size=self.tile_size)
        

        # Data extractors
        self.point_cloud_extractor = PointCloudExtarctor(resolution=6)
        self.floor_color_extractor = FloorColorExtractor(tile_resolution=50)

        self.fixture_detector = FixtureDetector(self.pixel_grid)

    def update(self, in_bounds_point_cloud: list = None, 
               out_of_bounds_point_cloud: list = None,
               lidar_detections: list = None,
               camera_images: list = None, 
               robot_position: Position2D = None, 
               robot_orientation: Angle = None):
        
        if robot_position is None or robot_orientation is None:
            return
        
        self.robot_position = robot_position
        self.robot_orientation = robot_orientation

        self.robot_grid_index = self.pixel_grid.coordinates_to_grid_index(self.robot_position)

        # Load walls and obstacles (Lidar detections)
        if in_bounds_point_cloud is not None and out_of_bounds_point_cloud is not None:
            self.wall_mapper.load_point_cloud(in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position)
        
        self.robot_mapper.map_traversed_by_robot(self.robot_grid_index)
        self.robot_mapper.map_seen_by_camera(self.robot_grid_index, self.robot_orientation)
        self.robot_mapper.map_discovered_by_robot(self.robot_grid_index, self.robot_orientation)

        self.fixture_mapper.generate_detection_zone()
        self.fixture_mapper.clean_up_fixtures()

        # Load floor colors
        if camera_images is not None:
            self.floor_mapper.map_floor(camera_images, self.pixel_grid.coordinates_to_grid_index(self.robot_position))

        if camera_images is not None and lidar_detections is not None:
            self.fixture_detector.map_fixtures(camera_images, self.robot_position)
        
        self.filterer.remove_isolated_points(self.pixel_grid.arrays["occupied"])

        self.filterer.smooth_edges(self.pixel_grid.arrays["occupied"])

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

    def has_detected_victim_from_position(self):
        robot_array_index = self.pixel_grid.grid_index_to_array_index(self.robot_grid_index)
        return self.pixel_grid.arrays["robot_detected_fixture_from"][robot_array_index[0], robot_array_index[1]]