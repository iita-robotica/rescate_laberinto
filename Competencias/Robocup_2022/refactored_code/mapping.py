import numpy as np
import copy
import cv2 as cv
import math

import utilities
from data_processing import camera_processor, data_extractor, point_cloud_processor
from data_structures import lidar_persistent_grid, expandable_node_grid
from algorithms.expandable_node_grid.bfs import bfs

from flags import SHOW_DEBUG

class Mapper:
    def __init__(self, tile_size):
        self.tile_size = tile_size

        # Data structures
        self.node_grid = expandable_node_grid.Grid((1, 1))
        self.lidar_grid = lidar_persistent_grid.LidarGrid(tile_size, 6, 100)

        res = 50 / tile_size

        # Data processors
        self.point_cloud_processor = point_cloud_processor.PointCloudProcessor(350, res)
        self.camera_processor = camera_processor.CameraProcessor(100)

        # Data extractors
        self.point_cloud_extractor = data_extractor.PointCloudExtarctor(resolution=6)
        self.floor_color_extractor = data_extractor.FloorColorExtractor(50)

        self.robot_node = None
        self.robot_vortex_center = None

        self.start_node = None
    
    def register_start(self, robot_position):
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        robot_node = [int(t * 2) for t in robot_vortex]
        self.start_node = robot_node
        for adj in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            adj = utilities.sumLists(robot_node, adj)
            self.node_grid.get_node(adj).tile_type = "start"
        self.node_grid.get_node(robot_node).is_start = True
    
    @utilities.do_every_n_frames(5, 32)
    def load_point_cloud(self, point_cloud, robot_position):
        point_cloud = self.point_cloud_processor.processPointCloud(point_cloud, robot_position)
        self.lidar_grid.update(point_cloud)
    
    @utilities.do_every_n_frames(5, 32)
    def lidar_to_node_grid(self):
        grid, offsets = self.point_cloud_extractor.transform_to_grid(self.lidar_grid)
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                xx = (x - offsets[0]) * 2 + 1
                yy = (y - offsets[1]) * 2 + 1
                #print(value)
                for direction in value:
                    self.node_grid.load_straight_wall((xx, yy),  direction)

    @utilities.do_every_n_frames(5, 32)
    def process_floor(self, camera_images, total_point_cloud, robot_position, robot_rotation):
        floor_image = self.camera_processor.get_floor_image(camera_images, robot_rotation)
        final_image = np.zeros(floor_image.shape, dtype=np.uint8)

        ranged_floor_image = cv.inRange(cv.cvtColor(floor_image, cv.COLOR_BGR2HSV), (0, 0, 100), (1, 1, 255))
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
          
        #self.lidar_grid.print_grid((600, 600))
        #self.lidar_grid.print_bool((600, 600))
    
    def degs_to_orientation(self, degs):
        """divides degrees in up, left, right or down"""
        if utilities.normalizeDegs(180 - 45) < degs < 180:
            return "up", "right"
        if 180 <= degs < utilities.normalizeDegs(180 + 45):
            return "up", "left"

        elif utilities.normalizeDegs(360 - 45) < degs <= 360:
            return "down", "left"
        elif 0 <= degs < utilities.normalizeDegs(0 + 45):
            return "down", "right" 
        
        elif utilities.normalizeDegs(90 - 45) < degs < 90:
            return "right", "down"
        elif 90 <= degs < utilities.normalizeDegs(90 + 45):
            return "right", "up"
        
        elif utilities.normalizeDegs(270 - 45) < degs < 270:
            return "left", "up"
        elif 270 <= degs < utilities.normalizeDegs(270 + 45):
            return "left", "down"

    
    def load_wall_fixture(self, letter, image_angle):
        
        orient = self.degs_to_orientation(utilities.normalizeDegs(image_angle))
        if SHOW_DEBUG:
            print("images_angle:", image_angle)
            print("orientation:", orient)   
        dir1, dir2 = orient
        direction = utilities.dir2list(dir1)
        direction = utilities.multiplyLists(direction, [2, 2])
        direction = utilities.sumLists(direction, utilities.dir2list(dir2))
        wall_index = utilities.sumLists(self.robot_node, direction)
        assert self.node_grid.get_node(wall_index).node_type == "wall"
        self.node_grid.get_node(wall_index).fixtures_in_wall.append(letter)

        

    def load_fixture(self, letter, camera_angle, robot_rotation):
        fixture = self.node_grid.get_node(self.robot_node).fixture
        fixture.exists = True
        fixture.type = letter

        image_angle = utilities.normalizeDegs(camera_angle + robot_rotation)
        fixture.detection_angle = image_angle
    
    def get_fixture(self):
        return self.node_grid.get_node(self.robot_node).fixture

    def set_robot_node(self, robot_position):
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        self.robot_vortex_center = [rt * self.tile_size for rt in robot_vortex]
        robot_node = [int(t * 2) for t in robot_vortex]
        self.robot_node = robot_node
        for row in self.node_grid.grid:
            for node in row:
                node.is_robots_position = False

        self.node_grid.get_node(self.robot_node).is_robots_position = True
    
    def block_front_vortex(self, robot_rotation):
        orientation = utilities.dir2list(self.degs_to_orientation(robot_rotation)[0])

        front_node = [r + (f * 2) for r, f in zip(self.robot_node, orientation)]
        self.node_grid.get_node(front_node).status = "occupied"


    def update(self, point_cloud=None, camera_images=None, robot_position=None, robot_rotation=None, current_time=None):
        if robot_position is None or robot_rotation is None:
            return
        
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        robot_node = [int(t * 2) for t in robot_vortex]
        robot_vortex_center = [rt * self.tile_size for rt in robot_vortex]
        
        distance = math.sqrt(sum([(x - y) ** 2 for x, y in zip(robot_vortex_center, robot_position)]))

        if SHOW_DEBUG:
            print("robot_vortex:", robot_vortex)

        if self.robot_node is None:
            self.set_robot_node(robot_position)
        
        if distance < 0.02:
            for adj in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
                adj_node = utilities.sumLists(robot_node, adj)
                self.node_grid.get_node(adj_node).explored = True
        if distance < 0.02:
            self.node_grid.get_node(robot_node).explored = True

        if point_cloud is not None:
            in_bounds_point_cloud, out_of_bounds_point_cloud = point_cloud
            self.load_point_cloud(current_time, in_bounds_point_cloud, robot_position)
            self.lidar_to_node_grid(current_time)

        if point_cloud is not None and camera_images is not None and current_time is not None:
            total_point_cloud = np.vstack((in_bounds_point_cloud, out_of_bounds_point_cloud))
            self.process_floor(current_time, camera_images, total_point_cloud, robot_position, robot_rotation)
            
            if SHOW_DEBUG:
                self.lidar_grid.print_grid((600, 600))
                self.lidar_grid.print_bool((600, 600))  

            #self.node_grid.print_grid()
        
        cv.waitKey(1) 
            
    
    def get_node_grid(self):
        return self.node_grid #copy.deepcopy()
    
    def get_grid_for_bonus(self):
        final_grid = []
        for row in self.get_node_grid().grid:
            final_row = []
            for node in row:
                final_row.append(node.get_representation())
            final_grid.append(final_row)
        return np.array(final_grid)
