import numpy as np
import copy
import cv2 as cv

import utilities
from data_processing import camera_processor, data_extractor, point_cloud_processor
from data_structures import lidar_persistent_grid, expandable_node_grid
from algorithms.expandable_node_grid.bfs import bfs

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
        self.point_cloud_extractor = data_extractor.PointCloudExtarctor(6, 5)
        self.floor_color_extractor = data_extractor.FloorColorExtractor(50)

        self.robot_node = None
    
    def register_start(self, robot_position):
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        robot_node = [int(t * 2) for t in robot_vortex]
        for adj in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            adj = utilities.sumLists(robot_node, adj)
            self.node_grid.get_node(adj).tile_type = "start"
        self.node_grid.get_node(robot_node).is_start = True
    
    def load_point_cloud(self, point_cloud, robot_position):
        point_cloud = self.point_cloud_processor.processPointCloud(point_cloud, robot_position)
        self.lidar_grid.update(point_cloud)
    
    def lidar_to_node_grid(self):
        grid, offsets = self.point_cloud_extractor.transform_to_grid(self.lidar_grid)
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                xx = (x - offsets[0]) * 2 + 1
                yy = (y - offsets[1]) * 2 + 1
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
        print(seen_points)


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
            print(self.node_grid.get_node(tile).node_type)
            if self.node_grid.get_node(tile).tile_type != "start":
                self.node_grid.get_node(tile).tile_type = color
        
        
        #cv.imshow('final_image', utilities.resize_image_to_fixed_size(final_image, (600, 600)))
          
        #self.lidar_grid.print_grid((600, 600))
        #self.lidar_grid.print_bool((600, 600))
        
    def load_fixture(self, letter):
        fixture = self.node_grid.get_node(self.robot_node).fixture
        fixture.exists = True
        fixture.type = letter
    
    def get_fixture(self):
        return self.node_grid.get_node(self.robot_node).fixture

    def set_robot_node(self, robot_position):
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        robot_node = [int(t * 2) for t in robot_vortex]
        self.robot_node = robot_node
        for row in self.node_grid.grid:
            for node in row:
                node.is_robots_position = False

        self.node_grid.get_node(self.robot_node).is_robots_position = True


    def update(self, point_cloud=None, camera_images=None, robot_position=None, robot_rotation=None, current_time=None):
        if robot_position is None or robot_rotation is None:
            return
        
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        robot_node = [int(t * 2) for t in robot_vortex]
        #robot_vortex_center = [rt * self.tile_size for rt in robot_vortex]

        print("robot_vortex:", robot_vortex)

        if self.robot_node is None:
            self.set_robot_node(robot_position)
        
        for adj in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
                adj_node = utilities.sumLists(robot_node, adj)
                self.node_grid.get_node(adj_node).explored = True

        if point_cloud is not None:
            in_bounds_point_cloud, out_of_bounds_point_cloud = point_cloud
            self.load_point_cloud(in_bounds_point_cloud, robot_position)
            self.lidar_to_node_grid()

        if point_cloud is not None and camera_images is not None and current_time is not None:
            total_point_cloud = np.vstack((in_bounds_point_cloud, out_of_bounds_point_cloud))
            self.process_floor(current_time, camera_images, total_point_cloud, robot_position, robot_rotation)
            #self.lidar_grid.print_grid((600, 600))
            #self.lidar_grid.print_bool((600, 600))  

            #self.node_grid.print_grid()
        
        cv.waitKey(1) 
            
    
    def get_node_grid(self):
        return copy.deepcopy(self.node_grid)