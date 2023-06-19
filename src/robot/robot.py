from controller import Robot as WebotsRobot

from flow_control.step_counter import StepCounter

from data_structures.angle import Angle
from data_structures.vectors import Position2D, Vector2D

# Devices
from robot.devices.wheel import Wheel
from robot.devices.camera import Camera
from robot.devices.lidar import Lidar
from robot.devices.gps import Gps
from robot.devices.gyroscope import Gyroscope
from robot.devices.comunicator import Comunicator

from robot.pose_manager import PoseManager

from robot.drive_base import DriveBase, Criteria

import cv2 as cv


class Robot:
    """
    Abstraction layer for the webots robot. In charge of low level movement and sensing.
    """
    def __init__(self, time_step):
        self.time_step = time_step
        self.__start_time = 0
        self.__time = 0

        self.diameter = 0.074 # Robot diameter in meters
        
        self.robot = WebotsRobot() # Robot object provided by webots

        self.gps = Gps(self.robot.getDevice("gps"), self.time_step)
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.time_step)
        
        self.pose_manager = PoseManager(self.gps, self.gyroscope) # This manages position and orientation

        # LIDAR
        lidar_interval = 6
        self.lidar = Lidar(webots_device = self.robot.getDevice("lidar"), 
                           time_step = self.time_step * lidar_interval, 
                           step_counter = StepCounter(lidar_interval),
                           layers_used=(2,))
        
        # Cameras
        self.camera_distance_from_center = 0.0310
        camera_interval = 3
        self.center_camera = Camera(webots_device = self.robot.getDevice("camera1"),
                                    time_step = self.time_step * camera_interval,
                                    step_counter = StepCounter(camera_interval),
                                    orientation=Angle(0, Angle.DEGREES),
                                    distance_from_center=self.camera_distance_from_center)
        
        self.right_camera = Camera(webots_device = self.robot.getDevice("camera2"),
                                   time_step = self.time_step * camera_interval,
                                   step_counter = StepCounter(camera_interval),
                                   orientation=Angle(270, Angle.DEGREES),
                                   distance_from_center=self.camera_distance_from_center)
        
        self.left_camera = Camera(webots_device = self.robot.getDevice("camera3"), 
                                  time_step = self.time_step * camera_interval, 
                                  step_counter = StepCounter(camera_interval),
                                  orientation=Angle(90, Angle.DEGREES),
                                  distance_from_center=self.camera_distance_from_center,
                                  rotate180=True)
        
        # Comunicator (Emmiter and reciever)
        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.time_step)
        
        # Low level movement
        max_wheel_speed = 6.28
        self.drive_base = DriveBase(left_wheel = Wheel(self.robot.getDevice("wheel1 motor"), max_wheel_speed), 
                                    right_wheel = Wheel(self.robot.getDevice("wheel2 motor"), max_wheel_speed),
                                    max_wheel_velocity = max_wheel_speed)

    def update(self):
        """Must run every TimeStep"""
        # Update current time
        self.__time = self.robot.getTime()

        # Update pose manager (Position and rotation)
        self.pose_manager.update(self.drive_base.get_wheel_average_angular_velocity(), 
                                 self.drive_base.get_wheel_velocity_difference())

        # Update drive base
        self.drive_base.orientation = self.orientation
        self.drive_base.position = self.position

        # Lidar update
        self.lidar.set_orientation(self.orientation)
        self.lidar.update()

        # Camera update
        self.right_camera.update(self.orientation)
        self.left_camera.update(self.orientation)
        self.center_camera.update(self.orientation)

    def do_loop(self):
        """Advances the simulation by one step and returns True if the simulation is running."""
        return self.robot.step(self.time_step) != -1
    
    def set_start_time(self):
        self.__start_time = self.robot.getTime()

    @property
    def time(self):
        return self.__time - self.__start_time

    # Wrappers for DriveBase
    @property
    def max_wheel_speed(self):
        return self.drive_base.max_wheel_velocity

    def move_wheels(self, left_ratio, right_ratio):
        self.drive_base.move_wheels(left_ratio, right_ratio)

    def rotate_to_angle(self, angle, direction=Criteria.CLOSEST):
        return self.drive_base.rotate_to_angle(Angle(angle, Angle.DEGREES), direction)
    
    def rotate_slowly_to_angle(self, angle, direction=Criteria.CLOSEST):
        return self.drive_base.rotate_slowly_to_angle(angle, direction)
        

    def move_to_coords(self, targetPos):
        return self.drive_base.move_to_position(Position2D(targetPos[0], targetPos[1]))
    
    # Wrappers for lidar
    @property
    def point_is_close(self) -> bool:
        return self.lidar.is_point_close

    def get_point_cloud(self):
        return self.lidar.get_point_cloud()

    def get_out_of_bounds_point_cloud(self):
        return self.lidar.get_out_of_bounds_point_cloud()
    

    def get_lidar_detections(self):
        return self.lidar.get_detections()
    
    # Wrapper for cameras
    def get_camera_images(self):
        if self.center_camera.step_counter.check():
            return [self.right_camera.get_image(), 
                    self.center_camera.get_image(), 
                    self.left_camera.get_image()]
        
    def get_last_camera_images(self):
        return [self.right_camera.get_last_image(),
                self.center_camera.get_last_image(),
                self.left_camera.get_last_image()]
        
    # Wrappers for pose
    @property
    def position(self):
        return self.pose_manager.position
    
    @property
    def raw_position(self):
        return self.pose_manager.raw_position
    
    @property
    def previous_position(self):
        return self.pose_manager.previous_position
    
    @property
    def position_offsets(self):
        return self.pose_manager.position_offsets
    
    @position_offsets.setter
    def position_offsets(self, value):
        self.pose_manager.position_offsets = value
    
    @property
    def orientation(self):
        return self.pose_manager.orientation
    
    @property
    def previous_orientation(self):
        return self.pose_manager.previous_orientation
    
    @property
    def auto_decide_orientation_sensor(self):
        return self.pose_manager.automatically_decide_orientation_sensor
    
    @auto_decide_orientation_sensor.setter
    def auto_decide_orientation_sensor(self, value):
        self.pose_manager.automatically_decide_orientation_sensor = value

    @property
    def orientation_sensor(self):
        return self.pose_manager.orientation_sensor
    
    @orientation_sensor.setter
    def orientation_sensor(self, value):
        self.pose_manager.orientation_sensor = value
    
    @property
    def GPS(self):
        return PoseManager.GPS
    
    @property
    def GYROSCOPE(self):
        return PoseManager.GYROSCOPE
    

    def is_shaky(self):
        return self.pose_manager.is_shaky()
    