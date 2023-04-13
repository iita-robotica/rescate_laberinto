from controller import Robot

import utilities

from flow_control.step_counter import StepCounter

# Devices
from devices.wheel import Wheel

from devices.camera import Camera

from devices.lidar import Lidar

from devices.gps import Gps
from devices.gyroscope import Gyroscope

from devices.comunicator import Comunicator

from low_level_movement.drive_base import DriveBase, Criteria
from data_structures.angle import Angle, Unit
from data_structures.vectors import Position2D, Vector2D

from flags import SHOW_DEBUG

# Abstraction layer for robot
# In charge of low level movement
class RobotLayer:
    def __init__(self, time_step):
        
        # The timestep
        self.time_step = time_step
        
        #Robot diameter in mts.
        self.diameter = 0.074

        # Robot object provided by webots
        self.robot = Robot()

        #Location data
        self.prev_rotation = Angle(0, Unit.DEGREES)
        self.rotation = Angle(0, Unit.DEGREES)
        self.position = Position2D(0, 0)
        self.prev_global_position = Position2D(0, 0)
        self.position_offsets = Position2D(0, 0)

        self.time = 0

        # Function specific variables
        self.delay_first_time = True
        self.delay_start = self.robot.getTime()

        # Position and rotation sensors
        self.auto_decide_rotation = True
        self.rotation_sensor = "gyro"

        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.time_step)
        self.gps = Gps(self.robot.getDevice("gps"), self.time_step)

        # LIDAR
        lidar_interval = 6
        self.lidar = Lidar(webots_device = self.robot.getDevice("lidar"), 
                           time_step = self.time_step * lidar_interval, 
                           step_counter = StepCounter(lidar_interval),
                           layers_used=(2,))

        # Cameras
        camera_interval = 3
        self.center_camera = Camera(webots_device = self.robot.getDevice("camera1"),
                                    time_step = self.time_step * camera_interval,
                                    step_counter = StepCounter(camera_interval))
        
        self.right_camera = Camera(webots_device = self.robot.getDevice("camera2"),
                                   time_step = self.time_step * camera_interval,
                                   step_counter = StepCounter(camera_interval))
        
        self.left_camera = Camera(webots_device = self.robot.getDevice("camera3"), 
                                  time_step = self.time_step * camera_interval, 
                                  step_counter = StepCounter(camera_interval),
                                  rotate180=True)

        
        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.time_step)
        
        # Low level movement
        # Maximum wheel speed
        max_wheel_speed = 6.28
        self.drive_base = DriveBase(left_wheel = Wheel(self.robot.getDevice("wheel1 motor"), max_wheel_speed), 
                                    right_wheel = Wheel(self.robot.getDevice("wheel2 motor"), max_wheel_speed),
                                    max_wheel_velocity = max_wheel_speed)

        # Stuck counter
        self.stuck_counter = 0

    def delay_sec(self, delay):
        if SHOW_DEBUG:
            print("Current delay: ", delay)
        if self.delay_first_time:
            self.delay_start = self.robot.getTime()
            self.delay_first_time = False
        else:
            if self.time - self.delay_start >= delay:
                
                self.delay_first_time = True
                return True
        return False

    # Wrappers for DriveBase
    @property
    def max_wheel_speed(self):
        return self.drive_base.max_wheel_velocity

    def move_wheels(self, left_ratio, right_ratio):
        self.drive_base.move_wheels(left_ratio, right_ratio)

    def rotate_to_angle(self, angle, direction=Criteria.CLOSEST):
        return self.drive_base.rotate_to_angle(Angle(angle, Unit.DEGREES), direction)

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
    
    # Wrapper for cameras
    def get_camera_images(self):
        if self.center_camera.step_counter.check():
            return [self.right_camera.get_image(), 
                    self.center_camera.get_image(), 
                    self.left_camera.get_image()]
    
    # Loops and returns True if the simulation is running
    def do_loop(self):
        return self.robot.step(self.time_step) != -1
    
    
    def is_stuck_this_step(self):
        return self.drive_base.get_wheel_direction() > 0 and self.position.get_distance_to(self.prev_global_position) < 0.00001

    def is_stuck(self):
        return self.stuck_counter > 50

    # Must run every TimeStep
    def update(self):
        # Update current time
        self.time = self.robot.getTime()

        # Gyro and gps update
        self.gps.update()
        self.gyroscope.update(self.time)
        
        # Get global position
        self.prev_global_position = self.position
        self.position = self.gps.getPosition()
        self.position += self.position_offsets

        # Decides wich sensor to use for roatation detection
        # if the robot is going srtaight i tuses the gps
        if self.auto_decide_rotation:
            if self.gyroscope.getDiff() < 0.00001 and self.drive_base.get_wheel_direction() >= 0:
                self.rotation_sensor = "gps"
            # if it isn't going straight it uses the gyro
            else:
                self.rotation_sensor = "gyro"

        # Remembers the corrent rotation for the next timestep
        self.prev_rotation = self.rotation.degrees

        # Gets global rotation
        if self.rotation_sensor == "gyro":
            self.rotation = self.gyroscope.get_angle()
            if SHOW_DEBUG:
                print("USING GYRO")
        else:
            if SHOW_DEBUG:
                print("USING GPS")
            val = self.gps.getRotation()
            if val is not None:
                self.rotation = val
            self.gyroscope.set_angle(self.rotation)

        self.drive_base.orientation = self.rotation
        self.drive_base.position = Position2D(self.position[0], self.position[1])

        # Lidar and camera update
        self.lidar.set_rotation(self.rotation)
        self.lidar.update()

        self.right_camera.update()
        self.left_camera.update()
        self.center_camera.update()

        # Check if the robot is not moving
        if self.is_stuck_this_step():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
