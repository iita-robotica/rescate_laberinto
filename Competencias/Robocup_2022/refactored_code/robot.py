from controller import Robot

import utilities

# Devices
from devices.wheel import Wheel

from devices.camera import Camera
from devices.colour_sensor import ColourSensor

from devices.lidar import Lidar

from devices.gps import Gps
from devices.gyroscope import Gyroscope

from devices.comunicator import Comunicator

from low_level_movement.drive_base import RotationManager, Criteria
from data_structures.angle import Angle, Unit

from flags import SHOW_DEBUG

# Abstraction layer for robot
# In charge of low level movement
class RobotLayer:
    def __init__(self, time_step):
        # Maximum wheel speed
        self.max_wheel_speed = 6.28
        # The timestep
        self.time_step = time_step
        
        #Robot diameter in mts.
        self.diameter = 0.074

        # Robot object provided by webots
        self.robot = Robot()

        #Location data
        self.prev_rotation = 0
        self.rotation = 0
        self.position = [0, 0]
        self.prev_global_position = [0, 0]
        self.position_offsets = [0, 0]

        self.time = 0

        # Function specific variables
        self.delay_first_time = True
        self.delay_start = self.robot.getTime()

        self.auto_decide_rotation = True
        self.rotation_sensor = "gyro"

        #Sensors
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.time_step)
        self.gps = Gps(self.robot.getDevice("gps"), self.time_step)


        lidar_interval = 6
        self.lidar = Lidar(webots_device = self.robot.getDevice("lidar"), 
                           time_step = self.time_step * lidar_interval, 
                           step_counter = utilities.StepCounter(lidar_interval),
                           layers_used=(2,))

        camera_interval = 3
        self.center_camera = Camera(webots_device = self.robot.getDevice("camera1"),
                                    time_step = self.time_step * camera_interval,
                                    step_counter = utilities.StepCounter(camera_interval))
        
        self.right_camera = Camera(webots_device = self.robot.getDevice("camera2"),
                                   time_step = self.time_step * camera_interval,
                                   step_counter = utilities.StepCounter(camera_interval))
        
        self.left_camera = Camera(webots_device = self.robot.getDevice("camera3"), 
                                  time_step = self.time_step * camera_interval, 
                                  step_counter = utilities.StepCounter(camera_interval),
                                  rotate180=True)

        #Actuators
        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.time_step)
        self.left_wheel = Wheel(self.robot.getDevice("wheel1 motor"), self.max_wheel_speed)
        self.right_wheel = Wheel(self.robot.getDevice("wheel2 motor"), self.max_wheel_speed)

        self.rot_manager = RotationManager(self.left_wheel, self.right_wheel)

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

    # Moves the wheels at the specified ratio
    def move_wheels(self, left_ratio, right_ratio):
        self.left_wheel.move(left_ratio)
        self.right_wheel.move(right_ratio)

    def rotate_to_angle(self, angle, direction=Criteria.CLOSEST):
        self.rot_manager.rotate_to_angle(angle, direction)
        return self.rot_manager.finished_rotating

    def move_to_coords(self, targetPos):
        errorMargin = 0.01
        descelerationStart = 0.5 * 0.12
        diffX = targetPos[0] - self.position[0]
        diffY = targetPos[1] - self.position[1]
        # print("Target Pos: ", targetPos)
        # print("Used global Pos: ", self.position)
        # print("diff in pos: " + str(diffX) + " , " + str(diffY))
        dist = utilities.getDistance((diffX, diffY))
        if SHOW_DEBUG: print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            # self.robot.move(0,0)
            if SHOW_DEBUG: print("FinisehedMove")
            return True
        else:
            
            ang = utilities.getDegsFromCoords((diffX, diffY))
            ang = utilities.normalizeDegs(ang)
            # print("traget ang: " + str(ang))
            ratio = min(utilities.mapVals(dist, 0, descelerationStart, 0.1, 1), 1)
            ratio = max(ratio, 0.8)
            if self.rotate_to_angle(Angle(ang, Unit.DEGREES)):
                self.move_wheels(ratio, ratio)
                # print("Moving")
        return False
    
    
    # Wrappers for lidar
    @property
    def point_is_close(self):
        return self.lidar.is_point_close

    def get_point_cloud(self):
        return self.lidar.get_point_cloud()

    def get_out_of_bounds_point_cloud(self):
        return self.lidar.get_out_of_bounds_point_cloud()
    
    # Wrappers for camera images
    def get_camera_images(self):
        if self.center_camera.step_counter.check():
            return [self.right_camera.get_image(), 
                    self.center_camera.get_image(), 
                    self.left_camera.get_image()]
    
    # Starts loop ans returns True if the simulation is running
    def do_loop(self):
        return self.robot.step(self.time_step) != -1
    
    def get_wheel_direction(self):
        if self.right_wheel.velocity + self.left_wheel.velocity == 0:
            return 0
        return (self.right_wheel.velocity + self.left_wheel.velocity) / 2
    
    def is_stuck_this_step(self):
        return self.get_wheel_direction() > 0 and abs(utilities.getDistance(utilities.substractLists(self.position, self.prev_global_position))) < 0.00001

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
        self.position[0] += self.position_offsets[0]
        self.position[1] += self.position_offsets[1]

        # Decides wich sensor to use for roatation detection
        # if the robot is going srtaight i tuses the gps
        if self.auto_decide_rotation:
            if self.gyroscope.getDiff() < 0.00001 and self.get_wheel_direction() >= 0:
                self.rotation_sensor = "gps"
            # if it isn't going straight it uses the gyro
            else:
                self.rotation_sensor = "gyro"

        # Remembers the corrent rotation for the next timestep
        self.prev_rotation = self.rotation

        # Gets global rotation
        if self.rotation_sensor == "gyro":
            self.rotation = self.gyroscope.getDegrees()
            if SHOW_DEBUG:
                print("USING GYRO")
        else:
            if SHOW_DEBUG:
                print("USING GPS")
            val = self.gps.getRotation()
            if val is not None:
                self.rotation = val
            self.gyroscope.setDegrees(self.rotation)

        self.rot_manager.current_angle = Angle(self.rotation, Unit.DEGREES)

        # Lidar and camera update
        self.lidar.setRotationDegrees(self.rotation + 0)
        self.lidar.update()

        self.right_camera.update()
        self.left_camera.update()
        self.center_camera.update()

        # Check if the robot is not moving
        if self.is_stuck_this_step():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
