# In charge of low level movement
from controller import Robot

import utilities

# devices
from devices.wheel import Wheel

from devices.camera import Camera
from devices.colour_sensor import ColourSensor

from devices.lidar import Lidar

from devices.gps import Gps
from devices.gyroscope import Gyroscope

from devices.comunicator import Comunicator


# Abstraction layer for robot
class RobotLayer:
    def __init__(self, time_step):
        # Maximum wheel speed
        self.max_wheel_speed = 6.28
        # The timestep
        self.time_step = time_step

        self.diameter = 0.074
        # Robot object provided by webots
        self.robot = Robot()
        self.prev_rotation = 0
        self.rotation = 0
        self.position = [0, 0]
        self.prev_global_position = [0, 0]
        self.position_offsets = [0, 0]

        self.rotation_sensor = "gyro"

        self.time = 0
        self.rotate_to_degs_first_time = True
        self.delay_first_time = True
        self.delay_start = self.robot.getTime()

        self.auto_decide_rotation = True
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.time_step)
        self.gps = Gps(self.robot.getDevice("gps"), self.time_step)
        self.lidar = Lidar(self.robot.getDevice("lidar"), self.time_step, 0.03, (0, 360))
        self.left_wheel = Wheel(self.robot.getDevice("wheel1 motor"), self.max_wheel_speed)
        self.right_wheel = Wheel(self.robot.getDevice("wheel2 motor"), self.max_wheel_speed)

        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.time_step)
        self.center_camera = Camera(self.robot.getDevice("camera1"), self.time_step)
        self.right_camera = Camera(self.robot.getDevice("camera2"), self.time_step)
        self.left_camera = Camera(self.robot.getDevice("camera3"), self.time_step)
        

        self.point_is_close = False

        self.stuck_counter = 0

    def delay_sec(self, delay):
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

    def rotate_to_degs(self, degs, orientation="closest", max_speed=0.5):
        accuracy = 2
        if self.rotate_to_degs_first_time:
            # print("STARTED ROTATION")
            self.rotate_to_degs_first_time = False
        self.seqRotateToDegsInitialRot = self.rotation
        self.seqRotateToDegsinitialDiff = round(self.seqRotateToDegsInitialRot - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(utilities.mapVals(moveDiff, accuracy, 90, 0.2, 0.8), max_speed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotate_to_degs_first_time = True
            return True
        else:
            if orientation == "closest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation

            if moveDiff > 10:
                if direction == "right":
                    self.move_wheels(speedFract * -1, speedFract)
                elif direction == "left":
                    self.move_wheels(speedFract, speedFract * -1)
            else:
                if direction == "right":
                    self.move_wheels(speedFract * -0.5, speedFract)
                elif direction == "left":
                    self.move_wheels(speedFract, speedFract * -0.5)
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(self.seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False
    
    def rotate_smoothly_to_degs(self, degs, orientation="closest", maxSpeed=0.5):
        accuracy = 2
        seqRotateToDegsinitialDiff = round(self.rotation - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(utilities.mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotate_to_degs_first_time = True
            return True
        else:
            if orientation == "closest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation
            if direction == "right":
                self.move_wheels(speedFract * -0.5, speedFract)
            elif direction == "left":
                self.move_wheels(speedFract, speedFract * -0.5)
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False

    def move_to_coords(self, targetPos):
        errorMargin = 0.01
        descelerationStart = 0.5 * 0.12
        diffX = targetPos[0] - self.position[0]
        diffY = targetPos[1] - self.position[1]
        # print("Target Pos: ", targetPos)
        # print("Used global Pos: ", self.position)
        # print("diff in pos: " + str(diffX) + " , " + str(diffY))
        dist = utilities.getDistance((diffX, diffY))
        print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            # self.robot.move(0,0)
            print("FinisehedMove")
            return True
        else:
            
            ang = utilities.getDegsFromCoords((diffX, diffY))
            ang = utilities.normalizeDegs(ang)
            # print("traget ang: " + str(ang))
            ratio = min(utilities.mapVals(dist, 0, descelerationStart, 0.1, 1), 1)
            ratio = max(ratio, 0.8)
            if self.rotate_to_degs(ang):
                self.move_wheels(ratio, ratio)
                # print("Moving")
        return False
    
    # Gets a point cloud with all the detections from lidar and distance sensorss
    def get_detection_point_cloud(self):
        point_clouds = self.lidar.getPointCloud(layers=(2, 3))
        self.point_is_close = self.lidar.pointIsClose
        return point_clouds
    
    def get_camera_images(self):
        return [self.right_camera.getImg(), self.center_camera.getImg(), self.left_camera.getImg()]
    
    # Returns True if the simulation is running
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
        # Updates the current time
        self.time = self.robot.getTime()
        # Updates the gps, gyroscope
        self.gps.update()
        self.gyroscope.update(self.time)

        # Gets global position
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
            print("USING GYRO")
        else:
            print("USING GPS")
            val = self.gps.getRotation()
            if val is not None:
                self.rotation = val
            self.gyroscope.setDegrees(self.rotation)

        # Sets lidar rotation
        self.lidar.setRotationDegrees(self.rotation + 0)

        #print("Delay time:", self.time - self.delayStart)

        if self.is_stuck_this_step():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        self.comunicator.update()
