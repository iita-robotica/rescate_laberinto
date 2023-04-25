from robot.devices.gps import Gps
from robot.devices.gyroscope import Gyroscope
from data_structures.angle import Angle
from data_structures.vectors import Position2D

from flags import SHOW_DEBUG


class PoseManager:
    GPS = 0
    GYROSCOPE = 1

    def __init__(self, gps: Gps, gyroscope: Gyroscope, position_offsets=Position2D(0, 0)) -> None:
        self.maximum_angular_velocity_for_gps = 0.00001

        self.gps = gps
        self.gyroscope = gyroscope

        self.orientation = Angle(0)
        self.previous_orientation = Angle(0)

        self.__position = Position2D(0, 0)
        self.__previous_position = Position2D(0, 0)

        self.orientation_sensor = self.GYROSCOPE
        self.automatically_decide_orientation_sensor = True

        self.position_offsets = position_offsets
    
    def update(self, wheel_direction):
        # Gyro and gps update
        self.gps.update()
        self.gyroscope.update()
        
        # Get global position
        self.__previous_position = self.position
        self.__position = self.gps.get_position()

        # Decides wich sensor to use for orientation detection
        if self.automatically_decide_orientation_sensor:
            self.decide_orientation_sensor(wheel_direction)

        # Remembers the corrent rotation for the next timestep
        self.previous_orientation = self.orientation

        self.calculate_orientation()

    def decide_orientation_sensor(self, wheel_direction):
        """if the robot is going srtaight it tuses the gps. If not it uses the gyro."""
        if self.robot_is_going_straight(wheel_direction):
                self.orientation_sensor = self.GPS
        else:
            self.orientation_sensor = self.GYROSCOPE

    def robot_is_going_straight(self, wheel_direction):
        return self.gyroscope.get_angular_velocity() < self.maximum_angular_velocity_for_gps and wheel_direction >= 0

    def calculate_orientation(self):
        
         # Gets global rotation
        gps_orientation = self.gps.get_orientation()

        if self.orientation_sensor == self.GYROSCOPE or gps_orientation is None:
            self.orientation = self.gyroscope.get_orientation()
            if SHOW_DEBUG: print("USING GYRO")
        else:
            self.orientation = gps_orientation
            self.gyroscope.set_orientation(self.orientation)
            if SHOW_DEBUG: print("USING GPS")

    @property
    def position(self):
        return self.__position + self.position_offsets
    
    @property
    def previous_position(self):
        return self.__previous_position + self.position_offsets
        

