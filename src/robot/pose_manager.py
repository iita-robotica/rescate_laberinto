from robot.devices.gps import Gps
from robot.devices.gyroscope import Gyroscope
from data_structures.angle import Angle
from data_structures.vectors import Position2D

from flags import SHOW_DEBUG


class PoseManager:
    GPS = 0
    GYROSCOPE = 1

    def __init__(self, gps: Gps, gyroscope: Gyroscope, position_offsets=Position2D(0, 0)) -> None:
        self.maximum_angular_velocity_for_gps = Angle(1, Angle.DEGREES)
        #self.maximum_angular_velocity_change_for_shaky = Angle(3, Angle.DEGREES)
        self.maximum_angular_velocity_change_for_shaky = Angle(1, Angle.DEGREES)

        self.gps = gps
        self.gyroscope = gyroscope

        self.orientation = Angle(0)
        self.previous_orientation = Angle(0)

        self.__position = Position2D(0, 0)
        self.__previous_position = Position2D(0, 0)

        self.orientation_sensor = self.GYROSCOPE
        self.previous_orientation_sensor = self.GYROSCOPE
        self.automatically_decide_orientation_sensor = True

        self.position_offsets = position_offsets

        self.shaky_threshold = Angle(5, unit=Angle.DEGREES)
    
    def update(self, average_wheel_velocity, wheel_velocity_difference):
        # Gyro and gps update
        self.gps.update()
        self.gyroscope.update()
        
        # Get global position
        self.__previous_position = self.__position
        self.__position = self.gps.get_position()

        # Decides wich sensor to use for orientation detection
        if self.automatically_decide_orientation_sensor:
            self.decide_orientation_sensor(average_wheel_velocity, wheel_velocity_difference)

        # Remembers the corrent rotation for the next timestep
        self.previous_orientation = self.orientation

        self.calculate_orientation()

    def decide_orientation_sensor(self, average_wheel_velocity, wheel_velocity_difference):
        """if the robot is going srtaight it tuses the gps. If not it uses the gyro."""
        if self.robot_is_going_straight(average_wheel_velocity, wheel_velocity_difference):
                self.orientation_sensor = self.GPS
        else:
            self.orientation_sensor = self.GYROSCOPE

    def robot_is_going_straight(self, average_wheel_velocity, wheel_velocity_difference) -> bool:
        return self.gyroscope.get_angular_velocity() < self.maximum_angular_velocity_for_gps and \
               average_wheel_velocity >= 1 and \
               wheel_velocity_difference < 3
               

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
    def raw_position(self):
        return self.__position
    
    @property
    def previous_position(self):
        return self.__previous_position + self.position_offsets
    
    def is_shaky(self) -> bool:
        high_orient_diff = self.orientation.get_absolute_distance_to(self.previous_orientation) > self.shaky_threshold
        changed_direction = self.gyroscope.angular_velocity * self.gyroscope.previous_angular_velocity < 0
        high_angular_velocity_difference = self.gyroscope.previous_angular_velocity.get_absolute_distance_to(self.gyroscope.angular_velocity) > self.maximum_angular_velocity_change_for_shaky

        #print(high_orient_diff, changed_direction, high_angular_velocity_difference)

        return  high_orient_diff or \
                changed_direction or \
                high_angular_velocity_difference
        

