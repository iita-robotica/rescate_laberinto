from utilities import mapVals
from enum import Enum
from data_structures.angle import Angle
from data_structures.vectors import Position2D, Vector2D
import math
from flags import SHOW_DEBUG

from robot.devices.wheel import Wheel

class Criteria(Enum):
    LEFT = 1
    RIGHT = 2
    CLOSEST = 3
    FARTHEST = 4

class DriveBase:
    def __init__(self, left_wheel, right_wheel, max_wheel_velocity) -> None:
        self.max_wheel_velocity = max_wheel_velocity
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.rotation_manager = RotationManager(self.left_wheel, self.right_wheel)
        self.slow_rotation_manager = RotationManager(self.left_wheel, self.right_wheel)
        self.slow_rotation_manager.max_velocity = 0.4
        self.slow_rotation_manager.max_velocity_cap = 0.4
        #self.movement_manager = MovementToCoordinatesManager(self.left_wheel, self.right_wheel)
        self.movement_manager = SmoothMovementToCoordinatesManager(self.left_wheel, self.right_wheel)

    # Moves the wheels at the specified Velocity
    def move_wheels(self, left_ratio, right_ratio):
        self.left_wheel.move(left_ratio)
        self.right_wheel.move(right_ratio)
    
    def rotate_to_angle(self, angle:Angle, criteria:Criteria.CLOSEST) -> bool:
        self.rotation_manager.rotate_to_angle(angle, criteria)
        return self.rotation_manager.finished_rotating
    
    def rotate_slowly_to_angle(self, angle:Angle, criteria:Criteria.CLOSEST) -> bool:
        self.slow_rotation_manager.rotate_to_angle(angle, criteria)
        return self.slow_rotation_manager.finished_rotating
    
    def move_to_position(self, position:Position2D) -> bool:
        self.movement_manager.move_to_position(position)
        return self.movement_manager.finished_moving
    
    @property
    def position(self) -> Position2D:
        return self.movement_manager.current_position
    
    @position.setter
    def position(self, value:Position2D):
        self.movement_manager.current_position = value

    @property
    def orientation(self) -> Angle:
        return self.rotation_manager.current_angle
    
    @orientation.setter
    def orientation(self, value:Angle):
        self.movement_manager.current_angle = value
        self.rotation_manager.current_angle = value
        self.slow_rotation_manager.current_angle = value


    def get_wheel_average_angular_velocity(self):
        if self.right_wheel.velocity + self.left_wheel.velocity == 0:
            return 0
        return (self.right_wheel.velocity + self.left_wheel.velocity) / 2
    
    def get_wheel_velocity_difference(self):
        return self.right_wheel.velocity - self.left_wheel.velocity


class RotationManager:
    def __init__(self, left_wheel, right_wheel) -> None:
        self.Directions = Enum("Directions", ["LEFT", "RIGHT"])
        
        self.right_wheel = right_wheel
        self.left_wheel = left_wheel

        self.initial_angle = Angle(0)
        self.current_angle = Angle(0)

        self.first_time = True
        self.finished_rotating = True

        self.max_velocity_cap = 1
        self.min_velocity_cap = 0.2

        self.max_velocity = 1
        self.min_velocity = 0.2

        self.velocity_reduction_threshold = Angle(10, Angle.DEGREES)
        self.velocity_reduction_factor = 0.5

        self.accuracy = Angle(2, Angle.DEGREES)

    def rotate_to_angle(self, target_angle, criteria=Criteria.CLOSEST):
        if self.first_time:
            self.initial_angle = self.current_angle
            self.first_time = False
            self.finished_rotating = False

        if self.is_at_angle(target_angle):
            self.finished_rotating = True
            self.first_time = True
            self.left_wheel.move(0)
            self.right_wheel.move(0)

        absolute_difference = self.current_angle.get_absolute_distance_to(target_angle)
        velocity = mapVals(absolute_difference.degrees, self.accuracy.degrees, 90, self.min_velocity, self.max_velocity)

        if absolute_difference < self.velocity_reduction_threshold:
            velocity *= self.velocity_reduction_factor

        velocity = min(velocity, self.max_velocity_cap)
        velocity = max(velocity, self.min_velocity_cap)


        direction = self.__get_direction(target_angle, criteria)
        
        if direction == self.Directions.RIGHT:
            self.left_wheel.move(velocity * -1)
            self.right_wheel.move(velocity)
        elif direction == self.Directions.LEFT:
            self.left_wheel.move(velocity)
            self.right_wheel.move(velocity * -1)
    
    def is_at_angle(self, angle) -> bool:
        return self.current_angle.get_absolute_distance_to(angle) < self.accuracy

    def __get_direction(self, target_angle, criteria):
        if criteria == Criteria.CLOSEST:
            angle_difference = self.current_angle - target_angle

            if 180 > angle_difference.degrees > 0 or angle_difference.degrees < -180:
                return self.Directions.RIGHT
            else:
                return self.Directions.LEFT

        elif criteria == Criteria.FARTHEST:
            angle_difference = self.initial_angle - target_angle
            if 180 > angle_difference.degrees > 0 or angle_difference.degrees < -180:
                return self.Directions.LEFT
            else:
                return self.Directions.RIGHT

        elif criteria == Criteria.LEFT: return self.Directions.LEFT
        elif criteria == Criteria.RIGHT: return self.Directions.RIGHT


class MovementToCoordinatesManager:
    def __init__(self, left_wheel, right_wheel) -> None:
        self.current_position = Position2D()

        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.rotation_manager = RotationManager(self.left_wheel, self.right_wheel)

        #self.error_margin = 0.0005
        self.error_margin = 0.001
        self.desceleration_start = 0.5 * 0.12

        self.max_velocity_cap = 1
        self.min_velocity_cap = 0.8

        self.max_velocity = 1
        self.min_velocity = 0.1

        self.finished_moving = False

    @property
    def current_angle(self) -> Angle:
        return self.rotation_manager.current_angle
    
    @current_angle.setter
    def current_angle(self, value):
        self.rotation_manager.current_angle = value

    
    def move_to_position(self, target_position:Position2D):

        # print("Target Pos: ", targetPos)
        # print("Used global Pos: ", self.position)

        dist = abs(self.current_position.get_distance_to(target_position))

        if SHOW_DEBUG: print("Dist: "+ str(dist))

        if dist < self.error_margin:
            # self.robot.move(0,0)
            if SHOW_DEBUG: print("FinisehedMove")
            self.finished_moving = True
        else:
            self.finished_moving = False
            ang = self.current_position.get_angle_to(target_position)

            if self.rotation_manager.is_at_angle(ang):

                velocity = mapVals(dist, 0, self.desceleration_start, self.min_velocity, self.max_velocity)
                velocity = min(velocity, self.max_velocity_cap)
                velocity = max(velocity, self.min_velocity_cap)

                self.right_wheel.move(velocity)
                self.left_wheel.move(velocity)


            else:
                
                self.rotation_manager.rotate_to_angle(ang)


class SmoothMovementToCoordinatesManager:
    def __init__(self, left_wheel: Wheel, right_wheel: Wheel) -> None:
        self.current_position = Position2D()

        self.left_wheel = left_wheel
        self.right_wheel = right_wheel

        self.current_angle = Angle(0)

        self.error_margin = 0.003

        self.velocity = 1

        self.distance_weight = 5
        self.angle_weight = 5

        self.min_velocity_cap = 0

        self.turning_speed_multiplier = 0.8

        self.finished_moving = False

        self.angle_error_margin = Angle(3, Angle.DEGREES)

        self.strong_rotation_start = Angle(45, Angle.DEGREES)

        self.light_rotation_start = Angle(30, Angle.DEGREES)

    def move_to_position(self, target_position:Position2D):
        dist = abs(self.current_position.get_distance_to(target_position))

        if SHOW_DEBUG: print("Dist: "+ str(dist))

        if dist < self.error_margin:
            # self.robot.move(0,0)
            if SHOW_DEBUG: print("FinisehedMove")
            self.finished_moving = True


        else:
            self.finished_moving = False

            angle_to_target = self.current_position.get_angle_to(target_position)
            angle_diff = self.current_angle - angle_to_target
            absolute_angle_diff = self.current_angle.get_absolute_distance_to(angle_to_target)

            if absolute_angle_diff < self.angle_error_margin:
                #print("straight")
                self.right_wheel.move(self.velocity)
                self.left_wheel.move(self.velocity)


            elif absolute_angle_diff > self.strong_rotation_start:
                #print("strong")
                if 180 > angle_diff.degrees > 0 or angle_diff.degrees < -180:
                    self.right_wheel.move(self.velocity)
                    self.left_wheel.move(self.velocity * -1)
                else:
                    self.right_wheel.move(self.velocity * -1)
                    self.left_wheel.move(self.velocity)
            
            elif absolute_angle_diff < self.light_rotation_start:
                #print("light")
                if 180 > angle_diff.degrees > 0 or angle_diff.degrees < -180:
                    self.right_wheel.move(1)
                    self.left_wheel.move(0.8)
                else:
                    self.right_wheel.move(0.8)
                    self.left_wheel.move(1)


            else:
                #print("dynamic")
                distance_speed = abs(dist * -self.distance_weight)
                angle_speed = absolute_angle_diff.radians * self.angle_weight

                speed = angle_speed * self.turning_speed_multiplier

                speed = max(self.min_velocity_cap, speed)

                speed2 = speed * distance_speed
                speed2 = max(speed2, self.min_velocity_cap)

                if 180 > angle_diff.degrees > 0 or angle_diff.degrees < -180:
                    self.right_wheel.move(speed)
                    self.left_wheel.move(speed2)
                else:
                    self.right_wheel.move(speed2)
                    self.left_wheel.move(speed)


            #print("Wheel vel:", self.right_wheel.velocity / 6.28 , self.left_wheel.velocity / 6.28)
                
                
    

