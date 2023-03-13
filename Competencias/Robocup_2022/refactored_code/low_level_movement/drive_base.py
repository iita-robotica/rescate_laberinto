import utilities
from enum import Enum
from data_structures.angle import Angle, Unit

class DriveBase:
    def __init__(self, left_wheel, right_wheel) -> None:
        self.rotation_manager = RotationManager(left_wheel, right_wheel)
        self.movement_manager = MovementToCoordinatesManager()
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel


    # Moves the wheels at the specified ratio
    def move_wheels(self, left_ratio, right_ratio):
        self.left_wheel.move(left_ratio)
        self.right_wheel.move(right_ratio)


class Criteria(Enum):
    LEFT = 1
    RIGHT = 2
    CLOSEST = 3
    FARTHEST = 4
    

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

        self.velocity_reduction_threshold = Angle(10, Unit.DEGREES)

        self.accuracy = Angle(2, Unit.DEGREES)

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
        velocity = utilities.mapVals(absolute_difference.degrees, self.accuracy.degrees, 90, self.min_velocity, self.max_velocity)

        if absolute_difference < self.velocity_reduction_threshold:
            velocity *= 0.5

        velocity = min(velocity, self.max_velocity_cap)
        velocity = max(velocity, self.min_velocity_cap)


        direction = self.__get_direction(target_angle, criteria)
        
        if direction == self.Directions.RIGHT:
            self.left_wheel.move(velocity * -1)
            self.right_wheel.move(velocity)
        elif direction == self.Directions.LEFT:
            self.left_wheel.move(velocity)
            self.right_wheel.move(velocity * -1)
    
    def is_at_angle(self, angle):
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
    def __init__(self) -> None:
        pass
    

