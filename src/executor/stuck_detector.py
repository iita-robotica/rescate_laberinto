from data_structures.vectors import Position2D

class StuckDetector:
    """Checks if the robot is rotating the wheels but not actually moving."""
    def __init__(self) -> None:
        self.stuck_counter = 0

        self.stuck_threshold = 50
        self.minimum_distance_traveled = 0.001

        self.__position = Position2D(0, 0)
        self.__previous_position = Position2D(0, 0)
        self.__wheel_direction = 0

    def update(self, position, previous_position, wheel_direction):
        self.__wheel_direction = wheel_direction
        self.__position = position
        self.__previous_position = previous_position

        # Check if the robot is not moving
        if self.__is_stuck_this_step():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0    

    def is_stuck(self):
        return self.stuck_counter > self.stuck_threshold
    
    def __is_stuck_this_step(self):
        distance_traveled = self.__position.get_distance_to(self.__previous_position)
        is_rotating_wheels = self.__wheel_direction > 0
        return is_rotating_wheels and distance_traveled < self.minimum_distance_traveled


   