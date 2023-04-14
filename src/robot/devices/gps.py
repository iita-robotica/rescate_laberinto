from data_structures.vectors import Position2D

from robot.devices.sensor import Sensor

class Gps(Sensor):
    """
    Tracks global position and rotation.
    """
    def __init__(self, webots_device, time_step, coords_multiplier=1):
        super().__init__(webots_device, time_step)
        self.multiplier = coords_multiplier
        self.__prev_position = Position2D()
        self.position = self.get_position()

    def update(self):
        """
        Updates gps, must run every timestep.
        """
        self.__prev_position = self.position
        self.position = self.get_position()

    def get_position(self):
        """
        Returns the global position.
        """
        vals = self.device.getValues()
        return Position2D(vals[0] * self.multiplier, vals[2] * self.multiplier)

    def get_orientation(self):
        """
        Returns the global orientation according to gps. This is calculated from the difference in angle from the current position
        to the position of the previous time_step (The robot must be driving perfectly straight for it to work).
        """
        if self.__prev_position != self.position:
            accuracy = abs(self.position.get_distance_to(self.__prev_position))
            if accuracy > 0.001:
                angle = self.__prev_position.get_angle_to(self.position)
                angle.normalize()
                return angle
        return None