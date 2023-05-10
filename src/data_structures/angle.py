import math
from copy import copy

class Angle:
    RADIANS = 0
    DEGREES = 1
    def __init__(self, value, unit=RADIANS):
        if unit == self.RADIANS:
            self.__radians = float(value)
        else:
            self.degrees = value

    @property
    def radians(self, value):
        self.__radians = value

    @radians.getter
    def radians(self):
        return float(self.__radians)
    
    @property
    def degrees(self):
        return float(self.__radians * 180 / math.pi)
    
    @degrees.setter
    def degrees(self, value):
        self.__radians = value * math.pi / 180

    def normalize(self):
        self.__radians %= 2 * math.pi

        if self.__radians < 0:
            self.__radians += 2 + math.pi
    
    def get_absolute_distance_to(self, angle):
        angle = copy(angle)
        angle.normalize()
        min_ang = min(self.radians, angle.radians)
        max_ang = max(self.radians, angle.radians)

        clockwise_distance = max_ang - min_ang
        counterclockwise_distance = (math.pi * 2 + min_ang) - max_ang

        return Angle(min(clockwise_distance, counterclockwise_distance))
    
    def get_distance_to(self, angle):
        val = self.get_absolute_distance_to(angle)

        angle_difference = self - angle

        if 180 > angle_difference.degrees > 0 or angle_difference.degrees < -180:
            return val
        else:
            return val * -1
            
        
    
    def __str__(self):
        return str(self.degrees)
    
    def __repr__(self):
        return str(self.degrees)
    
    def __add__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians + other.radians)
        return Angle(self.radians + other)
    
    def __radd__(self, other):
        return self.__add__(other)
    
    def __sub__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians - other.radians)
        return Angle(self.radians - other)
    
    def __rsub__(self, other):
        return self.__sub__(other)
    
    def __mul__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians * other.radians)
        return Angle(self.radians * other)
    
    def __rmul__(self, other):
        return self.__mul__(other)
    
    def __truediv__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians / other.radians)
        return Angle(self.radians / other)
    
    def __rtruediv__(self, other):
        return self.__truediv__(other)
    
    def __floordiv__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians // other.radians)
        return Angle(self.radians // other)
    
    def __rfloordiv__(self, other):
        return self.__floordiv__(other)
    
    def __mod__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians % other.radians)
        return Angle(self.radians % other)
    
    def __rmod__(self, other):
        return self.__mod__(other)
    
    def __divmod__(self, other):
        if isinstance(other, Angle):
            return (Angle(self.radians // other.radians), Angle(self.radians % other.radians))
        return (Angle(self.radians // other), Angle(self.radians % other))
    
    def __rdivmod__(self, other):
        return self.__divmod__(other)
    
    def __pow__(self, other):
        if isinstance(other, Angle):
            return Angle(self.radians ** other.radians)
        return Angle(self.radians ** other)
    
    def __rpow__(self, other):
        return self.__pow__(other)
    
    def __neg__(self):
        return Angle(-self.radians)

    def __pos__(self):
        return self

    def __abs__(self):
        return Angle(abs(self.radians))

    def __eq__(self, other):
        if isinstance(other, Angle):
            return self.radians == other.radians
        return self.radians == other

    def __ne__(self, other):
        if isinstance(other, Angle):
            return self.radians != other.radians
        return self.radians != other

    def __lt__(self, other):
        if isinstance(other, Angle):
            return self.radians < other.radians
        return self.radians < other

    def __le__(self, other):
        if isinstance(other, Angle):
            return self.radians <= other.radians
        return self.radians <= other

    def __gt__(self, other):
        if isinstance(other, Angle):
            return self.radians > other.radians
        return self.radians > other

    def __ge__(self, other):
        if isinstance(other, Angle):
            return self.radians >= other.radians
        return self.radians >= other

    def __int__(self):
        return int(self.radians)

    def __float__(self):
        return float(self.radians)

    def __complex__(self):
        return complex(self.radians)
    
    def __round__(self, ndigits=None):
        return Angle(round(self.__radians, ndigits))
    
