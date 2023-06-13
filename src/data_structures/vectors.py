import math
import numpy as np

from data_structures.angle import Angle

class Position2D:
    def __init__(self, *args, **kwargs):
        """
        Takes either two values or an iterable with at least two indices.
        """
        if len(args) == 0:
            self.x = None
            self.y = None
        elif len(args) == 1:
            self.x = args[0][0]
            self.y = args[0][1]
        elif len(args) == 2:
            self.x = args[0]
            self.y = args[1]
        else:
            raise TypeError()

    
    def __iter__(self):
        yield self.x
        yield self.y
    
    def __array__(self, *args, **kwargs):
        return np.array([self.x, self.y], *args, **kwargs)
        
    def __repr__(self):
        return f"Position2D({self.x}, {self.y})"
    
    def __eq__(self, other):
        if isinstance(other, Position2D):
            return self.x == other.x and self.y == other.y
        else:
            return False
    
    def __add__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x + other.x, self.y + other.y)
        else:
            return Position2D(self.x + other, self.y + other)
    
    def __radd__(self, other):
        return self + other
    
    def __sub__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x - other.x, self.y - other.y)
        else:
            return Position2D(self.x - other, self.y - other)
    
    def __rsub__(self, other):
        return -self + other
    
    def __mul__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x * other.x, self.y * other.y)
        else:
            return Position2D(self.x * other, self.y * other)
    
    def __rmul__(self, other):
        return self * other
    
    def __truediv__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x / other.x, self.y / other.y)
        else:
            return Position2D(self.x / other, self.y / other)
    
    def __rtruediv__(self, other):
        return Position2D(other / self.x, other / self.y)
    
    def __floordiv__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x // other.x, self.y // other.y)
        return Position2D(self.x // other, self.y // other)
    
    def __rfloordiv__(self, other):
        return self.__floordiv__(other)
    
    def __mod__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x % other.x, self.y % other.y)
        else:
            return Position2D(self.x % other, self.y % other)
    
    def __rmod__(self, other):
        return self.__mod__(other)
    
    def __divmod__(self, other):
        return self.__floordiv__(other), self.__mod__(other)
    
    
    def __rdivmod__(self, other):
        return self.__divmod__(other)
    
    def __pow__(self, other):
        if isinstance(other, Position2D):
            return Position2D(self.x ** other.x, self.y ** other.y)
        else:
            return Position2D(self.x ** other, self.y ** other)
    
    def __rpow__(self, other):
        return self.__pow__(other)
    
    def __neg__(self):
        return Position2D(-self.x, -self.y)
    
    def __pos__(self):
        return Position2D(self.x, self.y)
    
    def __abs__(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    
    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        else:
            raise IndexError("Vector index out of range")
    
    def __setitem__(self, index, value):
        if index == 0:
            self.x = value
        elif index == 1:
            self.y = value
        else:
            raise IndexError("Vector index out of range")
        
    def astype(self, dtype: type):
        return self.apply_to_all(dtype)
    
    def apply_to_all(self, function):
        return Position2D(function(self.x), function(self.y))
    
    def get_distance_to(self, other):
        return abs(self - other)
    
    def get_angle_to(self, other):
        delta = self - other 
        result = Angle(math.atan2(delta.x, delta.y)) + Angle(180, Angle.DEGREES)
        result.normalize()
        return result
    

    def to_vector(self):
        m = Position2D(0, 0).get_distance_to(self)
        a = Position2D(0, 0).get_angle_to(self)
        return Vector2D(a, m)
       
class Vector2D:
    def __init__(self, direction:Angle=None, magnitude=None):
        self.direction = direction
        self.magnitude = magnitude
        
    def __repr__(self):
        return f"Vector2D(direction={self.direction}, magnitude={self.magnitude})"
    
    def __eq__(self, other):
        if isinstance(other, Vector2D):
            return self.direction == other.direction and self.magnitude == other.magnitude
        else:
            return False
    
    def __add__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.direction + other.direction, self.magnitude + other.magnitude)
        else:
            raise TypeError("Argument must be of type Vector2D")
    
    def __radd__(self, other):
        return self + other
    
    def __sub__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.direction - other.direction, self.magnitude - other.y)
        else:
            raise TypeError("Argument must be of type Vector2D")
    
    def __rsub__(self, other):
        return -self + other
    
    
    def __neg__(self):
        return Vector2D(-self.direction, -self.magnitude)
    
    def __pos__(self):
        return Vector2D(self.direction, self.magnitude)
    
    
    def to_position(self):
        y = float(self.magnitude * math.cos(self.direction.radians))
        x = float(self.magnitude * math.sin(self.direction.radians))
        return Position2D(x, y)
    