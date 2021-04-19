import math

# Corrects the given angle in degrees to be in a range from 0 to 360
def normalizeDegs(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    if ang == 360:
        ang = 0
    return ang

def normalizeRads(rad):
    ang = radsToDegs(rad)
    normAng = normalizeDegs(ang)
    return degsToRads(normAng)

def degsToRads(deg):
    return deg * math.pi / 180

def radsToDegs(rad):
    return rad * 180 / math.pi

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle in radians and distance
def getCoordsFromRads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]

# Gets x, y coordinates from a given angle in degrees and distance
def getCoordsFromDegs(deg, distance):
    rad = degsToRads(deg)
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return [x, y]


# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

def isInRange(val, minVal, maxVal):
    return minVal < val < maxVal