import math
import cv2 as cv
import os

script_dir = os.path.dirname(__file__)
image_dir = os.path.join(script_dir, "images")

def save_image(image, filename):
    cv.imwrite(os.path.join(image_dir, filename), image)

# Corrects the given angle in degrees to be in a range from 0 to 360
def normalizeDegs(ang):
    ang = ang % 360
    if ang < 0:
        ang += 360
    if ang == 360:
        ang = 0
    return ang

# Corrects the given angle in radians to be in a range from 0 to a full rotaion
def normalizeRads(rad):
    ang = radsToDegs(rad)
    normAng = normalizeDegs(ang)
    return degsToRads(normAng)

# Converts from degrees to radians
def degsToRads(deg):
    return deg * math.pi / 180

# Converts from radians to degrees
def radsToDegs(rad):
    return rad * 180 / math.pi

# Converts a number from a range of value to another
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Gets x, y coordinates from a given angle in radians and distance
def getCoordsFromRads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)

# Gets x, y coordinates from a given angle in degrees and distance
def getCoordsFromDegs(deg, distance):
    rad = degsToRads(deg)
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)

def getRadsFromCoords(coords):
    return math.atan2(coords[0], coords[1])


def getDegsFromCoords(coords):
    rads = math.atan2(coords[0], coords[1])
    return radsToDegs(rads)

# Gets the distance to given coordinates
def getDistance(position):
    return math.sqrt((position[0] ** 2) + (position[1] ** 2))

# Checks if a value is between two values
def isInRange(val, minVal, maxVal):
    return minVal < val < maxVal

def roundDecimal(number, decimal):
    return (round(number * decimal) / decimal)

def multiplyLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 * item2)
    return finalList

def sumLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 + item2)
    return finalList

def substractLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 - item2)
    return finalList

def divideLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 / item2)
    return finalList


def draw_grid(image, square_size, offset = [0,0], color=255):
    for y, row in enumerate(image):
        for x, pixel in enumerate(row):
            if (y + offset[1]) % square_size == 0 or (x + offset[0]) % square_size == 0:
                image[y][x][:] = color

def draw_poses(image, poses, color=255, back_image = None, xx_yy_format = False):
    if xx_yy_format:
        cropped_poses = [[], []]
        cropped_poses[0] = poses[0][poses[0] < image.shape[0]]
        cropped_poses[1] = poses[1][poses[1] < image.shape[1]]

        cropped_poses[0] = cropped_poses[0][cropped_poses[0] < back_image.shape[0]]
        cropped_poses[1] = cropped_poses[1][cropped_poses[1] < back_image.shape[1]]

        print("back_image_shape", back_image.shape)
        print("image_shape", image.shape)

        if back_image is None:
            image[cropped_poses[1], cropped_poses[0]][:] = color
        else:
            image[cropped_poses[1]][cropped_poses[0]][:] = back_image[cropped_poses[1]][cropped_poses[0]][:]

    for pos in poses:
        if pos[0] < 0 or pos[1] < 0:
            continue
        if pos[0] >= image.shape[1] or pos[1] >= image.shape[0]:
            continue
        if back_image is None:
            image[pos[1]][pos[0]][:] = color
        else:
            image[pos[1]][pos[0]][:] = back_image[pos[1]][pos[0]][:]