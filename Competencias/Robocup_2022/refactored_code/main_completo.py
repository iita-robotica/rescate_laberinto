import cv2 as cv
import numpy as np
import time
import copy
import math


# from data_processing import fixture_detection
#############################################################################################
################################  FIXTURE DETECTION #########################################
import random 

class Filter:
    def __init__(self, lower_hsv, upper_hsv):
        self.lower = np.array(lower_hsv)
        self.upper = np.array(upper_hsv)
    
    def filter(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        #imgResult = cv.bitwise_and(img, img, mask=mask)
        return mask

#red_filter = Filter(lower_hsv=(73, 157, 127), upper_hsv=(179, 255, 255))
red_filter = Filter(lower_hsv=(160, 170, 127), upper_hsv=(170, 255, 255))
#yellow_filter = Filter(lower_hsv=(0, 157, 82), upper_hsv=(40, 255, 255))
yellow_filter = Filter(lower_hsv=(25, 157, 82), upper_hsv=(30, 255, 255))
white_filter = Filter(lower_hsv=(0, 0, 207), upper_hsv=(0, 0, 207))
black_filter = Filter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 0))
vicitim_letter_filter = Filter(lower_hsv=(0, 0, 0), upper_hsv=(5, 255, 100))

filter_for_tuning = white_filter

cv.namedWindow("trackbars")

cv.createTrackbar("min_h", "trackbars", filter_for_tuning.lower[0], 255, lambda x: None)
cv.createTrackbar("max_h", "trackbars", filter_for_tuning.upper[0], 255, lambda x: None)

cv.createTrackbar("min_s", "trackbars", filter_for_tuning.lower[1], 255, lambda x: None)
cv.createTrackbar("max_s", "trackbars", filter_for_tuning.upper[1], 255, lambda x: None)

cv.createTrackbar("min_v", "trackbars", filter_for_tuning.lower[2], 255, lambda x: None)
cv.createTrackbar("max_v", "trackbars", filter_for_tuning.upper[2], 255, lambda x: None)

def tune_filter(image):
    min_h = cv.getTrackbarPos("min_h", "trackbars")
    max_h = cv.getTrackbarPos("max_h", "trackbars")
    min_s = cv.getTrackbarPos("min_s", "trackbars")
    max_s = cv.getTrackbarPos("max_s", "trackbars")
    min_v = cv.getTrackbarPos("min_v", "trackbars")
    max_v = cv.getTrackbarPos("max_v", "trackbars")
    filter_for_tuning = Filter((min_h, min_s, min_v), (max_h, max_s, max_v))
    #print(filter_for_tuning.lower, filter_for_tuning.upper)
    cv.imshow("tunedImage", filter_for_tuning.filter(image))


def sum_images(images):
    final_img = images[0]
    for index, image in enumerate(images):
        final_img += image
        #cv.imshow(str(index), image)
    final_img[final_img > 255] = 255
    return final_img

def filter_victims(victims):
    final_victims = []
    for vic in victims:
        #print("victim:", vic["position"], vic["image"].shape)
        if vic["image"].shape[0] > 25 and vic["image"].shape[1] > 20:
            final_victims.append(vic)
    return final_victims

def find_victims(image):
    """
    Finds victims in the image.
    Returns a list of dictionaries containing vicitims positions and images.
    """
    binary_images = [red_filter.filter(image), 
                    yellow_filter.filter(image), 
                    white_filter.filter(image), 
                    black_filter.filter(image)]

    binary_image = sum_images(binary_images)
    cv.imshow("binaryImage", binary_image)
    
    # Encuentra los contornos, aunque se puede confundir con el contorno de la letra
    contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # Pra evitar la confusion dibuja rectangulos blancos donde estan los contornos en la imagen y despues vuelve a
    # sacar los contornos para obtener solo los del rectangulo, no los de las letras.
    for c0 in contours:
        x, y, w, h = cv.boundingRect(c0)
        cv.rectangle(binary_image, (x, y), (x + w, y + h), (225, 255, 255), -1)
    contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # saca las medidas y la posicion de los contornos y agrega a la lista de imagenes la parte esa de la imagen original
    # Tambien anade la posicion de cada recuadro en la imagen original
    final_victims = []
    for c in contours:
        x, y, w, h = cv.boundingRect(c)
        final_victims.append({"image":image[y:y + h, x:x + w], "position":(x, y)})
    return filter_victims(final_victims)

def crop_white(binaryImg):
    white = 255
    #print(conts)
    maxX = 0
    maxY = 0
    minX = binaryImg.shape[0]
    minY = binaryImg.shape[1]
    for yIndex, row in enumerate(binaryImg):
        for xIndex, pixel in enumerate(row):
            if pixel == white:
                maxX = max(maxX, xIndex)
                maxY = max(maxY, yIndex)
                minX = min(minX, xIndex)
                minY = min(minY, yIndex)

    return binaryImg[minY:maxY, minX:maxX]

def classify_victim(victim):
    white = 255
    img = victim["image"]
    img =  cv.resize(img, (100, 100), interpolation=cv.INTER_AREA)
    #conts, h = cv.findContours(thresh1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    binary = vicitim_letter_filter.filter(img)

    letter1 = crop_white(binary)
    letter1 = cv.resize(letter1, (100, 100), interpolation=cv.INTER_AREA)
    letter = letter1[:,10:90]
    letter = crop_white(letter)
    letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)
    cv.imshow("letra", letter)
    cv.imshow("letra1", letter1)
    cv.imshow("thresh", binary)
    letter_color = cv.cvtColor(letter, cv.COLOR_GRAY2BGR)
    area_width = 20
    area_height = 30
    areas = {
        "top": ((0, area_height),(50 - area_width // 2, 50 + area_width // 2)),
        "middle": ((50 - area_height // 2, 50 + area_height // 2), (50 - area_width // 2, 50 + area_width // 2)),
        "bottom": ((100 - area_height, 100), (50 - area_width // 2, 50 + area_width // 2 ))
        }
    images = {
        "top": letter[areas["top"][0][0]:areas["top"][0][1], areas["top"][1][0]:areas["top"][1][1]],
        "middle": letter[areas["middle"][0][0]:areas["middle"][0][1], areas["middle"][1][0]:areas["middle"][1][1]],
        "bottom": letter[areas["bottom"][0][0]:areas["bottom"][0][1], areas["bottom"][1][0]:areas["bottom"][1][1]]
        }
    cv.rectangle(letter_color,(areas["top"][1][0], areas["top"][0][0]), (areas["top"][1][1], areas["top"][0][1]), (0, 255, 0), 1)
    cv.rectangle(letter_color, (areas["middle"][1][0], areas["middle"][0][0]), (areas["middle"][1][1], areas["middle"][0][1]), (0, 0, 255), 1)
    cv.rectangle(letter_color,(areas["bottom"][1][0], areas["bottom"][0][0]), (areas["bottom"][1][1], areas["bottom"][0][1]), (225, 0, 255), 1)
    counts = {}
    for key in images.keys():
        count = 0
        for row in images[key]:
            for pixel in row:
                if pixel == white:
                    count += 1
        counts[key] = count > 20
    letters = {
        "H":{'top': False, 'middle': True, 'bottom': False},
        "S":{'top': True, 'middle': True, 'bottom': True},
        "U":{'top': False, 'middle': False, 'bottom': True}
        }

    final_letter = random.choice(list(letters.keys()))
    for letter_key in letters.keys():
        if counts == letters[letter_key]:
            final_letter = letter_key
            break
    
    #print(counts)
    #print(finalLetter)
    return final_letter


def is_poison(black_points, white_points):
    return black_points < 600 and white_points > 700 and white_points < 4000

def is_victim(black_points, white_points):
    return white_points > 5000 and 2000 > black_points > 100

def is_corrosive(black_points, white_points):
    return 700 < white_points < 2500 and 1000 < black_points < 2500

def is_flammable(red_points, white_points):
    return red_points and white_points

def is_organic_peroxide(red_points, yellow_points):
    return red_points and yellow_points

def is_already_detected(point_counts):
    if point_counts["white"] and not (point_counts["black"] + point_counts["red"] + point_counts["yellow"]):
        return True


def classify_fixture(vic):
    possible_fixture_letters = ["P", "O", "F", "C", "S", "H", "U"]
    letter = random.choice(possible_fixture_letters)
    image = cv.resize(vic["image"], (100, 100), interpolation=cv.INTER_AREA)
    color_images = {
    "red" : red_filter.filter(image),
    "yellow" : yellow_filter.filter(image), 
    "white" : white_filter.filter(image),
    "black" : black_filter.filter(image)}

    color_point_counts = {}
    for key, img in color_images.items():
        all_points = np.where(img == 255)
        all_points = all_points[0]
        count = len(all_points)
        color_point_counts[key] = count
    
    #print(color_point_counts)

    if is_poison(color_point_counts["black"], color_point_counts["white"]):
        #print("Poison!")
        letter = "P"
    
    if is_victim(color_point_counts["black"], color_point_counts["white"]):
        cv.imshow("black filter:", color_images["black"])
        letter = classify_victim(vic)
        #print("Victim:", letter)
        
    
    if is_corrosive(color_point_counts["black"], color_point_counts["white"]):
        #print("Corrosive!")
        letter = "C"
    
    if is_organic_peroxide(color_point_counts["red"], color_point_counts["yellow"]):
       # print("organic peroxide!")
        letter = "O"
    
    if is_flammable(color_point_counts["red"], color_point_counts["white"]):
       # print("Flammable!")
        letter = "F"
    
    if is_already_detected(color_point_counts):
       # print("Already detected!")
        letter = None

    return letter

# import utilities
##############################################################################################
#################################  UTILITIES  ################################################
import os
from functools import wraps

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
                if len(image.shape) == 3:
                    image[y][x][:] = color
                else:
                    image[y][x] = color

def draw_poses(image, poses, color=255, back_image = None, xx_yy_format = False):
    if xx_yy_format:
        cropped_poses = [[], []]
        if back_image is not None:
            in_bounds_x = (poses[0] < min(image.shape[0], back_image.shape[0]) - 1) * (poses[0] > 0)
            in_bounds_y = (poses[1] < min(image.shape[1], back_image.shape[1]) - 1) * (poses[1] > 0)
        else:
            in_bounds_x = (poses[0] < image.shape[0] - 1) * (poses[0] > 0)
            in_bounds_y = (poses[1] < image.shape[1] - 1) * (poses[1] > 0)
        
        cropped_poses[0] = poses[0][in_bounds_x * in_bounds_y]
        cropped_poses[1] = poses[1][in_bounds_x * in_bounds_y]

        #print("back_image_shape", back_image.shape)
        #print("image_shape", image.shape)

        
        if back_image is None:
            image[cropped_poses[1], cropped_poses[0], :] = color
        else:
            image[cropped_poses[1], cropped_poses[0], :] = back_image[cropped_poses[1], cropped_poses[0], :]
        
    for pos in poses:
        if pos[0] < 0 or pos[1] < 0:
            continue
        if pos[0] >= image.shape[1] or pos[1] >= image.shape[0]:
            continue
        if back_image is None:
            image[pos[1]][pos[0]][:] = color
        else:
            image[pos[1]][pos[0]][:] = back_image[pos[1]][pos[0]][:]

def draw_squares_where_not_zero(image, square_size, offsets, color=(255, 255, 255)):
    ref_image = image.copy()
    for y in range(image.shape[0] // square_size):
        for x in range(image.shape[1] // square_size):
            square_points = [
                (y * square_size)        + (square_size - offsets[1]),
                ((y + 1) * square_size)  + (square_size - offsets[1]), 
                (x * square_size)        + (square_size - offsets[0]),
                ((x + 1) * square_size)  + (square_size - offsets[0])]
            square = ref_image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
            non_zero_count = np.count_nonzero(square)
            if non_zero_count > 0:
                #print("Non zero count: ", non_zero_count)
                #print("max: ", np.max(square))
                cv.rectangle(image, (square_points[2], square_points[0]), (square_points[3], square_points[1]), color, 3)

def get_squares(image, square_size, offsets):
    grid = []
    for y in range(image.shape[0] // square_size):
        row = []
        for x in range(image.shape[1] // square_size):
            square_points = [
                (y * square_size)        + (square_size - offsets[1]),
                ((y + 1) * square_size)  + (square_size - offsets[1]), 
                (x * square_size)        + (square_size - offsets[0]),
                ((x + 1) * square_size)  + (square_size - offsets[0])]
            row.append(square_points)
        grid.append(row)
    return grid

def resize_image_to_fixed_size(image, size):
    if image.shape[0] > size[0]:
        ratio = size[0] / image.shape[0]

        width = round(image.shape[1] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(width, size[0]))
    
    elif image.shape[1] > size[1]:
        ratio = size[1] / image.shape[1]

        height = round(image.shape[0] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(size[1], height))
    
    elif image.shape[1] >= image.shape[0]:
        ratio = size[1] / image.shape[1]

        height = round(image.shape[0] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(size[1], height), interpolation=cv.INTER_NEAREST)
    
    elif image.shape[0] >= image.shape[1]:
        ratio = size[0] / image.shape[0]

        width = round(image.shape[1] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(width, size[0]), interpolation=cv.INTER_NEAREST)
    
    return final_image

def dir2list(direction):
    directions = {
        "up": [0, -1],
        "down": [0, 1],
        "left": [-1, 0],
        "right": [1, 0],
        "up_left": [-1, -1],
        "up_right": [1, -1],
        "down_left": [-1, 1],
        "down_right": [1, 1],
        "u": [0, -1],
        "d": [0, 1],
        "l": [-1, 0],
        "r": [1, 0],
        "ul": [-1, -1],
        "ur": [1, -1],
        "dl": [-1, 1],
        "dr": [1, 1]
    }
    return directions[direction]

def list2dir(direction):
    direction = tuple(direction)
    directions = {
        (0, -1): "up",
        (0, 1): "down",
        (-1, 0): "left",
        (1, 0): "right",
        (-1, -1): "up_left",
        (1, -1): "up_right",
        (-1, 1): "down_left",
        (1, 1): "down_right",
        (0, -1): "u",
        (0, 1): "d",
        (-1, 0): "l",
        (1, 0): "r",
        (-1, -1): "ul",
        (1, -1): "ur",
        (-1, 1): "dl",
        (1, 1): "dr"
    }
    return directions[direction]

def is_color_in_range(color, rng):
    minimums = rng[0]
    maximums = rng[1]
    for i, c in enumerate(color):
        if c < minimums[i] or c > maximums[i]:
            return False
    return True


def do_every_n_frames(n, time_step):
    def inner_function(func):
        @wraps(func)
        def wrapper(self, current_time, *args, **kwargs):
            if (current_time // (time_step / 1000)) % n == 0:
                return func(self, *args, **kwargs)
        return wrapper
    return inner_function


# import state_machines
##############################################################################################
####################################  STATES MACHINES  #######################################

# Manages states
class StateManager:
    def __init__(self, initialState):
        self.state = initialState

    # Sets the state to a certain value
    def changeState(self, newState):
        self.state = newState
        return True

    # Checks if the state corresponds to a specific value
    def checkState(self, state):
        return self.state == state

# Makes it possible to run arbitrary code sequentially without interrupting other code that must run continuoulsy
class SequenceManager:
    def __init__(self, resetFunction=None):
        self.lineIdentifier = 0
        self.linePointer = 1
        self.done = False
        self.resetFunction = resetFunction

    # Resets the sequence and makes it start from the first event
    def resetSequence(self):
        if self.resetFunction is not None:
            self.resetFunction()
        self.linePointer = 1
        #print("----------------")
        #print("reseting sequence")
        #print("----------------")

    def seqResetSequence(self):
        if self.check():
            self.resetSequence()
            
            return True
        return False

    # This has to be at the start of any sequence of events
    def startSequence(self):
        self.lineIdentifier = 0
        self.done = False

    # Returns if the line pointer and identifier match and increases the identifier
    # Must be included at the end of any sequential function
    def check(self):
        self.done = False
        self.lineIdentifier += 1
        return self.lineIdentifier == self.linePointer

    # Changes to the next event
    def nextSeq(self):
        self.linePointer += 1
        self.done = True

    # returns if the sequence has reached its end
    def seqDone(self):
        return self.done

    # Can be used to make a function sequential or used in an if statement to make a code block sequential
    def simpleEvent(self, function=None, *args, **kwargs):
        if self.check():
            if function is not None:
                function(*args, **kwargs)
            self.nextSeq()
            return True
        return False

    # The function inputted must return True when it ends
    def complexEvent(self, function, *args, **kwargs):
        if self.check():
            if function(*args, **kwargs):
                self.nextSeq()
                return True
        return False
    
    # When inpuuted any function it returns a sequential version of it that can be used in a sequence
    def makeSimpleEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                function(*args, **kwargs)
                self.nextSeq()
                return True
            return False
        return event

    # When inputted a function that returns True when it ends returns a sequential version of it that can be used in a sequence
    def makeComplexEvent(self, function):
        def event(*args, **kwargs):
            if self.check():
                if function(*args, **kwargs):
                    self.nextSeq()
                    return True
            return False
        return event


# import robot
##############################################################################################
#####################################  ROBOT  ################################################
# In charge of low level movement
from controller import Robot

# devices
##############################################################################################
#####################################  WHEEL  ################################################
# from devices.wheel import Wheel
# Controlls a wheel
class Wheel:
    def __init__(self, wheel, maxVelocity):
        self.maxVelocity = maxVelocity
        self.wheel = wheel
        self.velocity = 0
        self.wheel.setPosition(float("inf"))
        self.wheel.setVelocity(0)

    # Moves the wheel at a ratio of the maximum speed (between 0 and 1)
    def move(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < -1:
            ratio = -1
        self.velocity = ratio * self.maxVelocity
        self.wheel.setVelocity(self.velocity)
##############################################################################################
####################################  CAMERA  ################################################
# from devices.camera import Camera
import numpy as np

# Captures images and processes them
class Camera:
    def __init__(self, camera, timeStep):
        self.camera = camera
        self.camera.enable(timeStep)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()

    # Gets an image from the raw camera data
    def getImg(self):
        imageData = self.camera.getImage()
        return np.array(np.frombuffer(imageData, np.uint8).reshape((self.height, self.width, 4)))
##############################################################################################
####################################  COLOUR SENSOR  #########################################
# from devices.colour_sensor import ColourSensor
# Reads the colour sensor
class ColourSensor:
    def __init__(self, sensor, distancefromCenter, timeStep):
        self.distance = distancefromCenter
        self.position = [0, 0]
        self.sensor = sensor
        self.sensor.enable(timeStep)
        self.r = 0
        self.g = 0
        self.b = 0
    
    def setPosition(self, robotGlobalPosition, robotGlobalRotation):
        realPosition = getCoordsFromDegs(robotGlobalRotation, self.distance)
        self.position = [robotGlobalPosition[0] + realPosition[0], robotGlobalPosition[1] + realPosition[1]]
    
    def __update(self):
        colour = self.sensor.getImage()
        # print("Colourimg:", colour)
        self.r = self.sensor.imageGetRed(colour, 1, 0, 0)
        self.g = self.sensor.imageGetGreen(colour, 1, 0, 0)
        self.b = self.sensor.imageGetBlue(colour, 1, 0, 0)
        # print("Colour:", self.r, self.g, self.b)
    
    def __isTrap(self):
        return (35 < self.r < 45 and 35 < self.g < 45)

    def __isSwamp(self):
        return (200 < self.r < 210 and 165 < self.g < 175 and 95 < self.b < 105)

    def __isCheckpoint(self):
        return (self.r > 232 and self.g > 232 and self.b > 232)

    def __isNormal(self):
        return self.r == 227 and self.g == 227

    def __isBlue(self):
        return (55 < self.r < 65 and 55 < self.g < 65 and 245 < self.b < 255)

    def __isPurple(self):
        return (135 < self.r < 145 and 55 < self.g < 65 and 215 < self.b < 225)

    def __isRed(self):
        return (245 < self.r < 255 and 55 < self.g < 65 and 55 < self.b < 65)

    # Returns the type of tyle detected from the colour data
    def getTileType(self):
        self.__update()
        tileType = "undefined"
        if self.__isNormal():
            tileType = "normal"
        elif self.__isTrap():
            tileType = "hole"
        elif self.__isSwamp():
            tileType = "swamp"
        elif self.__isCheckpoint():
            tileType = "checkpoint"
        elif self.__isBlue():
            tileType = "connection1-2"
        elif self.__isPurple():
            tileType = "connection2-3"
        elif self.__isRed():
            tileType = "connection1-3"

        # print("Color: " + tileType)
        # print("r: " + str(self.r) + "g: " + str(self.g) + "b: " +  str(self.b))
        return tileType
##############################################################################################
####################################  LIDAR  #################################################
# from devices.lidar import Lidar
import math

# Returns a point cloud of the detctions it makes
class Lidar():
    def __init__(self, device, timeStep, pointIsCloseThresh, pointIsCloseRange):
        self.device = device
        self.device.enable(timeStep)
        self.x = 0
        self.y = 0
        self.z = 0
        self.rotation = 0
        self.fov = device.getFov()
        self.verticalFov = self.device.getVerticalFov()
        self.horizontalRes = self.device.getHorizontalResolution()
        self.verticalRes = self.device.getNumberOfLayers()
        self.hRadPerDetection = self.fov / self.horizontalRes
        self.vRadPerDetection = self.verticalFov / self.verticalRes
        self.detectRotOffset = 0  # math.pi * 0.75
        self.maxDetectionDistance = 0.06 * 5
        self.minDetectionDistance = 0.06 * 0.5
        self.pointIsClose = False
        self.pointIsCloseThresh = pointIsCloseThresh
        self.pointIsCloseRange = pointIsCloseRange
        self.distBias = 0.06 * 0.2
        self.distCoeff = 1
        self.distFactor = 1 #0.8
    
    def getRotationsAndDistances(self, layers=range(3)):
        self.pointIsClose = False
        
        # (degsToRads(359 - radsToDegs(self.rotation)))
        # rangeImage = self.device.getRangeImageArray()
        # print("Lidar vFov: ", self.verticalFov/ self.verticalRes)

        rots = []
        distances = []
        
        for layer in layers:
            actualVDetectionRot = (layer * self.vRadPerDetection) + self.verticalFov / 2
            depthArray = self.device.getLayerRangeImage(layer)
            actualHDetectionRot = self.detectRotOffset + ((2 * math.pi) - self.rotation)
            for item in depthArray:
                if self.minDetectionDistance <= item:# <= self.maxDetectionDistance:

                    if item == float("inf") or item == float("inf") * -1:
                        item = 0.5
                    x = item * math.cos(actualVDetectionRot)
                    
                    x += self.distBias
                    x *= self.distCoeff
                    x = x ** self.distFactor


                    if degsToRads(self.pointIsCloseRange[0]) > actualHDetectionRot > degsToRads(self.pointIsCloseRange[1]) and x < self.pointIsCloseThresh:
                        self.pointIsClose = True

                    rots.append(actualHDetectionRot)
                    distances.append(x)
                actualHDetectionRot += self.hRadPerDetection
        return rots, distances


    # Does a detection pass and returns a point cloud with the results
    def getPointCloud(self, layers=range(3)):
        self.pointIsClose = False
        
        # (degsToRads(359 - radsToDegs(self.rotation)))
        # rangeImage = self.device.getRangeImageArray()
        # print("Lidar vFov: ", self.verticalFov/ self.verticalRes)

        pointCloud = []

        outOfBounds = []
        
        for layer in layers:
            actualVDetectionRot = (layer * self.vRadPerDetection) + self.verticalFov / 2
            depthArray = self.device.getLayerRangeImage(layer)
            actualHDetectionRot = self.detectRotOffset + ((2 * math.pi) - self.rotation)
            for item in depthArray:
                if item >= self.maxDetectionDistance or item == float("inf"):
                    x = 10 * math.cos(actualVDetectionRot)
                    x += self.distBias
                    x *= self.distCoeff
                    x = x ** self.distFactor

                    coords = getCoordsFromRads(actualHDetectionRot, x)
                    outOfBounds.append([coords[0] - 0, (coords[1] * -1) - 0])

                else:
                    if item >= self.minDetectionDistance:
                            #item = self.maxDetectionDistance
                            if item != float("inf") and item != float("inf") * -1 and item != 0:
                                x = item * math.cos(actualVDetectionRot)
                                x += self.distBias
                                x *= self.distCoeff
                                x = x ** self.distFactor

                                if degsToRads(self.pointIsCloseRange[0]) > actualHDetectionRot > degsToRads(self.pointIsCloseRange[1]) and x < self.pointIsCloseThresh:
                                    self.pointIsClose = True

                                coords = getCoordsFromRads(actualHDetectionRot, x)
                                pointCloud.append([coords[0] - 0, (coords[1] * -1) - 0])

                actualHDetectionRot += self.hRadPerDetection
        if len(outOfBounds) == 0:
            outOfBounds = [[0, 0]]
        
        if len(pointCloud) == 0:
            pointCloud = [[0, 0]]

        return pointCloud, outOfBounds

    # Sets the rotation of the sensors in radians
    def setRotationRadians(self, rads):
        self.rotation = rads
    
    # Sets the rotation of the sensors in degrees
    def setRotationDegrees(self, degs):
        self.rotation = degsToRads(degs)
##############################################################################################
####################################  GPS  ###################################################
# from devices.gps import Gps

# Tracks global position
class Gps:
    def __init__(self, gps, timeStep, coordsMultiplier=1):
        self.gps = gps
        self.gps.enable(timeStep)
        self.multiplier = coordsMultiplier
        self.__prevPosition = []
        self.position = self.getPosition()

    # updates gps, must run every timestep
    def update(self):
        self.__prevPosition = self.position
        self.position = self.getPosition()

    # Returns the global position
    def getPosition(self):
        vals = self.gps.getValues()
        return [vals[0] * self.multiplier, vals[2] * self.multiplier]

    # Returns the global rotation according to gps
    def getRotation(self):
        if self.__prevPosition != self.position:
            posDiff = ((self.position[0] - self.__prevPosition[0]), (self.position[1] - self.__prevPosition[1]))
            accuracy = getDistance(posDiff)
            if accuracy > 0.001:
                degs = getDegsFromCoords(posDiff)
                return normalizeDegs(degs)
        return None
##############################################################################################
#############################  GYROSCOPE  ####################################################
# from devices.gyroscope import Gyroscope

# Tracks global rotation
class Gyroscope:
    def __init__(self, gyro, index, timeStep):
        self.sensor = gyro
        self.sensor.enable(timeStep)
        self.oldTime = 0.0
        self.index = index
        self.rotation = 0
        self.lastRads = 0

    # Do on every timestep
    def update(self, time):
        timeElapsed = time - self.oldTime  # Time passed in time step
        radsInTimestep = (self.sensor.getValues())[self.index] * timeElapsed
        self.lastRads = radsInTimestep
        finalRot = self.rotation + radsInTimestep
        self.rotation = normalizeRads(finalRot)
        self.oldTime = time

    # Gets the actual angular Velocity
    def getDiff(self):
        if self.lastRads < 0:
            return self.lastRads * -1
        
        return self.lastRads

    # Returns the rotation on degrees
    def getDegrees(self):
        return radsToDegs(self.rotation)

    # Returns the rotation on radians
    def getRadians(self):
        return self.rotation

    # Sets the rotation in radians
    def setRadians(self, rads):
        self.rotation = rads

    # Sets the rotation in degrees
    def setDegrees(self, degs):
        self.rotation = degsToRads(degs)
##############################################################################################
#################################  COMUNICATOR  ##############################################
# from devices.comunicator import Comunicator
import struct

class Comunicator:
    def __init__(self, emmiter, receiver, timeStep):
        self.receiver = receiver
        self.emmiter = emmiter
        self.receiver.enable(timeStep)
        self.lackOfProgress = False
        self.doGetWordInfo = True
        self.gameScore = 0
        self.remainingTime = 0

    def sendVictim(self, position, victimtype):
        self.doGetWordInfo = False
        letter = bytes(victimtype, "utf-8")
        position = multiplyLists(position, [100, 100])
        position = [int(position[0]), int(position[1])]
        message = struct.pack("i i c", position[0], position[1], letter)
        self.emmiter.send(message)

    def sendLackOfProgress(self):
        self.doGetWordInfo = False
        message = struct.pack('c', 'L'.encode())  # message = 'L' to activate lack of progress
        self.emmiter.send(message)

    def sendEndOfPlay(self):
        self.doGetWordInfo = False
        exit_mes = struct.pack('c', b'E')
        self.emmiter.send(exit_mes)
        #print("Ended!!!!!")

    def sendMap(self, npArray):
        # Get shape
        #print(npArray)
        s = npArray.shape
        # Get shape as bytes
        s_bytes = struct.pack('2i', *s)
        # Flattening the matrix and join with ','
        flatMap = ','.join(npArray.flatten())
        # Encode
        sub_bytes = flatMap.encode('utf-8')
        # Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes
        # Send map data
        self.emmiter.send(a_bytes)
        # STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        self.emmiter.send(map_evaluate_request)
        self.doGetWordInfo = False

    def requestGameData(self):
        if self.doGetWordInfo:
            message = struct.pack('c', 'G'.encode())  # message = 'G' for game information
            self.emmiter.send(message)  # send message

    def update(self):

        if self.doGetWordInfo:
            """
            self.requestGameData()
            if self.receiver.getQueueLength() > 0: # If receiver queue is not empty
                receivedData = self.receiver.getData()
                if len(receivedData) > 2:
                    tup = struct.unpack('c f i', receivedData) # Parse data into char, float, int
                    if tup[0].decode("utf-8") == 'G':
                        self.gameScore = tup[1]
                        self.remainingTime = tup[2]
                        self.receiver.nextPacket() # Discard the current data packet
            """

            self.lackOfProgress = False
            if self.receiver.getQueueLength() > 0:  # If receiver queue is not empty
                receivedData = self.receiver.getData()
                #print(receivedData)
                if len(receivedData) < 2:
                    tup = struct.unpack('c', receivedData)  # Parse data into character
                    if tup[0].decode("utf-8") == 'L':  # 'L' means lack of progress occurred
                        #print("Detected Lack of Progress!")
                        self.lackOfProgress = True
                    self.receiver.nextPacket()  # Discard the current data packetelse:
        else:
            self.doGetWordInfo = True

##############################################################################################
##############################################################################################

# Abstraction layer for robot
class RobotLayer:
    def __init__(self, time_step):
        # Maximum wheel speed
        self.max_wheel_speed = 6.28
        # The timestep
        self.time_step = time_step

        self.diameter = 0.074
        # Robot object provided by webots
        self.robot = Robot()
        self.prev_rotation = 0
        self.rotation = 0
        self.position = [0, 0]
        self.prev_global_position = [0, 0]
        self.position_offsets = [0, 0]

        self.rotation_sensor = "gyro"

        self.time = 0
        self.rotate_to_degs_first_time = True
        self.delay_first_time = True
        self.delay_start = self.robot.getTime()

        self.auto_decide_rotation = True
        self.gyroscope = Gyroscope(self.robot.getDevice("gyro"), 1, self.time_step)
        self.gps = Gps(self.robot.getDevice("gps"), self.time_step)
        self.lidar = Lidar(self.robot.getDevice("lidar"), self.time_step, 0.03, (0, 360))
        self.left_wheel = Wheel(self.robot.getDevice("wheel1 motor"), self.max_wheel_speed)
        self.right_wheel = Wheel(self.robot.getDevice("wheel2 motor"), self.max_wheel_speed)

        self.comunicator = Comunicator(self.robot.getDevice("emitter"), self.robot.getDevice("receiver"), self.time_step)
        self.center_camera = Camera(self.robot.getDevice("camera1"), self.time_step)
        self.right_camera = Camera(self.robot.getDevice("camera2"), self.time_step)
        self.left_camera = Camera(self.robot.getDevice("camera3"), self.time_step)
        

        self.point_is_close = False

        self.stuck_counter = 0

    def delay_sec(self, delay):
        #print("Current delay: ", delay)
        if self.delay_first_time:
            self.delay_start = self.robot.getTime()
            self.delay_first_time = False
        else:
            if self.time - self.delay_start >= delay:
                
                self.delay_first_time = True
                return True
        return False

    # Moves the wheels at the specified ratio
    def move_wheels(self, left_ratio, right_ratio):
        self.left_wheel.move(left_ratio)
        self.right_wheel.move(right_ratio)

    def rotate_to_degs(self, degs, orientation="closest", max_speed=0.5):
        accuracy = 2
        if self.rotate_to_degs_first_time:
            # print("STARTED ROTATION")
            self.rotate_to_degs_first_time = False
        self.seqRotateToDegsInitialRot = self.rotation
        self.seqRotateToDegsinitialDiff = round(self.seqRotateToDegsInitialRot - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, accuracy, 90, 0.2, 0.8), max_speed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotate_to_degs_first_time = True
            return True
        else:
            if orientation == "closest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > self.seqRotateToDegsinitialDiff > 0 or self.seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation

            if moveDiff > 10:
                if direction == "right":
                    self.move_wheels(speedFract * -1, speedFract)
                elif direction == "left":
                    self.move_wheels(speedFract, speedFract * -1)
            else:
                if direction == "right":
                    self.move_wheels(speedFract * -0.5, speedFract)
                elif direction == "left":
                    self.move_wheels(speedFract, speedFract * -0.5)
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(self.seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False
    
    def rotate_smoothly_to_degs(self, degs, orientation="closest", maxSpeed=0.5):
        accuracy = 2
        seqRotateToDegsinitialDiff = round(self.rotation - degs)
        diff = self.rotation - degs
        moveDiff = max(round(self.rotation), degs) - min(self.rotation, degs)
        if diff > 180 or diff < -180:
            moveDiff = 360 - moveDiff
        speedFract = min(mapVals(moveDiff, accuracy, 90, 0.2, 0.8), maxSpeed)
        if accuracy * -1 < diff < accuracy or 360 - accuracy < diff < 360 + accuracy:
            self.rotate_to_degs_first_time = True
            return True
        else:
            if orientation == "closest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "right"
                else:
                    direction = "left"
            elif orientation == "farthest":
                if 180 > seqRotateToDegsinitialDiff > 0 or seqRotateToDegsinitialDiff < -180:
                    direction = "left"
                else:
                    direction = "right"
            else:
                direction = orientation
            if direction == "right":
                self.move_wheels(speedFract * -0.5, speedFract)
            elif direction == "left":
                self.move_wheels(speedFract, speedFract * -0.5)
            # print("speed fract: " +  str(speedFract))
            # print("target angle: " +  str(degs))
            # print("moveDiff: " + str(moveDiff))
            # print("diff: " + str(diff))
            # print("orientation: " + str(orientation))
            # print("direction: " + str(direction))
            # print("initialDiff: " + str(seqRotateToDegsinitialDiff))

        # print("ROT IS FALSE")
        return False

    def move_to_coords(self, targetPos):
        errorMargin = 0.01
        descelerationStart = 0.5 * 0.12
        diffX = targetPos[0] - self.position[0]
        diffY = targetPos[1] - self.position[1]
        dist = getDistance((diffX, diffY))
        #print("Dist: "+ str(dist))
        if errorMargin * -1 < dist < errorMargin:
            # self.robot.move(0,0)
            #print("FinisehedMove")
            return True
        else:
            
            ang = getDegsFromCoords((diffX, diffY))
            ang = normalizeDegs(ang)
            # print("traget ang: " + str(ang))
            ratio = min(mapVals(dist, 0, descelerationStart, 0.1, 1), 1)
            ratio = max(ratio, 0.8)
            if self.rotate_to_degs(ang):
                self.move_wheels(ratio, ratio)
                # print("Moving")
        return False
    
    # Gets a point cloud with all the detections from lidar and distance sensorss
    def get_detection_point_cloud(self):
        point_clouds = self.lidar.getPointCloud(layers=(2, 3))
        self.point_is_close = self.lidar.pointIsClose
        return point_clouds
    
    def get_camera_images(self):
        return [self.right_camera.getImg(), self.center_camera.getImg(), self.left_camera.getImg()]
    
    # Returns True if the simulation is running
    def do_loop(self):
        return self.robot.step(self.time_step) != -1
    
    def get_wheel_direction(self):
        if self.right_wheel.velocity + self.left_wheel.velocity == 0:
            return 0
        return (self.right_wheel.velocity + self.left_wheel.velocity) / 2
    
    def is_stuck_this_step(self):
        return self.get_wheel_direction() > 0 and abs(getDistance(substractLists(self.position, self.prev_global_position))) < 0.00001

    def is_stuck(self):
        return self.stuck_counter > 50

    # Must run every TimeStep
    def update(self):
        # Updates the current time
        self.time = self.robot.getTime()
        # Updates the gps, gyroscope
        self.gps.update()
        self.gyroscope.update(self.time)

        # Gets global position
        self.prev_global_position = self.position
        self.position = self.gps.getPosition()
        self.position[0] += self.position_offsets[0]
        self.position[1] += self.position_offsets[1]

        # Decides wich sensor to use for roatation detection
        # if the robot is going srtaight i tuses the gps
        
        if self.auto_decide_rotation:
            if self.gyroscope.getDiff() < 0.00001 and self.get_wheel_direction() >= 0:
                self.rotation_sensor = "gps"
            # if it isn't going straight it uses the gyro
            else:
                self.rotation_sensor = "gyro"

        # Remembers the corrent rotation for the next timestep
        self.prev_rotation = self.rotation

        # Gets global rotation
        if self.rotation_sensor == "gyro":
            self.rotation = self.gyroscope.getDegrees()
            #print("USING GYRO")
        else:
            #print("USING GPS")
            val = self.gps.getRotation()
            if val is not None:
                self.rotation = val
            self.gyroscope.setDegrees(self.rotation)

        # Sets lidar rotation
        self.lidar.setRotationDegrees(self.rotation + 0)

        #print("Delay time:", self.time - self.delayStart)

        if self.is_stuck_this_step():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        self.comunicator.update()



# import mapping
##############################################################################################
###################################  MAPPING  ################################################
# from data_processing import camera_processor
##############################################################################################
#################################  CAMERA PROCESSOR  #########################################
import imutils

class CameraProcessor:
    def __init__(self, tile_resolution):
        self.tile_resolution = tile_resolution # 100

    def sharpen_image(self, image):
        kernel = np.array([[-1,-1,-1], [-1,5,-1], [-1,-1,-1]])
        return cv.filter2D(image, -1, kernel)

    def upscale_image(image, scale):
        return cv.resize(image, (0,0), fx=scale, fy=scale, interpolation=cv.INTER_CUBIC)

    def flatten_image(self, image):
        tiles_up = 2
        tiles_down = 0
        tiles_side = 1

        minimum_x = self.tile_resolution * tiles_side
        maximum_x = self.tile_resolution * (tiles_side + 1)
        minimum_y = self.tile_resolution * (tiles_up)
        maximum_y = self.tile_resolution * (tiles_up  + 1)  - 40

        #robot1_points = np.array(([4, 17], [35, 17],  [31, 12],  [8, 12],), dtype=np.float32)
        img_points = np.array(([5, 6],  [35, 6], [31, 3], [8, 3], ), dtype=np.float32)
        final_points = np.array(([minimum_x, minimum_y],  [maximum_x, minimum_y], [maximum_x, maximum_y], [minimum_x, maximum_y],), dtype=np.float32)

        ipm_matrix = cv.getPerspectiveTransform(img_points, final_points, solveMethod=cv.DECOMP_SVD)
        
        final_x = self.tile_resolution * ((tiles_side * 2) + 1)
        final_y = self.tile_resolution * (tiles_up + 1 + tiles_down)
        
        final_y_modiff = round(final_y * 1)#0.95)

        ipm = cv.warpPerspective(image, ipm_matrix, (final_x, final_y_modiff), flags=cv.INTER_NEAREST)
        ipm = cv.resize(ipm, (final_x, final_y), interpolation=cv.INTER_CUBIC)

        return ipm

    def overlay_image_alpha(self, img, img_overlay, x, y, alpha_mask):
        """Overlay `img_overlay` onto `img` at (x, y) and blend using `alpha_mask`.

        `alpha_mask` must have same HxW as `img_overlay` and values in range [0, 1].
        """
        # Image ranges
        y1, y2 = max(0, y), min(img.shape[0], y + img_overlay.shape[0])
        x1, x2 = max(0, x), min(img.shape[1], x + img_overlay.shape[1])

        # Overlay ranges
        y1o, y2o = max(0, -y), min(img_overlay.shape[0], img.shape[0] - y)
        x1o, x2o = max(0, -x), min(img_overlay.shape[1], img.shape[1] - x)

        # Exit if nothing to do
        if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
            return

        # Blend overlay within the determined ranges
        img_crop = img[y1:y2, x1:x2]
        img_overlay_crop = img_overlay[y1o:y2o, x1o:x2o]
        alpha = alpha_mask[y1o:y2o, x1o:x2o, np.newaxis]
        alpha_inv = 1.0 - alpha

        img_crop[:] = img_overlay_crop * alpha + img_crop * alpha_inv

    def join_camera_images(self, images, translations):
        max_x = 0
        max_y = 0

        for translation in translations:
            max_x = max(max_x, translation[1])
            max_y = max(max_y, translation[0])

        backround = np.zeros((max_x + images[0].shape[1], max_y + images[0].shape[0], 3), dtype=np.uint8)
        
        rot_imgs = []
        for index, img in enumerate(images):
            rot_imgs.append(np.rot90(img, index + 2, (0,1)))

        for rot_img, translation in zip(rot_imgs, translations):
            self.overlay_image_alpha(backround, rot_img[:,:,:3], translation[0], translation[1], rot_img[:,:,3] / 255)
        
        return backround.copy()

    def rotate_image(self, image, robot_rotation):
        rot = normalizeDegs(robot_rotation)
        return imutils.rotate(image, rot)

    def get_floor_image(self, images, robot_rotation):
        flattened_images = []
        for img in images:
            img = np.rot90(img, 3, (0,1))
            img = self.flatten_image(img)
            img = np.flip(img, 0)
            flattened_images.append(img)

        x_red = 5
        y_red = 2

        translations = [[200 - y_red, 400 - x_red], [400 - x_red, 200 + y_red], [200 + y_red, 0 + x_red]]
        camera_final_image = self.join_camera_images(flattened_images, translations)

        return self.rotate_image(camera_final_image, robot_rotation - 90)

if __name__ == "__main__":
    pass



# from data_processing import data_extractor
##############################################################################################
##############################  DATA EXTRACTOR  ##############################################
class FloorColorExtractor:
    def __init__(self, tile_resolution) -> None:
        self.tile_resolution = tile_resolution
        self.floor_color_ranges = {
                    "normal":
                        {   
                            "range":   ((0, 0, 37), (0, 0, 192)), 
                            "threshold":0.2},

                    "nothing":
                        {
                            "range":((100, 0, 0), (101, 1, 1)),
                            "threshold":0.9},
                    
                    "checkpoint":
                        {
                            "range":((95, 0, 65), (128, 122, 198)),
                            "threshold":0.2},
                    "hole":
                        {
                            "range":((0, 0, 10), (0, 0, 30)),
                            "threshold":0.2},
                    
                    "swamp":
                        {
                            "range":((19, 112, 32), (19, 141, 166)),
                            "threshold":0.2},

                    "connection1-2":
                        {
                            "range":((120, 182, 49), (120, 204, 232)),
                            "threshold":0.2},

                    "connection1-3":
                        {
                            "range":((132, 156, 36), (133, 192, 185)),
                            "threshold":0.2},

                    "connection2-3":
                        {
                            "range":((0, 182, 49), (0, 204, 232)),
                            "threshold":0.2},
                    }
        self.final_image = np.zeros((700, 700, 3), np.uint8)
        
    def get_square_color(self, image, square_points):
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        square = cv.cvtColor(square, cv.COLOR_BGR2HSV)
        if np.count_nonzero(square) == 0:
            return "nothing"
        color_counts = {}
        for color_key, color_range in self.floor_color_ranges.items():
            colour_count = np.count_nonzero(cv.inRange(square, color_range["range"][0], color_range["range"][1]))
            if colour_count > color_range["threshold"] * square.shape[0] * square.shape[1]:
                color_counts[color_key] = colour_count
        
        if len(color_counts) == 0:
            return "nothing"
        else:
            return max(color_counts, key=color_counts.get)
    
    def get_sq_color(self, image, square_points):
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        # remove pixels with value 0, 0, 0
        white_count = np.count_nonzero(cv.inRange(square, (180, 180, 180), (255, 255, 255)))
        black_count = np.count_nonzero(cv.inRange(square, (20, 20, 20), (180, 180, 180)))

        if white_count > black_count and white_count > square.shape[0] * square.shape[1] / 8:
            return (255, 255, 255)
        else:
            return (100, 100, 100)

    def get_floor_colors(self, floor_image, robot_position):

        grid_offsets = [(((p + 0) % 0.06) / 0.06) * 50 for p in robot_position]
        
        grid_offsets = [int(o) for o in grid_offsets]

        offsets = [((((p + 0.03) % 0.06) - 0.03) / 0.06) * 50 for p in robot_position]
        
        offsets = [int(o) for o in offsets]

        
        save_image(floor_image, "floor_image.png")

        squares_grid = get_squares(floor_image, self.tile_resolution, offsets)

        color_tiles = []
        for row in squares_grid:
            for square in row:
                color_key = self.get_square_color(floor_image, square)
                if color_key == "normal":
                    color = (255, 255, 255)
                elif color_key == "checkpoint":
                    color = (100, 100, 100)
                else:
                    color = (0, 0, 0)
                #if color != (0, 0, 0):
                #cv.rectangle(self.final_image, [square[2], square[0]], [square[3], square[1]], color, -1)

                tile = [square[2], square[0]]
                tile = substractLists(tile, (350 - offsets[0], 350 - offsets[1]))
                tile = divideLists(tile, [self.tile_resolution, self.tile_resolution])
                tile = [int(t) for t in tile]
                if color_key != "nothing":
                    #print(tile, color_key)
                    color_tiles.append((tile, color_key))

        drawing_image = floor_image.copy() #self.final_image.copy()
        draw_grid(drawing_image, self.tile_resolution, offset=grid_offsets)
        cv.circle(drawing_image, (350 - offsets[0], 350 - offsets[1]), 10, (255, 0, 0), -1)
        cv.imshow("final_floor_image", resize_image_to_fixed_size(drawing_image, (600, 600)))        
        return color_tiles

class PointCloudExtarctor:
    def __init__(self, resolution):
        self.threshold = 8
        self.resolution = resolution
        self.straight_template = np.zeros((self.resolution + 1, self.resolution + 1), dtype=np.int)
        self.straight_template[:][0:2] = 1
        #self.straight_template[3:-3][0:2] = 2
        self.straight_template[0][0:2] = 0
        self.straight_template[-1][0:2] = 0

        straight = [
            [0, 1, 2, 2, 2, 1, 0],
            [0, 1, 2, 2, 2, 1, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.straight_template = np.array(straight)

        curved = [
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 1, 1, 0],
            [0, 0, 3, 1, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [1, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]
        
        self.curved_template = np.array(curved)


        self.templates = {}

        for i, name in enumerate([("u",), ("l",), ("d",), ("r",)]):
            self.templates[name] = np.rot90(self.straight_template, i)
        
        for i, name in enumerate([("u", "l"), ("d", "l"), ("d", "r"),  ("u", "r")]):
            self.templates[name] = np.rot90(self.curved_template, i)

    def get_tile_status(self, min_x, min_y, max_x, max_y, point_cloud):
        counts = {}
        for name in self.templates:
            counts[name] = 0
        square = point_cloud[min_x:max_x+1, min_y:max_y+1]
        #print(square.shape)
        
        if square.shape != (self.resolution+1, self.resolution+1):
            return []
        for name, template in self.templates.items():
            for i in range(self.resolution + 1):
                for j in range(self.resolution + 1):
                    if square[i][j] == 1:
                        counts[name] += copy.copy(template[i][j])
        
        names = []
        for name, count in counts.items():
            if count >= self.threshold:
                names += list(name)
        """
        if np.count_nonzero(square):
            cv.imshow(f"square" + str(names), (square*255).astype(np.uint8))
        """
        return names

    def transform_to_grid(self, point_cloud):
        offsets = point_cloud.offsets
        offsets = [o % self.resolution for o in offsets]
        offsets.reverse()
        grid = []
        bool_array_copy = point_cloud.get_bool_array()
        bool_array_copy = bool_array_copy.astype(np.uint8) * 100
        for x in range(offsets[0], bool_array_copy.shape[0] - self.resolution, self.resolution):
            row = []
            for y in range(offsets[1], bool_array_copy.shape[1] - self.resolution, self.resolution):
                min_x = x
                min_y = y
                max_x = x + self.resolution
                max_y = y + self.resolution
                #print(min_x, min_y, max_x, max_y)
                bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)
                
                val = self.get_tile_status(min_x, min_y, max_x, max_y, point_cloud.get_bool_array())
                
                row.append(list(val))
            grid.append(row)
        factor = 10
        cv.imshow("point_cloud_with_squares", resize_image_to_fixed_size(bool_array_copy, (600, 600)))
        offsets = point_cloud.offsets
        return grid, [o // self.resolution for o in offsets]

# from data_procesing import point_cloud_processor
##############################################################################################
#############################  POINT CLOUD PROCESSOR  ########################################
from bresenham import bresenham
from skimage.draw import line

class PointCloudProcessor:
    def __init__(self, center_point, map_scale):
        self.map_scale = map_scale # 850
        self.center_point = [center_point, center_point] #350

    def processPointCloud(self, pc, robotPos):
        return [[pcv + rpv for pcv, rpv in zip(pos, robotPos)] for pos in pc]

    def processPointCloudForCamera(self, pc):
        if len(pc) == 0:
            return np.empty((0, 2), dtype=np.int)
        return np.array([[int(p * self.map_scale) + c for c, p in zip(self.center_point,  pos)] for pos in pc], dtype=np.int)


    def get_intermediate_points(self, point_cloud):
        seen_x = np.empty(0, dtype=np.int)
        seen_y = np.empty(0, dtype=np.int)
        for point in point_cloud:
            xx, yy = line(self.center_point[0], self.center_point[1], point[0], point[1]) #list(bresenham(self.center_point[0], self.center_point[1], point[0], point[1]))
            seen_x = np.hstack((seen_x, xx))
            seen_y = np.hstack((seen_y, yy))
        return seen_x, seen_y


if __name__ == '__main__':
    pass


# from data_structures import lidar_persistent_grid
##############################################################################################
###########################  LIDAR PERSISTENT GRID  ##########################################

# from data_structures import resizable_pixel_grid
##############################################################################################
############################  RESIZABLE PIXEL GRID  ##########################################
class Grid:
    def __init__(self, initial_shape, res=100):
        self.offsets = [initial_shape[0] // 2, initial_shape[1] // 2]
        self.resolution = res
        self.shape = initial_shape
        self.grid = np.zeros(self.shape, dtype=np.int)

        self.value_divider = 1023 / 255
        self.value_limit = 1023

    def expand_grid_to_point(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]

        if y + 1 > self.shape[0]:
            self.add_end_row(y - self.shape[0] +1)
        if x + 1 > self.shape[1]:
            self.add_end_column(x - self.shape[1] +1)
        if y < 0:
            self.add_begining_row(-y)
        if x < 0:
            self.add_begining_column(-x)
    
    
    def add_point(self, point, value=255):
        self.expand_grid_to_point(point)
        
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
    
        self.grid[y, x] = value
    
    def sum_to_point(self, point, value):
        self.expand_grid_to_point(point)
        
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
    
        self.grid[y, x] = min(self.grid[y, x] + value, self.value_limit)
    
    def get_point(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
        return self.grid[y, x]
     
    def add_end_row(self, size):
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((self.grid, np.zeros((size, self.shape[1]), dtype=np.int)))
    
    def add_begining_row(self, size):
        self.offsets[1] += size
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((np.zeros((size, self.shape[1]), dtype=np.int), self.grid))
    
    def add_end_column(self, size):
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((self.grid, np.zeros((self.shape[0], size), dtype=np.int)))

    def add_begining_column(self, size):
        self.offsets[0] += size
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((np.zeros((self.shape[0], size), dtype=np.int), self.grid))

    def print_grid(self, max_size=(2000, 1000)):
        grid1 = copy.deepcopy(self.grid)
       
        grid1 = grid1 // self.value_divider

        grid1 = resize_image_to_fixed_size(grid1, max_size)

        cv.imshow("grid", grid1.astype(np.uint8))
        cv.waitKey(1)

if __name__ == "__main__":
    my_grid = Grid((5, 5))

    my_grid.add_point((2, 2))
    print(my_grid.offsets)

    my_grid.print_grid()

    my_grid.add_point((-5, -10))
    print(my_grid.offsets)

    my_grid.print_grid()


class LidarGrid(Grid):
    def __init__(self, input_resolution, resolution, threshold=0):
        self.input_res = input_resolution
        self.res = resolution
        self.multiplier = self.res / self.input_res
        self.frame = 0
        
        self.shape = (self.res, self.res)
        super().__init__(self.shape, self.res)
        self.threshold = 100
        self.delete_threshold = 2

    def get_bool_array(self):
        return self.grid > self.threshold
    
    def clean_up(self):
       # print("Cleaning up lidar grid")
        self.grid = self.grid * (self.grid > self.delete_threshold).astype(np.int)

    
    def sum_detection(self, point):
        point = [round(p * self.multiplier) for p in point]
        self.sum_to_point(point, 1)
    
    def print_bool(self, max_size=(600, 600)):
        grid1 = resize_image_to_fixed_size(self.get_bool_array(), max_size)
        cv.imshow("bool_grid", grid1 * 255)
        cv.waitKey(1)
    
    def update(self, point_cloud):
        self.clean_up()
        for point in point_cloud:
            self.sum_detection(point)
        self.frame += 1

# from data_structures import expandable_node_grid
##############################################################################################
###########################  EXPANDABLE NODE GRID  ###########################################
"""
Requirements:

1 = Node type (tile, vortex, wall)
2 = Status (ocupied, not_occupied, undefined)
3 = Tile type (only if tile: undefined, start, normal, connection1-2, connection1-3, connection2-3, swamp, hole)

undefined = no conozco el tipo de casilla
"""
class Fixture:
    def __init__(self, exists=False, reported=False, type="N") -> None:
        self.exists = exists
        self.reported = reported
        self.type = type
        self.detection_angle = None

class Node:
    def __init__(self, node_type:str, status:str="undefined", tile_type:str="undefined", curved:int=0, explored:bool=False, is_robots_position:bool=False):
        self.node_type = node_type
        self.status = status
        self.tile_type = tile_type if node_type == "tile" else "undefined"
        self.explored = explored
        self.is_robots_position = is_robots_position
        self.fixture = Fixture()
        self.fixtures_in_wall = []
        self.is_start = False
        self.is_curved = False

        self.mark1 = 0
        self.mark2 = 0
        
        self.valid_node_type = ("tile", 
                                "vortex",
                                "wall") #tuple with valid values for the variables of the node
        
        self.valid_status = (   "occupied",
                                "undefined",
                                "not_occupied") #same tuple

        self.valid_tile_types = ("undefined",
                                "normal",
                                "start",
                                "connection1-2",
                                "connection1-3", 
                                "connection2-3", 
                                "swamp", 
                                "hole",
                                "checkpoint") #same tuple

        self.tile_type_to_string = {
            "undefined": "0",
            "normal": "0",
            "checkpoint": "4",
            "start": "5",
            "connection1-2": "6",
            "connection1-3": "7",
            "connection2-3": "8",
            "swamp": "3",
            "hole": "2"
        }
       


    def get_representation(self) -> str:
        if self.node_type == "tile":
            return self.tile_type_to_string[self.tile_type]

        elif self.node_type == "vortex":
            return str(int(self.status == "occupied" and not self.is_curved))
        
        elif self.node_type == "wall":
            if len(self.fixtures_in_wall) > 0:
                return str("".join(self.fixtures_in_wall))
            return str(int(self.status == "occupied"))
        
        else:
            return "0"

    # Returns a visual representation of the node in ASCII 
    def get_string(self):

        if self.mark1:
            return "\033[1;36;36m██" + "\033[0m"

        if self.status == "undefined":
            if not(self.node_type == "tile" and self.tile_type != "undefined"):    
                return "??"
        

        if self.status == "occupied":
            if self.node_type == "vortex" and self.explored:
                return "\033[1;31;47m██"+ "\033[0m"

            if self.node_type == "wall" and len(self.fixtures_in_wall) > 0:
                return f"\033[1;35;40m{self.fixtures_in_wall[0]*2}" + "\033[0m"
            
            return "\033[1;30;40m██" + "\033[0m"
        
        
        #elif self.is_robots_position:
        #    return "\033[1;32;47m██"+ "\033[0m"
        
        elif self.node_type == "wall":
            """
            if self.status == "not_occupied":
                return "\033[1;37;47m██"+ "\033[0m"
            """
            return "\033[1;30;47m||"+ "\033[0m"
        elif self.node_type == "vortex": #vertice
            """
            if self.status == "not_occupied":
                return "\033[1;37;47m██"+ "\033[0m"
            """
            if self.explored:
                return "\033[1;32;47m██"+ "\033[0m"

            return "\033[1;30;47m<>"+ "\033[0m"
        
        elif self.node_type == "tile":

            if self.explored:
                if self.tile_type == "start":
                    return "\033[1;32;47m█E"+ "\033[0m"
                if self.tile_type == "hole":
                    return "\033[0m  "+ "\033[0m"
                if self.tile_type == "swamp":
                    return "\033[1;33;40m█E"+ "\033[0m"
                if self.tile_type == "checkpoint":
                    return "\033[0m█E"+ "\033[0m"
                if self.tile_type == "connection1-3":
                    return "\033[1;35;47m█E"+ "\033[0m"
                if self.tile_type == "connection1-2":
                    return "\033[1;34;47m█E"+ "\033[0m"
                if self.tile_type == "connection2-3":
                    return "\033[1;31;47m█E"+ "\033[0m"
                if self.tile_type == "normal":
                    return "\033[1;37;47m█E"+ "\033[0m"
            
            if self.tile_type == "start":
                return "\033[1;32;47m██"+ "\033[0m"
            if self.tile_type == "hole":
                return "\033[0m  "+ "\033[0m"
            if self.tile_type == "swamp":
                return "\033[1;33;40m██"+ "\033[0m"
            if self.tile_type == "checkpoint":
                return "\033[0m██"+ "\033[0m"
            if self.tile_type == "connection1-3":
                return "\033[1;35;47m██"+ "\033[0m"
            if self.tile_type == "connection1-2":
                return "\033[1;34;47m██"+ "\033[0m"
            if self.tile_type == "connection2-3":
                return "\033[1;31;47m██"+ "\033[0m"
            if self.tile_type == "normal":
                return "\033[1;37;47m██"+ "\033[0m"

            return "\033[1;30;47m??"+ "\033[0m"
            
        
        
    def __str__(self) -> str:
        return self.get_string()

    def __repr__(self) -> str:
        return self.get_string()

class Grid:
    def __init__(self, initial_shape):
        self.offsets = [initial_shape[0] // 2, initial_shape[1] // 2]
        self.shape = initial_shape
        self.grid = np.empty(initial_shape, dtype=object)
        self.fill_nodes(self.grid)
        self.fill_node_types()
    
    def get_node_type(self, point):
        x_div = point[0] % 2 == 0
        y_div = point[1] % 2 == 0
        if x_div and y_div:
            return "vortex"
        elif x_div and not y_div:
            return "wall"
        elif not x_div and y_div:
            return "wall"
        elif not x_div and not y_div:
            return "tile"
    
    def get_type_node_poses(self, node_type):
        grid = []
        for y, row in enumerate(self.grid):
            new_row = []
            for x, node in enumerate(row):
                if node.node_type == node_type:
                    new_row.append((x, y))
            grid.append(row)
        return grid
    
    def fill_nodes(self, grid):
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                grid[i, j] = Node("undefined")

    def fill_node_types(self, x_min=0, y_min=0, x_max=None, y_max=None):
        if x_max is None:
            x_max = self.shape[1]
        if y_max is None:
            y_max = self.shape[0]
        
        for y in range(y_min, y_max):
            for x in range(x_min, x_max):
                self.grid[y, x].node_type = self.get_node_type((x + self.offsets[0], y + self.offsets[1]))
                self.grid[y, x].status = "not_occupied"

    def add_end_row(self, size):
        self.shape = (self.shape[0]+ size, self.shape[1] )
        row = np.empty((size, self.shape[1]), dtype=object)
        self.fill_nodes(row)
        self.grid = np.vstack((self.grid, row))
        self.fill_node_types(y_min=self.shape[0]-size)
    
    def add_begining_row(self, size):
        self.offsets[1] += size
        self.shape = (self.shape[0]+ size, self.shape[1] )
        row = np.empty((size, self.shape[1]), dtype=object)
        self.fill_nodes(row)
        self.grid = np.vstack((row, self.grid))
        self.fill_node_types(y_max=size)
    
    def add_end_column(self, size):
        self.shape = (self.shape[0], self.shape[1] + size)
        column = np.empty((self.shape[0], size), dtype=object)
        self.fill_nodes(column)
        self.grid = np.hstack((self.grid, column))
        self.fill_node_types(x_min=self.shape[1]-size)

    def add_begining_column(self, size):
        self.offsets[0] += size
        self.shape = (self.shape[0], self.shape[1] + size)
        column = np.empty((self.shape[0], size), dtype=object)
        self.fill_nodes(column)
        self.grid = np.hstack((column, self.grid))
        self.fill_node_types(x_max=size)

    def expand_grid_to_point(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]

        if y + 1 > self.shape[0]:
            self.add_end_row(y - self.shape[0] +1)
        if x + 1 > self.shape[1]:
            self.add_end_column(x - self.shape[1] +1)
        if y < 0:
            self.add_begining_row(-y)
        if x < 0:
            self.add_begining_column(-x)
    
    def get_node(self, point, expand=True, phantom=False):
        if expand:
            self.expand_grid_to_point(point)
        elif not self.is_in_grid(point):
            if phantom:
                return Node(self.get_node_type(point))
            return None
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
        return self.grid[y, x]
    
    def is_in_grid(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
        return 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]

    def fill_verticies_around_wall(self, wall_node):
        assert self.get_node(wall_node).node_type == "wall"
        for a in ([-1, 0], [1, 0], [0, -1], [0, 1]):
            x, y = wall_node
            x += a[1]
            y += a[0]
            node = self.get_node((x, y))
            if node.node_type == "vortex":
                node.status = "occupied"

    def load_straight_wall(self, tile, direction):
        assert self.get_node(tile).node_type == "tile"
        list_direction = dir2list(direction)
        wall = [t + d for t, d in zip(tile, list_direction)]

        self.get_node((wall[0], wall[1])).status = "occupied"
        self.fill_verticies_around_wall((wall[0], wall[1]))


    def print_grid(self):
        for row in self.grid:
            for node in row:
                print(node, end="")
    
if __name__ == "__main__":
    grid = Grid((10, 10))
    
    grid.get_node((0, 0)).status = "occupied"
    grid.get_node((-5, -5)).status = "occupied"
    grid.get_node((-5, 4)).status = "occupied"
    grid.get_node((4, -5)).status = "occupied"
    grid.get_node((4, 4)).status = "occupied"
    grid.print_grid()

    """
    grid.add_end_row(2)
    grid.print_grid()
    grid.add_begining_row(2)
    grid.print_grid()
    grid.add_end_column(2)
    grid.print_grid()
    grid.add_begining_column(2)
    grid.print_grid()
    """

     
    



# from algorithms.expandable_node_grid.bfs import bfs
##############################################################################################
###################################  BFS  ####################################################

# from algorithms.expandable_node_grid.traversable import is_traversable
##############################################################################################
##############################  IS TRAVERSABLE  ##############################################

def is_traversable(grid, index):
    node = grid.get_node(index, expand=False, phantom=True)
    if node.node_type == "vortex":
        if node.explored:
            return True
        if node.status == "occupied":
            return False
        traversable = True
        for adjacentIndex in ((-1, 1), (1, -1), (1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)):
            adjacent = grid.get_node((index[0] + adjacentIndex[0], index[1] + adjacentIndex[1]), expand=False, phantom=True)
            
            if adjacent.node_type == "tile":
                if adjacent.tile_type == "hole" or adjacent.status == "occupied":
                    traversable = False
                    
            elif adjacent.node_type == "wall":
                if adjacent.status == "occupied":
                    traversable = False
            else:
                raise ValueError((f"invalid instance: {node.node_type}"))
        return traversable
    else:
        raise ValueError((f"invalid instance: {node.node_type}"))

def is_bfs_addable(grid, index):
    node = grid.get_node(index, expand=False, phantom=True)
    if node.node_type == "vortex":
        for adj in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            adjacent = [index[0] + adj[0], index[1] + adj[1]]
            if not grid.get_node(adjacent, expand=False, phantom=True).explored:
                return True
        return False
    else:
        return False

# Breath First Search algorithm
# Returns the tiles in order and with the distance of each one
def bfs(grid, start, limit="undefined"):
    visited = []
    queue = []
    found = []
    start = [start[0], start[1], 0]
    visited.append(start)
    queue.append(start)
    while queue:
        if len(found) > 100:
            break
        coords = queue.pop(0)
        y = coords[1]
        x = coords[0]
        dist = coords[2]
        if limit != "undefined":
            if dist > limit:
                break
        
        if is_bfs_addable(grid, coords[:2]):
            found.append(coords)

        for newPosition in ((0, 1), (0, -1), (-1, 0), (1, 0)):
            neighbour = [x + newPosition[0] * 2, y + newPosition[1] * 2, dist + 1]
            inList = False
            for node in visited:
                if node[0] == neighbour[0] and node[1] == neighbour[1]:
                    inList = True
                    break
            if inList:
                continue

            # Make sure walkable terrain
            if is_traversable(grid, neighbour[:2]):
                visited.append(neighbour)
                queue.append(neighbour)

    return found




class Mapper:
    def __init__(self, tile_size):
        self.tile_size = tile_size

        # Data structures
        self.node_grid = Grid((1, 1))
        self.lidar_grid = LidarGrid(tile_size, 6, 100)

        res = 50 / tile_size

        # Data processors
        self.point_cloud_processor = PointCloudProcessor(350, res)
        self.camera_processor = CameraProcessor(100)

        # Data extractors
        self.point_cloud_extractor = PointCloudExtarctor(resolution=6)
        self.floor_color_extractor = FloorColorExtractor(50)

        self.robot_node = None
        self.robot_vortex_center = None
    
    def register_start(self, robot_position):
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        robot_node = [int(t * 2) for t in robot_vortex]
        for adj in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            adj = sumLists(robot_node, adj)
            self.node_grid.get_node(adj).tile_type = "start"
        self.node_grid.get_node(robot_node).is_start = True
    
    def load_point_cloud(self, point_cloud, robot_position):
        point_cloud = self.point_cloud_processor.processPointCloud(point_cloud, robot_position)
        self.lidar_grid.update(point_cloud)
    
    def lidar_to_node_grid(self):
        grid, offsets = self.point_cloud_extractor.transform_to_grid(self.lidar_grid)
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                xx = (x - offsets[0]) * 2 + 1
                yy = (y - offsets[1]) * 2 + 1
                for direction in value:
                    self.node_grid.load_straight_wall((xx, yy),  direction)

    @do_every_n_frames(5, 32)
    def process_floor(self, camera_images, total_point_cloud, robot_position, robot_rotation):
        floor_image = self.camera_processor.get_floor_image(camera_images, robot_rotation)
        final_image = np.zeros(floor_image.shape, dtype=np.uint8)

        ranged_floor_image = cv.inRange(cv.cvtColor(floor_image, cv.COLOR_BGR2HSV), (0, 0, 100), (1, 1, 255))
        #cv.imshow("floor_image", floor_image)

        self.point_cloud_processor.center_point = (floor_image.shape[1] // 2, floor_image.shape[0] // 2)

       
        camera_point_cloud = self.point_cloud_processor.processPointCloudForCamera(total_point_cloud)
        seen_points = self.point_cloud_processor.get_intermediate_points(camera_point_cloud)
        #print(seen_points)


        draw_poses(final_image, seen_points, back_image=floor_image, xx_yy_format=True)
        
        
        floor_colors = self.floor_color_extractor.get_floor_colors(final_image, robot_position)

        #robot_tile = divideLists(robot_position, [self.tile_size, self.tile_size])
        robot_tile = [round((x + 0.03) / self.tile_size - 0.5) for x in robot_position]
        robot_node = [t * 2 for t in robot_tile]
       

        
        for floor_color in floor_colors:
            tile = floor_color[0]
            color = floor_color[1]
            tile = multiplyLists(tile, [2, 2])
            #tile.reverse()
            tile = sumLists(tile, [1, 1])
            tile = sumLists(tile, robot_node)
            #print(self.node_grid.get_node(tile).node_type)
            if self.node_grid.get_node(tile).tile_type != "start":
                self.node_grid.get_node(tile).tile_type = color
        
        
        #cv.imshow('final_image', resize_image_to_fixed_size(final_image, (600, 600)))
          
        #self.lidar_grid.print_grid((600, 600))
        #self.lidar_grid.print_bool((600, 600))
    
    def degs_to_orientation(self, degs):
        """divides degrees in up, left, right or down"""
        if normalizeDegs(180 - 45) < degs < 180:
            return "up", "right"
        if 180 <= degs < normalizeDegs(180 + 45):
            return "up", "left"

        elif normalizeDegs(360 - 45) < degs <= 360:
            return "down", "left"
        elif 0 <= degs < normalizeDegs(0 + 45):
            return "down", "right" 
        
        elif normalizeDegs(90 - 45) < degs < 90:
            return "right", "down"
        elif 90 <= degs < normalizeDegs(90 + 45):
            return "right", "up"
        
        elif normalizeDegs(270 - 45) < degs < 270:
            return "left", "up"
        elif 270 <= degs < normalizeDegs(270 + 45):
            return "left", "down"

    
    def load_wall_fixture(self, letter, image_angle):
        #print("images_angle:", image_angle)
        orient = self.degs_to_orientation(normalizeDegs(image_angle))
       # print("orientation:", orient)
        dir1, dir2 = orient
        direction = dir2list(dir1)
        direction = multiplyLists(direction, [2, 2])
        direction = sumLists(direction, dir2list(dir2))
        wall_index = sumLists(self.robot_node, direction)
        assert self.node_grid.get_node(wall_index).node_type == "wall"
        self.node_grid.get_node(wall_index).fixtures_in_wall.append(letter)

        

    def load_fixture(self, letter, camera_angle, robot_rotation):
        fixture = self.node_grid.get_node(self.robot_node).fixture
        fixture.exists = True
        fixture.type = letter

        image_angle = normalizeDegs(camera_angle + robot_rotation)
        fixture.detection_angle = image_angle
    
    def get_fixture(self):
        return self.node_grid.get_node(self.robot_node).fixture

    def set_robot_node(self, robot_position):
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        self.robot_vortex_center = [rt * self.tile_size for rt in robot_vortex]
        robot_node = [int(t * 2) for t in robot_vortex]
        self.robot_node = robot_node
        for row in self.node_grid.grid:
            for node in row:
                node.is_robots_position = False

        self.node_grid.get_node(self.robot_node).is_robots_position = True
    
    def block_front_vortex(self, robot_rotation):
        orientation = dir2list(self.degs_to_orientation(robot_rotation)[0])

        front_node = [r + (f * 2) for r, f in zip(self.robot_node, orientation)]
        self.node_grid.get_node(front_node).status = "occupied"


    def update(self, point_cloud=None, camera_images=None, robot_position=None, robot_rotation=None, current_time=None):
        if robot_position is None or robot_rotation is None:
            return
        
        robot_vortex = [int((x + 0.03) // self.tile_size) for x in robot_position]
        robot_node = [int(t * 2) for t in robot_vortex]
        robot_vortex_center = [rt * self.tile_size for rt in robot_vortex]
        
        distance = math.sqrt(sum([(x - y) ** 2 for x, y in zip(robot_vortex_center, robot_position)]))

        #print("robot_vortex:", robot_vortex)

        if self.robot_node is None:
            self.set_robot_node(robot_position)
        
        if distance < 0.02:
            for adj in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
                adj_node = sumLists(robot_node, adj)
                self.node_grid.get_node(adj_node).explored = True
        if distance < 0.02:
            self.node_grid.get_node(robot_node).explored = True

        if point_cloud is not None:
            in_bounds_point_cloud, out_of_bounds_point_cloud = point_cloud
            self.load_point_cloud(in_bounds_point_cloud, robot_position)
            self.lidar_to_node_grid()

        if point_cloud is not None and camera_images is not None and current_time is not None:
            total_point_cloud = np.vstack((in_bounds_point_cloud, out_of_bounds_point_cloud))
            self.process_floor(current_time, camera_images, total_point_cloud, robot_position, robot_rotation)
            self.lidar_grid.print_grid((600, 600))
            self.lidar_grid.print_bool((600, 600))  

            #self.node_grid.print_grid()
        
        cv.waitKey(1) 
            
    
    def get_node_grid(self):
        return copy.deepcopy(self.node_grid)
    
    def get_grid_for_bonus(self):
        final_grid = []
        for row in self.get_node_grid().grid:
            final_row = []
            for node in row:
                final_row.append(node.get_representation())
            final_grid.append(final_row)
        return np.array(final_grid)



#from algorithms.expandable_node_grid.bfs import bfs
##############################################################################################
######################################  BFS  ########################################################


#from agents.closest_position_agent.closest_position_agent import ClosestPositionAgent
##############################################################################################
###############################  CLOSET POSITION AGENT  ######################################
# from agents.agent import Agent
##############################################################################################
################################  AGENT  #####################################################
class Agent:
    def __init__(self, possible_actions=[]) -> None:
        self.possible_actions = possible_actions

    def predict(self, state: list) -> list:
        raise NotImplementedError

    def get_action(self, state: list) -> str:
        return self.predict(state)


class RandomAgent(Agent):
    def __init__(self, possible_actions=[]) -> None:
        super().__init__(possible_actions)

    def predict(self, state: list) -> str:
        return random.choice(self.possible_actions)

# from algorithms.expandable_node_grid.a_star import a_star
##############################################################################################
##################################  A STAR  ##################################################

# aStarNode class for A* pathfinding (Not to be confused with the node grid)
class aStarNode():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# Returns a list of tuples as a path from the given start to the given end in the given maze
def a_star(grid, start, end):
    #assert is_traversable(grid, start)

    assert is_traversable(grid, end)

    # Create start and end node
    startNode = aStarNode(None, (start[0], start[1]))
    startNode.g = startNode.h = startNode.f = 0
    endNode = aStarNode(None, (end[0], end[1]))
    endNode.g = endNode.h = endNode.f = 0
    # Initialize open and closed list
    openList = []
    closedList = []
    # Add the start node
    openList.append(startNode)
    # Loop until end
    while len(openList) > 0:
        # Get the current node
        currentNode = openList[0]
        currentIndex = 0
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                currentNode = item
                currentIndex = index
        # Pop current off open list, add to closed list
        openList.pop(currentIndex)
        closedList.append(currentNode)
        # If found the goal
        if currentNode == endNode:
            path = []
            current = currentNode
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path
        # Generate children
        children = []
        for newPosition in ((0, 1), (0, -1), (-1, 0), (1, 0)):  # Adjacent squares
            # Get node position
            nodePosition = (currentNode.position[0] + (newPosition[0] * 2), currentNode.position[1] + (newPosition[1] * 2))
            # Make sure walkable terrain
            if not is_traversable(grid, nodePosition):
                continue
            # Create new node
            newNode = aStarNode(currentNode, nodePosition)
            # Append
            children.append(newNode)
        # Loop through children
        for child in children:
            continueLoop = False
            # Child is on the closed list
            for closedChild in closedList:
                if child == closedChild:
                    continueLoop = True
                    break
            # Create the f, g, and h values
            child.g = currentNode.g + 1
            child.h = ((child.position[0] - endNode.position[0]) ** 2) + (
                        (child.position[1] - endNode.position[1]) ** 2)
            child.f = child.g + child.h
            # Child is already in the open list
            for openNode in openList:
                if child == openNode and child.g > openNode.g:
                    continueLoop = True
                    break
            if continueLoop:
                continue
            # Add the child to the open list
            openList.append(child)


class ClosestPositionAgent(Agent):
    def __init__(self):
        super().__init__(["up", "down", "left", "right"])
        self.current_robot_node = self.previous_robot_node = None
        self.best_node = [None, None]
        self.a_star_path = []
        self.a_star_index = 0

    def find_robot_node(self, grid):
        for y, row in enumerate(grid.grid):
            for x, node in enumerate(row):
                if node.is_robots_position:
                    return [x - grid.offsets[0], y - grid.offsets[1]]
    
    def find_start_node(self, grid):
        for y, row in enumerate(grid.grid):
            for x, node in enumerate(row):
                if node.is_start:
                    return [x - grid.offsets[0], y - grid.offsets[1]]
                


    def get_best_node(self, possible_nodes):
        if len(possible_nodes) > 0:
            best_node = possible_nodes[0]
            if best_node[:2] == list(self.current_robot_node):
                best_node = possible_nodes[1]

            orientation = substractLists(self.current_robot_node, self.previous_robot_node)
            forward_node = sumLists(self.current_robot_node, orientation)
            for node in possible_nodes[:10]:
                if list(node[:2]) == list(forward_node):
                    best_node = forward_node

        else:
            best_node = self.current_robot_node
        #return possibleNodes[-1][:2]
        return best_node[:2]
    
    def check_path(self, grid):
        for position in self.a_star_path:
            if not is_traversable(grid, position):
                return False
        return True
    
    def predict(self, grid):
        robot_node = self.find_robot_node(grid)
        
        if robot_node != self.current_robot_node:
            self.previous_robot_node = self.current_robot_node
            self.current_robot_node = robot_node
        if self.previous_robot_node is None:
            self.previous_robot_node = self.current_robot_node

        if len(self.a_star_path) <= self.a_star_index or not self.check_path(grid):
            direction = substractLists(self.current_robot_node, self.previous_robot_node)
            if is_traversable(grid, self.current_robot_node):
                possible_nodes = bfs(grid, self.current_robot_node, 100)
            else:
                possible_nodes = bfs(grid, self.previous_robot_node, 100)

            #print("Possible nodes:", possible_nodes)
            if len(possible_nodes):
                self.best_node = self.get_best_node(possible_nodes)
            else:
                self.best_node = self.find_start_node(grid)

            best_path = a_star(grid, self.current_robot_node, self.best_node)

            if len(best_path) > 1:
                self.a_star_path = best_path[1:]
                self.a_star_index = 0

        for node in self.a_star_path:
            grid.get_node(node).mark1 = True
        grid.print_grid()

        move = substractLists(self.a_star_path[self.a_star_index], self.current_robot_node)
        move = multiplyLists(move, [0.5, 0.5])

        if self.current_robot_node == list(self.a_star_path[self.a_star_index]):
            self.a_star_index += 1

        #print("Best node:", self.best_node)
        #print("Start node:", self.current_robot_node)
        #print("AStar path: ", self.a_star_path)


        return [int(m) for m in move]
        



window_n = 0


# World constants
TIME_STEP = 32
TILE_SIZE = 0.06
TIME_IN_ROUND = (8 * 60)


# Components
#Robot
robot = RobotLayer(TIME_STEP)

# Stores, changes and compare states
stateManager = StateManager("init")

# Sequence manager
# Resets flags that need to be in a certain value when changing sequence, for example when changing state
def resetSequenceFlags():
    robot.delay_first_time = True
seq = SequenceManager(resetFunction=resetSequenceFlags)

# Mapper
mapper = Mapper(TILE_SIZE)

closest_position_agent = ClosestPositionAgent()


# Variables
do_mapping = False
do_victim_reporting = False


# Functions
# Sequential functions used frequently
seqPrint = seq.makeSimpleEvent(print)
seqDelaySec = seq.makeComplexEvent(robot.delay_sec)
seqMoveWheels = seq.makeSimpleEvent(robot.move_wheels)
seqRotateToDegs = seq.makeComplexEvent(robot.rotate_to_degs)
seqMoveToCoords = seq.makeComplexEvent(robot.move_to_coords)
seqResetSequenceFlags = seq.makeSimpleEvent(resetSequenceFlags)

# Calculates offsets in the robot position, in case it doesn't start perfectly centerd
def calibratePositionOffsets():
    actualTile = [robot.position[0] // TILE_SIZE, robot.position[1] // TILE_SIZE]
    robot.position_offsets = [
        round((actualTile[0] * TILE_SIZE) - robot.position[0]) + TILE_SIZE // 2,
        round((actualTile[1] * TILE_SIZE) - robot.position[1]) + TILE_SIZE // 2]
    robot.position_offsets = [robot.position_offsets[0] % TILE_SIZE, robot.position_offsets[1] % TILE_SIZE]
    #print("positionOffsets: ", robot.position_offsets)

def seqCalibrateRobotRotation():
    # Calibrates the robot rotation using the gps
    if seq.simpleEvent():
        robot.auto_decide_rotation = False
    seqMoveWheels(-1, -1)
    seqDelaySec(0.1)
    if seq.simpleEvent(): robot.rotation_sensor = "gps"
    seqMoveWheels(1, 1)
    seqDelaySec(0.1)
    if seq.simpleEvent(): robot.rotation_sensor= "gyro"
    seqDelaySec(0.1)
    seqMoveWheels(0, 0)
    seqMoveWheels(-1, -1)
    seqDelaySec(0.1)
    seqMoveWheels(0, 0)
    if seq.simpleEvent():
        robot.auto_decide_rotation = True

initial_position = robot.position

def seqMoveToRelativeCoords(x, y):
    global initial_position
    if seq.simpleEvent():
        initial_position = [round(p / TILE_SIZE) * TILE_SIZE for p in robot.position]
    return seqMoveToCoords((initial_position[0] + x, initial_position[1] + y))

def seqMoveToRelativeTile(x, y):
    node = mapper.robot_node
    tile = [node[0] // 2 + x, node[1] // 2 + y]
    return seqMoveToCoords(correct_position([tile[0] * TILE_SIZE, tile[1] * TILE_SIZE]))

def is_complete(grid, robot_node):
        possible_nodes = bfs(grid, robot_node, 500)
        if len(possible_nodes) == 0:
            return True
        return False

def robot_fits(robot_node, show_debug=False):
    global window_n
    robot_diameter_in_nodes = int(math.ceil(robot.diameter * mapper.lidar_grid.multiplier))
    robot_radious_in_nodes = robot_diameter_in_nodes // 2
    min_x = int(robot_node[0] - robot_radious_in_nodes + 1)
    max_x = int(robot_node[0] + robot_radious_in_nodes+ 0)
    min_y = int(robot_node[1] - robot_radious_in_nodes + 1)
    max_y = int(robot_node[1] + robot_radious_in_nodes + 0)
    min_x = max(min_x, 0)
    max_x = min(max_x, mapper.lidar_grid.grid.shape[0])
    min_y = max(min_y, 0)
    max_y = min(max_y, mapper.lidar_grid.grid.shape[1])
    #print("square: ", min_x, max_x, min_y, max_y)
    square = mapper.lidar_grid.get_bool_array()[min_y:max_y, min_x:max_x]

    square1 = copy.deepcopy(square).astype(np.uint8)
    square1 = square1 * 255

    if show_debug:
        cv.imshow(f"square{window_n}", square1.astype(np.uint8))
        #print(f"Showing square{window_n}")
        
    window_n += 1

    return np.count_nonzero(square)

def correct_position(robot_position):
    #print("INITIAL POSITION: ", robot_position)
    max_correction = 2
    exageration_factor = 1
    robot_node = [round(p * mapper.lidar_grid.multiplier) for p in robot_position]
    robot_node = [robot_node[0] + mapper.lidar_grid.offsets[0], robot_node[1] + mapper.lidar_grid.offsets[1]]

    best_node = {"pos":robot_node, "dist":0, "amount":robot_fits(robot_node, show_debug=False)}

    orientation = [abs(r - c) for r, c in zip(mapper.robot_vortex_center, robot_position)]

    if orientation[1] > orientation[0]:
        y = 0
        for x in range(-max_correction, max_correction + 1):
            possible_pos = [robot_node[0] + (x * exageration_factor), robot_node[1] + (y * exageration_factor)]
            distance = math.sqrt(abs(x) ** (2) + abs(y) ** 2)
            amount_of_nodes = robot_fits(possible_pos, show_debug=False)
            
            print("varying in x")

            if amount_of_nodes < best_node["amount"]:
                best_node["pos"] = [p - 0.0 for p in possible_pos]
                best_node["dist"] = distance
                best_node["amount"] = amount_of_nodes
            elif amount_of_nodes == best_node["amount"]:
                if distance < best_node["dist"]:
                    best_node["pos"] = [p - 0.0 for p in possible_pos]
                    best_node["dist"] = distance
                    best_node["amount"] = amount_of_nodes

    elif orientation[0] > orientation[1]:
        x = 0
        for y in range(-max_correction, max_correction + 1):
            possible_pos = [robot_node[0] + (x * exageration_factor), robot_node[1] + (y * exageration_factor)]
            distance = math.sqrt(abs(x) ** (2) + abs(y) ** 2)
            amount_of_nodes = robot_fits(possible_pos)

            #print("varying in y")

            if amount_of_nodes < best_node["amount"]:
                best_node["pos"] = [p - 0.0 for p in possible_pos]
                best_node["dist"] = distance
                best_node["amount"] = amount_of_nodes
            elif amount_of_nodes == best_node["amount"]:
                if distance < best_node["dist"]:
                    best_node["pos"] = [p - 0.0 for p in possible_pos]
                    best_node["dist"] = distance
                    best_node["amount"] = amount_of_nodes

    final_pos = [(p - o) / mapper.lidar_grid.multiplier for p, o in zip(best_node["pos"], mapper.lidar_grid.offsets)]
   # print("CORRECTED POSITION: ", final_pos)
    return final_pos
import time

hora_actual = int(time.time() * 1000)
print("Hora Actual en milsegundos")
# Each timeStep
while robot.do_loop():
    tiempo_loop = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo de time_step {tiempo_loop} milisegundos")
    print("----------------------------------------------------")
    # Updates robot position and rotation, sensor positions, etc.
    robot.update()    
    tiempo_update = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo de update {tiempo_update - tiempo_loop} milisegundos")
    print("----------------------------------------------------")
    # Loads data to mapping
    if do_mapping:
        lidar_point_cloud = robot.get_detection_point_cloud()
        images = robot.get_camera_images()
        #utilities.save_image(images[1], "camera_image_center.png")
        mapper.update(lidar_point_cloud, images, robot.position, robot.rotation, current_time=robot.time)
        tiempo_mapper_update = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de mapper.update {tiempo_mapper_update - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
        tiempo_if_mapping_uno = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de if_mapping_uno {tiempo_if_mapping_uno - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
    else:
        mapper.update(robot_position=robot.position, robot_rotation=robot.rotation, current_time=robot.time)        
        tiempo_else_mapping_uno = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de else_mapping_uno {tiempo_else_mapping_uno - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
        
    if do_mapping:
        images = robot.get_camera_images()
        for index, image in enumerate(images):
            angle = (index - 1) * 90

            rot_img = np.rot90(image, -1)

            victims = find_victims(rot_img)
            if len(victims) > 0:
                letter = classify_fixture(victims[0])
                if letter is not None:
                    mapper.load_fixture(letter, angle, robot.rotation)
                    
                break
    
    tiempo_mapping_dos = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo de mapping_dos {tiempo_mapping_dos - tiempo_loop} milisegundos")
    print("----------------------------------------------------")
    
    if is_complete(mapper.node_grid, mapper.robot_node) and mapper.node_grid.get_node(mapper.robot_node).is_start:
        seq.resetSequence()
        stateManager.changeState("end")
        
    tiempo_is_complete = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo de is_complete {tiempo_is_complete - tiempo_loop} milisegundos")
    print("----------------------------------------------------")
    
    tune_filter(robot.get_camera_images()[1])
    
    tiempo_tune_filter = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo de tune_filter {tiempo_tune_filter - tiempo_loop} milisegundos")
    print("----------------------------------------------------")

    # Updates state machine
    if not stateManager.checkState("init"):
        print("stuck_counter: ", robot.stuck_counter)
        if robot.is_stuck():
            print("FRONT BLOCKED")
            mapper.block_front_vortex(robot.rotation)
            if not stateManager.checkState("stuck"):
                seq.resetSequence()
                stateManager.changeState("stuck")
    
    tiempo_update_state_machine = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo de update_state_machine {tiempo_update_state_machine - tiempo_loop} milisegundos")
    print("----------------------------------------------------")
    
    if mapper.get_fixture().exists and not mapper.get_fixture().reported and do_victim_reporting:
        if not stateManager.checkState("report_victim"):
            seq.resetSequence()
            stateManager.changeState("report_victim")

    print("state: ", stateManager.state)
    
    tiempo_mapper_fixture = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo de mapper_fixture {tiempo_mapper_fixture - tiempo_loop} milisegundos")
    print("----------------------------------------------------")

    # Runs once when starting the game
    if stateManager.checkState("init"):
        seq.startSequence()
        seqDelaySec(0.5)
        # Calculates offsets in the robot position, in case it doesn't start perfectly centerd
        seq.simpleEvent(calibratePositionOffsets)
        # Informs the mapping components of the starting position of the robot
        seq.simpleEvent(mapper.register_start, robot.position)
        # Calibrates the rotation of the robot using the gps
        seqCalibrateRobotRotation()
        # Starts mapping walls
        if seq.simpleEvent():
            do_mapping = True
            do_victim_reporting = True
        if seq.simpleEvent():
            #do_mapping = False
            #do_victim_reporting = False
            pass
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()

        tiempo_state_init = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de state_init {tiempo_state_init - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
    
    elif stateManager.checkState("stop"):
        seq.startSequence()
        seqMoveWheels(0, 0)
        tiempo_check_state_stop = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de check_state_stop {tiempo_check_state_stop - tiempo_loop} milisegundos")
        print("----------------------------------------------------")

    # Explores and maps the maze
    elif stateManager.checkState("explore"):
        seq.startSequence()

        grid = mapper.get_node_grid()
        move = closest_position_agent.get_action(grid)
        print("move: ", move)
        node = mapper.robot_node

        if seqMoveToRelativeTile(move[0], move[1]):
            mapper.set_robot_node(robot.position)
        seq.seqResetSequence()

        print("rotation:", robot.rotation)
        print("position:", robot.position)
        tiempo_state_explorer = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de state_explorer {tiempo_state_explorer - tiempo_loop} milisegundos")
        print("----------------------------------------------------")

    # Reports a victim
    elif stateManager.checkState("report_victim"):
        seq.startSequence()
        seqMoveWheels(0, 0)
        seqDelaySec(1.2)
        if seq.simpleEvent():
            fixture = mapper.get_fixture()
            robot.comunicator.sendVictim(robot.position, fixture.type)
            fixture.reported = True
            mapper.load_wall_fixture(letter, fixture.detection_angle)
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
        
        tiempo_state_report_victim = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de state_report_victim {tiempo_state_report_victim - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
    
    elif stateManager.checkState("teleported"):
        seq.startSequence()
        # parar mapping
        do_mapping = False
        seqCalibrateRobotRotation()
        # Changes state and resets the sequence
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
        tiempo_state_teleported = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de state_teleported {tiempo_state_teleported - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
        
    
    elif stateManager.checkState("end"):
        robot.comunicator.sendMap(mapper.get_grid_for_bonus())
        robot.comunicator.sendEndOfPlay()
        tiempo_check_state_end = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de check_state_end {tiempo_check_state_end - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
    
    elif stateManager.changeState("stuck"):
        seq.startSequence()
        if seq.simpleEvent():
            robot.auto_decide_rotation = False
            robot.rotation_sensor = "gyro"
        seqMoveWheels(-0.5, -0.5)
        seqDelaySec(0.2)
        seqMoveWheels(0, 0)
        if seq.simpleEvent():
            robot.auto_decide_rotation = True
        seq.simpleEvent(stateManager.changeState, "explore")
        seq.seqResetSequence()
        tiempo_state_stuck = int(time.time() * 1000)
        print("----------------------------------------------------")
        print(f"Tiempo de state_stuck {tiempo_state_stuck - tiempo_loop} milisegundos")
        print("----------------------------------------------------")
    
    window_n = 0
    tiempo_final_loop = int(time.time() * 1000)
    print("----------------------------------------------------")
    print(f"Tiempo final_loop {tiempo_final_loop - tiempo_loop} milisegundos")
    print("----------------------------------------------------")   
