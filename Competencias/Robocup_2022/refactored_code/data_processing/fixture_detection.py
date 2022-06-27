import cv2 as cv
import numpy as np
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

red_filter = Filter(lower_hsv=(73, 157, 127), upper_hsv=(179, 255, 255))
yellow_filter = Filter(lower_hsv=(0, 157, 82), upper_hsv=(40, 255, 255))
white_filter = Filter(lower_hsv=(0, 0, 200), upper_hsv=(0, 255, 255))
black_filter = Filter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 10))
vicitim_letter_filter = Filter(lower_hsv=(0, 0, 0), upper_hsv=(5, 255, 100))


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
        print("victim:", vic["position"], vic["image"].shape)
        if vic["image"].shape[1] > 15:
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
    #print(binary_image)
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
        print("image shape:", img.shape)
        all_points = np.where(img == 255)
        all_points = all_points[0]
        count = len(all_points)
        color_point_counts[key] = count
    
    print(color_point_counts)

    if is_poison(color_point_counts["black"], color_point_counts["white"]):
        print("Poison!")
        letter = "P"
    
    if is_victim(color_point_counts["black"], color_point_counts["white"]):
        cv.imshow("black filter:", color_images["black"])
        letter = classify_victim(vic)
        print("Victim:", letter)
        
    
    if is_corrosive(color_point_counts["black"], color_point_counts["white"]):
        print("Corrosive!")
        letter = "C"
    
    if is_organic_peroxide(color_point_counts["red"], color_point_counts["yellow"]):
        print("organic peroxide!")
        letter = "O"
    
    if is_flammable(color_point_counts["red"], color_point_counts["white"]):
        print("Flammable!")
        letter = "F"

    return letter