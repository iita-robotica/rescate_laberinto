import math
import random

import numpy as np
import cv2 as cv

from fixture_detection.victim_clasification import VictimClassifier
from fixture_detection.color_filter import ColorFilter

from flags import SHOW_DEBUG, SHOW_FIXTURE_DEBUG
    
class FixtureType:
    def __init__(self, fixture_type, default_letter, ranges=None):
        self.fixture_type = fixture_type
        self.default_letter = default_letter
        self.ranges = ranges
    
    def is_fixture(self, colour_counts: dict):
        for color in self.ranges:
            if not self.ranges[color][0] <= colour_counts[color] <= self.ranges[color][1]:
                return False
        return True
            
class FixtureClasiffier:
    def __init__(self):
        # Victim classification
        self.victim_classifier = VictimClassifier()

        # Color filtering
        self.colors = ("black", "white", "yellow", "red")
        self.color_filters = {
            "black": ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 0)),
            "white": ColorFilter(lower_hsv=(0, 0, 207), upper_hsv=(0, 0, 207)),
            "yellow": ColorFilter(lower_hsv=(25, 157, 82), upper_hsv=(30, 255, 255)),
            "red": ColorFilter(lower_hsv=(160, 170, 127), upper_hsv=(170, 255, 255))
        }

        # Fixture filtering
        self.min_fixture_height = 23
        self.min_fixture_width = 19

        #self.min_fixture_height = 20
        #self.min_fixture_width = 15
    
        # Fixture classification
        self.possible_fixture_letters = ["P", "O", "F", "C", "S", "H", "U"]

        # In order of priority
        self.fixture_types = (
            FixtureType("already_detected", "",  {"white": (1,    math.inf), 
                                                  "black": (0,    0),
                                                  "red":   (0,    0), 
                                                  "yellow":(0,    0),}),

            FixtureType("flammable", "F",        {"white": (1,    math.inf), 
                                                  "red":   (1,    math.inf),}),

            FixtureType("organic_peroxide", "O", {"red":   (1,    math.inf), 
                                                  "yellow":(1,    math.inf),}),

            FixtureType("victim",    "H",        {"white": (4000, math.inf), 
                                                  "black": (100,  4000),}),

            FixtureType("corrosive", "C",        {"white": (700,  2500), 
                                                  "black": (1000, 2500),}),

            FixtureType("poison",    "P",        {"white": (700,  4000), 
                                                  "black": (0,    600),}),
        )                    


        # For tuning color filters
        self.do_color_filter_tuning = False
        self.filter_for_tuning = self.color_filters["white"]                       

        if self.do_color_filter_tuning:
            cv.namedWindow("trackbars")

            cv.createTrackbar("min_h", "trackbars", self.filter_for_tuning.lower[0], 255, lambda x: None)
            cv.createTrackbar("max_h", "trackbars", self.filter_for_tuning.upper[0], 255, lambda x: None)

            cv.createTrackbar("min_s", "trackbars", self.filter_for_tuning.lower[1], 255, lambda x: None)
            cv.createTrackbar("max_s", "trackbars", self.filter_for_tuning.upper[1], 255, lambda x: None)

            cv.createTrackbar("min_v", "trackbars", self.filter_for_tuning.lower[2], 255, lambda x: None)
            cv.createTrackbar("max_v", "trackbars", self.filter_for_tuning.upper[2], 255, lambda x: None)
        
    def tune_filter(self, image):
        min_h = cv.getTrackbarPos("min_h", "trackbars")
        max_h = cv.getTrackbarPos("max_h", "trackbars")
        min_s = cv.getTrackbarPos("min_s", "trackbars")
        max_s = cv.getTrackbarPos("max_s", "trackbars")
        min_v = cv.getTrackbarPos("min_v", "trackbars")
        max_v = cv.getTrackbarPos("max_v", "trackbars")
        self.filter_for_tuning = ColorFilter((min_h, min_s, min_v), (max_h, max_s, max_v))
        print(self.filter_for_tuning.lower, self.filter_for_tuning.upper)
        cv.imshow("tunedImage", self.filter_for_tuning.filter(image))


    def sum_images(self, images):
        final_img = images[0]
        for index, image in enumerate(images):
            final_img += image
            #cv.imshow(str(index), image)
        final_img[final_img > 255] = 255
        return final_img

    def filter_fixtures(self, victims) -> list:
        final_victims = []
        for vic in victims:
            if SHOW_FIXTURE_DEBUG:
                print("victim:", vic["position"], vic["image"].shape)

            if vic["image"].shape[0] > self.min_fixture_height and vic["image"].shape[1] > self.min_fixture_width:
                final_victims.append(vic)

        return final_victims

    def find_fixtures(self, image) -> list:
        """
        Finds fixtures in the image.
        Returns a list of dictionaries containing fixture positions and images.
        """
        binary_images = []
        for f in self.color_filters.values():
            binary_images.append(f.filter(image))

        binary_image = self.sum_images(binary_images)
        #print(binary_image)
        if SHOW_FIXTURE_DEBUG:
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

        #print("unfiltered", len(final_victims))
        return self.filter_fixtures(final_victims)
            
    def count_colors(self, image) -> dict:
        color_point_counts = {}

        for name, filter in self.color_filters.items():
            # Filter image to get specific color
            color_image = filter.filter(image)

            # Count where the mask is true
            color_point_counts[name] = np.count_nonzero(color_image)

        return color_point_counts

    def classify_fixture(self, fixture) -> str:
        image = cv.resize(fixture["image"], (100, 100), interpolation=cv.INTER_AREA)

        color_point_counts = self.count_colors(image)
        
        if SHOW_FIXTURE_DEBUG:
            print(color_point_counts)

        # Check all filters. Find first color counts that fit.
        final_fixture_filter = None
        for filter in self.fixture_types:
            if filter.is_fixture(color_point_counts):
                final_fixture_filter = filter
                break
        
        # If nothing matches return random letter
        if final_fixture_filter is None:
            letter = random.choice(self.possible_fixture_letters)

        # If it's a victim classify it
        elif final_fixture_filter.fixture_type == "victim":
            letter = self.victim_classifier.classify_victim(fixture)

        # If already detected then it shouldn't be reported
        elif final_fixture_filter.fixture_type == "already_detected":
            letter = None
        
        # If it's any other type then the letter defined for it can be returned
        else:
            letter = final_fixture_filter.default_letter

        if SHOW_FIXTURE_DEBUG:
            print("FIXTURE: ", letter)

        return letter