import cv2 as cv
import numpy as np
import random

class Filter:
    def __init__(self, lowerHSV, upperHSV):
        self.lower = np.array(lowerHSV)
        self.upper = np.array(upperHSV)
    
    def Filter(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        #imgResult = cv.bitwise_and(img, img, mask=mask)
        return mask

redFilter = Filter(lowerHSV=(73, 157, 127), upperHSV=(179, 255, 255))
yellowFilter = Filter(lowerHSV=(0, 157, 82), upperHSV=(40, 255, 255))
whiteFilter = Filter(lowerHSV=(0, 0, 200), upperHSV=(0, 255, 255))
blackFilter = Filter(lowerHSV=(0, 0, 0), upperHSV=(0, 255, 10))
victimLetterFilter = Filter(lowerHSV=(0, 0, 0), upperHSV=(5, 255, 100))


def sumImages(images):
    finalImg = images[0]
    for index, image in enumerate(images):
        finalImg += image
        #cv.imshow(str(index), image)
    return finalImg

def filterVictims(vicitms):
    finalVictims = []
    for vic in vicitms:
        if 0 < vic["position"][0] < 60:
            finalVictims.append(vic)
    return finalVictims

def findVictims(image):
    """
    Finds victims in the image.
    Returns a list of dictionaries containing vicitims positions and images.
    """
    binaryImages = [redFilter.Filter(image), 
                    yellowFilter.Filter(image), 
                    whiteFilter.Filter(image), 
                    blackFilter.Filter(image)]

    binaryImage = sumImages(binaryImages)
    #cv.imshow("binaryImage", binaryImage)
    
    # Encuentra los contornos, aunque se puede confundir con el contorno de la letra
    contours, _ = cv.findContours(binaryImage, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # Pra evitar la confusion dibuja rectangulos blancos donde estan los contornos en la imagen y despues vuelve a
    # sacar los contornos para obtener solo los del rectangulo, no los de las letras.
    for c0 in contours:
        x, y, w, h = cv.boundingRect(c0)
        cv.rectangle(binaryImage, (x, y), (x + w, y + h), (225, 255, 255), -1)
    contours, _ = cv.findContours(binaryImage, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # saca las medidas y la posicion de los contornos y agrega a la lista de imagenes la parte esa de la imagen original
    # Tambien anade la posicion de cada recuadro en la imagen original
    finalVicitms = []
    for c in contours:
        x, y, w, h = cv.boundingRect(c)
        finalVicitms.append({"image":image[y:y + h, x:x + w], "position":(x, y)})
    return filterVictims(finalVicitms)

def cropWhite(binaryImg):
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

def classifyVictim(victim):
    white = 255
    img = victim["image"]
    img =  cv.resize(img, (100, 100), interpolation=cv.INTER_AREA)
    #conts, h = cv.findContours(thresh1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    binary = victimLetterFilter.Filter(img)

    letter1 = cropWhite(binary)
    letter1 = cv.resize(letter1, (100, 100), interpolation=cv.INTER_AREA)
    letter = letter1[:,10:90]
    letter = cropWhite(letter)
    letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)
    cv.imshow("letra", letter)
    cv.imshow("letra1", letter1)
    cv.imshow("thresh", binary)
    letterColor = cv.cvtColor(letter, cv.COLOR_GRAY2BGR)
    areaWidth = 20
    areaHeight = 30
    areas = {
        "top": ((0, areaHeight),(50 - areaWidth // 2, 50 + areaWidth // 2)),
        "middle": ((50 - areaHeight // 2, 50 + areaHeight // 2), (50 - areaWidth // 2, 50 + areaWidth // 2)),
        "bottom": ((100 - areaHeight, 100), (50 - areaWidth // 2, 50 + areaWidth // 2 ))
        }
    images = {
        "top": letter[areas["top"][0][0]:areas["top"][0][1], areas["top"][1][0]:areas["top"][1][1]],
        "middle": letter[areas["middle"][0][0]:areas["middle"][0][1], areas["middle"][1][0]:areas["middle"][1][1]],
        "bottom": letter[areas["bottom"][0][0]:areas["bottom"][0][1], areas["bottom"][1][0]:areas["bottom"][1][1]]
        }
    cv.rectangle(letterColor,(areas["top"][1][0], areas["top"][0][0]), (areas["top"][1][1], areas["top"][0][1]), (0, 255, 0), 1)
    cv.rectangle(letterColor, (areas["middle"][1][0], areas["middle"][0][0]), (areas["middle"][1][1], areas["middle"][0][1]), (0, 0, 255), 1)
    cv.rectangle(letterColor,(areas["bottom"][1][0], areas["bottom"][0][0]), (areas["bottom"][1][1], areas["bottom"][0][1]), (225, 0, 255), 1)
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

    finalLetter = random.choice(list(letters.keys()))
    for letterKey in letters.keys():
        if counts == letters[letterKey]:
            finalLetter = letterKey
            break
    
    #print(counts)
    #print(finalLetter)
    return finalLetter

def isPoison(blackPoints, whitePoints):
    return blackPoints < 600 and whitePoints > 700 and whitePoints < 4000

def isVictim(blackPoints, whitePoints):
    return whitePoints > 5000 and 2000 > blackPoints > 100

def isCorrosive(blackPoints, whitePoints):
    return 700 < whitePoints < 2500 and 1000 < blackPoints < 2500

def isFlammable(redPoints, whitePoints):
    return redPoints and whitePoints

def isOrganicPeroxide(redPoints, yellowPoints):
    return redPoints and yellowPoints

def classifyFixture(vic):
    img = vic["image"]
    possibleFixtureLetters = ["P", "O", "F", "C", "S", "H", "U"]
    letter = random.choice(possibleFixtureLetters)
    image = cv.resize(img, (100, 100), interpolation=cv.INTER_AREA)
    colorImgs = {
    "red" : redFilter.Filter(image),
    "yellow" : yellowFilter.Filter(image), 
    "white" : whiteFilter.Filter(image),
    "black" : blackFilter.Filter(image)}

    colorPointCounts = {}
    for key, img in colorImgs.items():
        print("image shape:", img.shape)
        all_points = np.where(img == 255)
        all_points = all_points[0]
        count = len(all_points)
        colorPointCounts[key] = count
    
    print(colorPointCounts)
    if isPoison(colorPointCounts["black"], colorPointCounts["white"]):
        print("Poison!")
        letter = "P"
    
    if isVictim(colorPointCounts["black"], colorPointCounts["white"]):
        cv.imshow("black filter:", colorImgs["black"])
        letter = classifyVictim(vic)
        print("Victim:", letter)
        
    
    if isCorrosive(colorPointCounts["black"], colorPointCounts["white"]):
        print("Corrosive!")
        letter = "C"
    
    if isOrganicPeroxide(colorPointCounts["red"], colorPointCounts["yellow"]):
        print("organic peroxide!")
        letter = "O"
    
    if isFlammable(colorPointCounts["red"], colorPointCounts["white"]):
        print("Flammable!")
        letter = "F"

    return letter