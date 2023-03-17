import numpy as np
import cv2 as cv

img = cv.imread(r"C:/Users/ANA/Desktop/Webots - Erebus/rescate_laberinto/Alumnos/Alejandro_de_Ugarriza/TestVictimS.png")

def classifyVictim(img):
    img = cv.resize(img, (100, 100))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    thresh1 = cv.threshold(gray, 100, 255, cv.THRESH_BINARY_INV)[1]
    conts, h = cv.findContours(thresh1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv.boundingRect(conts[0])
    letter = thresh1[y:y + h, x:x + w]
    letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)
    #letterColor = cv.cvtColor(letter, cv.COLOR_GRAY2BGR)
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
    #cv.rectangle(letterColor,(areas["top"][1][0], areas["top"][0][0]), (areas["top"][1][1], areas["top"][0][1]), (0, 255, 0), 1)
    #cv.rectangle(letterColor, (areas["middle"][1][0], areas["middle"][0][0]), (areas["middle"][1][1], areas["middle"][0][1]), (0, 0, 255), 1)
    #cv.rectangle(letterColor,(areas["bottom"][1][0], areas["bottom"][0][0]), (areas["bottom"][1][1], areas["bottom"][0][1]), (225, 0, 255), 1)
    counts = {}
    acceptanceThreshold = 50
    for key in images.keys():
        count = 0
        for row in images[key]:
            for pixel in row:
                if pixel == 255:
                    count += 1
        counts[key] = count > acceptanceThreshold
    letters = {
        "H":{'top': False, 'middle': True, 'bottom': False},
        "S":{'top': True, 'middle': True, 'bottom': True},
        "U":{'top': False, 'middle': False, 'bottom': True}
        }
    for letterKey in letters.keys():
        if counts == letters[letterKey]:
            finalLetter = letterKey
            break
    return finalLetter

#cv.imshow("gray", gray)
#cv.imshow("thresh1", thresh1)
cv.imshow("letter", letter)
cv.imshow("lettercolor", letterColor)
cv.imshow("top", images["top"])
cv.imshow("middle", images["middle"])
cv.imshow("bottom", images["bottom"])


cv.waitKey(10000)
