import cv2 as cv
import numpy as np
image_path='c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/warning_webots_icons.png'



def redListener(image_path):
    image=cv.imread(image_path)
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    hue_min= 73
    hue_max=179

    saturation_min=157
    saturation_max=255

    min_value=82
    max_value=255

    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])

    mask = cv.inRange(hsv_image, lower, upper)
    imgResult = cv.bitwise_and(image, image, mask=mask)
    imgPanel = cv.hconcat(mask, imgResult)

    #cv.imshow("RedMask", mask)
    cv.imshow("RedPanel", imgResult)

def yellowListener(image_path):
    image=cv.imread(image_path)
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    hue_min= 0
    hue_max= 40

    saturation_min= 158
    saturation_max= 255

    min_value= 219
    max_value= 255

    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])

    mask = cv.inRange(hsv_image, lower, upper)
    imgResult = cv.bitwise_and(image, image, mask=mask)
    imgPanel = cv.hconcat(mask, imgResult)

    #cv.imshow("Yellow Mask", mask)
    cv.imshow("Yellow panel", imgResult)

def whiteListener(image_path):
    image=cv.imread(image_path)
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    hue_min= 0
    hue_max= 255

    saturation_min= 0
    saturation_max= 110

    min_value= 159
    max_value= 255

    lower = np.array([hue_min, saturation_min, min_value])
    upper = np.array([hue_max, saturation_max, max_value])

    mask = cv.inRange(hsv_image, lower, upper)
    imgResult = cv.bitwise_and(image, image, mask=mask)
    imgPanel = cv.hconcat(mask, imgResult)

    #cv.imshow("white Mask", mask)
    cv.imshow("white panel", imgResult)



while True:
    redListener(image_path)
    yellowListener(image_path)
    whiteListener(image_path)
    cv.waitKey(1)