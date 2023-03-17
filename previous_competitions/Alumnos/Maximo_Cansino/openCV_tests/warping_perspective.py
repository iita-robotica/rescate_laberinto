import cv2 as cv
import numpy as np
image_path="c:/Users/Maxi/Documents/program_robots/mapping_physical_robot/openCV/assets/images/abc_positionated.png"

img = cv.imread(image_path, 1)
objetive_points = np.float32(
    [[60,90], #point 1
    [620,90],
    [60, 500],
    [580,520]] #point 4
    ) #list of 4 list

width, height = 696,564
objetive_points2 = np.float32(
    [[0,0],
    [width, 0],
    [0, height],
    [width, height]],
)
matrix = cv.getPerspectiveTransform(objetive_points, objetive_points2)
img_out_put = cv.warpPerspective(img, matrix, (width, height))
cv.imshow("windows", img)
cv.imshow("windows", img_out_put)
cv.waitKey(0)