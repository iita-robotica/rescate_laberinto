import cv2 as cv
import numpy as np

def flatten_image(image):
    minimum = 200
    maximum = 300

    img_points = np.array(([38, 30], [91, 30],  [111, 50],  [20, 50],), dtype=np.float32)
    final_points = np.array(([minimum, minimum],  [maximum, minimum], [maximum, maximum], [minimum, maximum],), dtype=np.float32)

    ipm_matrix = cv.getPerspectiveTransform(img_points, final_points)
    ipm = cv.warpPerspective(image, ipm_matrix, (500, 400), flags=cv.INTER_NEAREST)

    return ipm

if __name__ == "__main__":
    img = cv.imread("/home/ale/rescate_laberinto/Competencias/Robocup_2022/Refactored Code/img.png")
    img1 = flatten_image(img)
    cv.imwrite("/home/ale/rescate_laberinto/Competencias/Robocup_2022/Refactored Code/img2.png", img1)