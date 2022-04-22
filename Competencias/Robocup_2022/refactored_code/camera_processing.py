import cv2 as cv
import numpy as np

def flatten_image(image):

    tile_size = 100
    tiles_up = 2
    tiles_side = 1

     

    warp_coef_x = 1#1.2
    warp_coef_y = 1#.037
    
    tile_size_x = tile_size * warp_coef_x
    tile_size_y = tile_size * warp_coef_y

    minimum_x = tile_size * tiles_side
    maximum_x = tile_size * (tiles_side + 1)
    minimum_y = tile_size * (tiles_up - 1)
    maximum_y = tile_size * tiles_up

    img_points = np.array(([38, 30* 0.980], [91, 30* 0.980],  [111, 50 * 1.008],  [20, 50* 1.008],), dtype=np.float32)
    final_points = np.array(([minimum_x, minimum_y],  [maximum_x, minimum_y], [maximum_x, maximum_y], [minimum_x, maximum_y],), dtype=np.float32)

    ipm_matrix = cv.getPerspectiveTransform(img_points, final_points)
    final_x = tile_size * ((tiles_side * 2) + 1)
    final_y = tile_size * (tiles_up + 1)
    ipm = cv.warpPerspective(image, ipm_matrix, (final_x, final_y), flags=cv.INTER_NEAREST)

    return ipm

if __name__ == "__main__":
    img = cv.imread("/home/ale/rescate_laberinto/Competencias/Robocup_2022/refactored_code/img.png")
    img1 = flatten_image(img)
    for y, row in enumerate(img1):
        for x, pixel in enumerate(row):
            if y % 100 == 0 or x % 100 == 0:
                img1[y][x] = [255, 255, 255]
    cv.imwrite("/home/ale/rescate_laberinto/Competencias/Robocup_2022/refactored_code/img2.png", img1)
    cv.waitKey(0)