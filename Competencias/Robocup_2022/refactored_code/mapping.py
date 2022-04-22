import numpy as np
import cv2 as cv
import sys
import copy

import utilities
import point_cloud_processor

class Grid:
    def __init__(self, initial_shape):
        self.offsets = [initial_shape[0] // 2, initial_shape[1] // 2]
        self.resolution = 10
        self.shape = initial_shape
        self.grid = np.zeros(self.shape, dtype=np.uint8)

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
    
    
    def add_point(self, point):
        self.expand_grid_to_point(point)
        
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
    
        self.grid[y, x] = 255
    
    def get_point(self, point):
        x, y = point
        x, y = x + self.offsets[0], y + self.offsets[1]
        return self.grid[y, x]
     
    def add_end_row(self, size):
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((self.grid, np.zeros((size, self.shape[1]), dtype=np.uint8)))
    
    def add_begining_row(self, size):
        self.offsets[1] += size
        self.shape = (self.shape[0]+ size, self.shape[1] )
        self.grid = np.vstack((np.zeros((size, self.shape[1]), dtype=np.uint8), self.grid))
    
    def add_end_column(self, size):
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((self.grid, np.zeros((self.shape[0], size), dtype=np.uint8)))

    def add_begining_column(self, size):
        self.offsets[0] += size
        self.shape = (self.shape[0], self.shape[1] + size)
        self.grid = np.hstack((np.zeros((self.shape[0], size), dtype=np.uint8), self.grid))

    def print_grid(self):
        print("grid shape: ", self.grid.shape)

        cv.imshow("grid", self.grid)
        cv.waitKey(1)

def overlay_image_alpha(img, img_overlay, x, y, alpha_mask):
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
    print("ALPHA_MAX", np.amax(alpha))
    print("ALPHA_MIN", np.amin(alpha))
    alpha_inv = 1.0 - alpha

    img_crop[:] = img_overlay_crop * alpha + img_crop * alpha_inv

def join_camera_images(images, translations):
    max_x = 0
    max_y = 0

    for translation in translations:
        max_x = max(max_x, translation[1])
        max_y = max(max_y, translation[0])

    backround = np.zeros((max_x + images[0].shape[1], max_y + images[0].shape[0], 3), dtype=np.uint8)
    
    rot_imgs = []
    for index, img in enumerate(images):
        rot_imgs.append(np.rot90(img, index + 2, (0,1)))
    #cv.imshow("channel1", rot_imgs[0][:,:,0])
    #cv.imshow("channel2", rot_imgs[0][:,:,1])
    #cv.imshow("channel3", rot_imgs[0][:,:,2])
    #cv.imshow("channel4", rot_imgs[0][:,:,3])
    
    for rot_img, translation in zip(rot_imgs, translations):
        overlay_image_alpha(backround, rot_img[:,:,:3], translation[0], translation[1], rot_img[:,:,3] / 255)
    print("BACK_MAX", np.amax(backround))
    print("BACK_MIN", np.amin(backround))
    
    return backround.copy()

if __name__ == "__main__":
    img = cv.imread("/home/ale/rescate_laberinto/Competencias/Robocup_2022/refactored_code/img2.png")
    print(img.shape)
    backround = np.zeros((2000, 2000, 3))
    overlay_image_alpha(backround, img, 200, 200, np.ones((400, 500)))
    cv.imwrite("/home/ale/rescate_laberinto/Competencias/Robocup_2022/refactored_code/img_bck.png", backround)

"""

    my_grid = Grid((5, 5))

    my_grid.add_point((2, 2))
    print(my_grid.offsets)

    my_grid.print_grid()

    my_grid.add_point((-5, -10))
    print(my_grid.offsets)

    my_grid.print_grid()

class Graph:
    def __init__(self) -> None:
        self.graph = {}
    
    def isAdjacent(self, pos1, pos2):
        diff = []
        for p1, p2 in zip(pos1, pos2):
            diff = p1 - p2
            if diff > 1 or diff < -1:
                return False
        return True

    def block(self, pos1, pos2):
        if pos1 in self.graph:
            self.graph[pos1].append(pos2)
        else:
            self.graph[pos1] = [pos2, ]
        if pos2 in self.graph:
            self.graph[pos2].append(pos2)
        else:
            self.graph[pos2] = [pos1, ]
    
    def check(self, pos1, pos2):
        if not self.isAdjacent(pos1, pos2):
            return False
        if pos1 in self.graph:
            return pos2 not in self.graph[pos1]
        else:
            return True
    
    def unblock(self, pos1, pos2):
        if pos1 in self.graph:
            self.graph[pos1].remove(pos2)
        if pos2 in self.graph:
            self.graph[pos2].remove(pos2)

my_graph = Graph()

my_graph.block((0, 0), (0, 1))

print(my_graph.graph)

print("puedo pasar?", my_graph.check((0,2), (0,1)))


"""