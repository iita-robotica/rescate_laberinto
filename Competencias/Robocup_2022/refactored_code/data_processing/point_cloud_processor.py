import numpy as np

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

