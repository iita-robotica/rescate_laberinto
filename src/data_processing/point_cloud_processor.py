import numpy as np

from bresenham import bresenham
from skimage.draw import line
from scipy.spatial.distance import cdist

class PointCloudProcessor:
    def __init__(self, center_point, map_scale):
        self.map_scale = map_scale # 850
        self.center_point = [center_point, center_point] #350

    def processPointCloud(self, pc, robotPos):
        return [[pcv + rpv for pcv, rpv in zip(pos, robotPos)] for pos in pc]

    def processPointCloudForCamera(self, pc):
        if len(pc) == 0:
            return np.empty((0, 2), dtype=int)
        return np.array([[int(p * self.map_scale) + c for c, p in zip(self.center_point,  pos)] for pos in pc], dtype=int)


    def get_intermediate_points(self, point_cloud):
        max_points_per_line = int(np.max(cdist(point_cloud, [self.center_point]))) + 1  # calculate max possible points per line
        seen_x = np.empty(len(point_cloud) * max_points_per_line, dtype=int)
        seen_y = np.empty(len(point_cloud) * max_points_per_line, dtype=int)
        index = 0
        for point in point_cloud:
            xx, yy = line(self.center_point[0], self.center_point[1], point[0], point[1]) 
            n_points = len(xx)
            seen_x[index:index+n_points] = xx
            seen_y[index:index+n_points] = yy
            index += n_points
        return seen_x[:index], seen_y[:index]


if __name__ == '__main__':
    pass

