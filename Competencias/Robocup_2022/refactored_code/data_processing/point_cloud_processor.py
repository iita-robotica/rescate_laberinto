import numpy as np

from bresenham import bresenham

class PointCloudProcessor:
    def __init__(self, center_point, map_scale):
        self.map_scale = map_scale # 850
        self.center_point = center_point #350

    def processPointCloud(self, pc, robotPos):
        return [[pcv + rpv for pcv, rpv in zip(pos, robotPos)] for pos in pc]

    def processPointCloudForCamera(self, pc):
        if len(pc) == 0:
            return np.empty((0, 2), dtype=np.int)
        return np.array([[int(p * self.map_scale) + self.center_point for p in pos] for pos in pc], dtype=np.int)


    def get_intermediate_points(self, point_cloud):
        seen_points = list()
        for point in point_cloud:
            intermediates = list(bresenham(self.center_point[0], self.center_point[1], point[0], point[1]))
            seen_points += intermediates
        return seen_points


if __name__ == '__main__':
    pass

