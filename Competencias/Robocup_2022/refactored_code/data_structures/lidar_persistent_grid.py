from data_structures import resizable_pixel_grid

class LidarGrid(resizable_pixel_grid.Grid):
    def __init__(self, input_resolution, resolution, threshold=0):
        self.input_res = input_resolution
        self.res = resolution
        self.multiplier = self.res / self.input_res
        
        self.shape = (self.res, self.res)
        super().__init__(self.shape, self.res)
        self.threshold = 100

    def get_bool_array(self):
        return self.grid > self.threshold
    
    def sum_detection(self, point):
        point = [round(p * self.multiplier) for p in point]
        self.sum_to_point(point, 1)
    
    def update(self, point_cloud):
        for point in point_cloud:
            self.sum_detection(point)

        
    