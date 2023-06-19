from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
import numpy as np

class OccupiedMapper:
    def __init__(self, grid: CompoundExpandablePixelGrid) -> None:
        self.__grid = grid

    def map_occupied(self):
        self.__grid.arrays["occupied"] = np.bitwise_or(self.__grid.arrays["walls"], self.__grid.arrays["holes"])

        self.__grid.arrays["occupied"][self.__grid.arrays["traversed"]] = False