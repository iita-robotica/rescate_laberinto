from flags import SHOW_DEBUG
import math
import copy
import numpy as np
import cv2 as cv

class PositionCorrector:
    def __init__(self, robot_diameter) -> None:
        self.robot_diameter = robot_diameter
    

    def robot_fits(self, robot_node, lidar_grid):
        robot_diameter_in_nodes = int(math.ceil(self.robot_diameter * lidar_grid.multiplier))
        robot_radious_in_nodes = robot_diameter_in_nodes // 2
        min_x = int(robot_node[0] - robot_radious_in_nodes + 1)
        max_x = int(robot_node[0] + robot_radious_in_nodes+ 0)
        min_y = int(robot_node[1] - robot_radious_in_nodes + 1)
        max_y = int(robot_node[1] + robot_radious_in_nodes + 0)
        min_x = max(min_x, 0)
        max_x = min(max_x, lidar_grid.grid.shape[0])
        min_y = max(min_y, 0)
        max_y = min(max_y, lidar_grid.grid.shape[1])
        #print("square: ", min_x, max_x, min_y, max_y)
        square = lidar_grid.get_bool_array()[min_y:max_y, min_x:max_x]

        square1 = copy.deepcopy(square).astype(np.uint8)
        square1 = square1 * 255

        """
        if SHOW_DEBUG:
            try:
                cv.imshow(f"square{self.window_n}", square1.astype(np.uint8))
                print(f"Showing square{self.window_n}")
            except:
                print(f"Error showing square{self.window_n}")
        self.window_n += 1
        """
        return np.count_nonzero(square)

    def correct_position(self, robot_position, robot_vortex, lidar_grid):
        if SHOW_DEBUG:
            print("INITIAL POSITION: ", robot_position)
        max_correction = 2
        exageration_factor = 1
        robot_node = [round(p * lidar_grid.multiplier) for p in robot_position]
        robot_node = [robot_node[0] + lidar_grid.offsets[0], robot_node[1] + lidar_grid.offsets[1]]

        best_node = {"pos":robot_node, "dist":0, "amount":self.robot_fits(robot_node, lidar_grid)}

        orientation = [abs(r - c) for r, c in zip(robot_vortex, robot_position)]

        if orientation[1] > orientation[0]:
            y = 0
            for x in range(-max_correction, max_correction + 1):
                possible_pos = [robot_node[0] + (x * exageration_factor), robot_node[1] + (y * exageration_factor)]
                distance = math.sqrt(abs(x) ** (2) + abs(y) ** 2)
                amount_of_nodes = self.robot_fits(possible_pos, lidar_grid)
                
                if amount_of_nodes < best_node["amount"]:
                    best_node["pos"] = [p - 0.0 for p in possible_pos]
                    best_node["dist"] = distance
                    best_node["amount"] = amount_of_nodes
                elif amount_of_nodes == best_node["amount"]:
                    if distance < best_node["dist"]:
                        best_node["pos"] = [p - 0.0 for p in possible_pos]
                        best_node["dist"] = distance
                        best_node["amount"] = amount_of_nodes

        elif orientation[0] > orientation[1]:
            x = 0
            for y in range(-max_correction, max_correction + 1):
                possible_pos = [robot_node[0] + (x * exageration_factor), robot_node[1] + (y * exageration_factor)]
                distance = math.sqrt(abs(x) ** (2) + abs(y) ** 2)
                amount_of_nodes = self.robot_fits(possible_pos, lidar_grid)

                #print("varying in y")

                if amount_of_nodes < best_node["amount"]:
                    best_node["pos"] = [p - 0.0 for p in possible_pos]
                    best_node["dist"] = distance
                    best_node["amount"] = amount_of_nodes
                elif amount_of_nodes == best_node["amount"]:
                    if distance < best_node["dist"]:
                        best_node["pos"] = [p - 0.0 for p in possible_pos]
                        best_node["dist"] = distance
                        best_node["amount"] = amount_of_nodes

        final_pos = [(p - o) / lidar_grid.multiplier for p, o in zip(best_node["pos"], lidar_grid.offsets)]
        #print("CORRECTED POSITION: ", final_pos)
        return final_pos