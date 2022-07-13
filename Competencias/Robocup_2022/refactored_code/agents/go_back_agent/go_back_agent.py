from agents.agent import Agent
import utilities

from algorithms.expandable_node_grid.a_star import a_star

from flags import SHOW_DEBUG

class GoBackAgent(Agent):
    def __init__(self):
        super().__init__(["up", "down", "left", "right"])
        self.current_robot_node = self.previous_robot_node = None
        self.best_node = [None, None]
        self.a_star_path = []
        self.a_star_index = 0

    def find_robot_node(self, grid):
        for y, row in enumerate(grid.grid):
            for x, node in enumerate(row):
                if node.is_robots_position:
                    return [x - grid.offsets[0], y - grid.offsets[1]]
    
    def find_start_node(self, grid):
        for y, row in enumerate(grid.grid):
            for x, node in enumerate(row):
                if node.is_start:
                    return [x - grid.offsets[0], y - grid.offsets[1]]
    
    def predict(self, grid):
        robot_node = self.find_robot_node(grid)
        
        if robot_node != self.current_robot_node:
            self.previous_robot_node = self.current_robot_node
            self.current_robot_node = robot_node
        if self.previous_robot_node is None:
            self.previous_robot_node = self.current_robot_node
        
        start_node = self.find_start_node(grid)
        if start_node == self.current_robot_node:
            return None

        if len(self.a_star_path) <= self.a_star_index:
            best_path = a_star(grid, self.current_robot_node, start_node)

            if len(best_path) > 1:
                self.a_star_path = best_path[1:]
                self.a_star_index = 0

        if SHOW_DEBUG:
            for node in self.a_star_path:
                grid.get_node(node).mark1 = True
            grid.print_grid()

        move = utilities.substractLists(self.a_star_path[self.a_star_index], self.current_robot_node)
        move = utilities.multiplyLists(move, [0.5, 0.5])

        if self.current_robot_node == list(self.a_star_path[self.a_star_index]):
            self.a_star_index += 1

        if SHOW_DEBUG:
            print("Best node:", self.best_node)
            print("Start node:", self.current_robot_node)
            print("AStar path: ", self.a_star_path)


        return [int(m) for m in move]