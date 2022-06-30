from agents.agent import Agent
import utilities

from algorithms.expandable_node_grid.a_star import a_star
from algorithms.expandable_node_grid.bfs import bfs
from algorithms.expandable_node_grid.traversable import is_traversable


class ClosestPositionAgent(Agent):
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
                


    def get_best_node(self, possible_nodes):
        if len(possible_nodes) > 0:
            best_node = possible_nodes[0]
            if best_node[:2] == list(self.current_robot_node):
                best_node = possible_nodes[1]

            orientation = utilities.substractLists(self.current_robot_node, self.previous_robot_node)
            forward_node = utilities.sumLists(self.current_robot_node, orientation)
            for node in possible_nodes[:10]:
                if list(node[:2]) == list(forward_node):
                    best_node = forward_node

        else:
            best_node = self.current_robot_node
        #return possibleNodes[-1][:2]
        return best_node[:2]
    
    def check_path(self, grid):
        for position in self.a_star_path:
            if not is_traversable(grid, position):
                return False
        return True
    
    def predict(self, grid):
        robot_node = self.find_robot_node(grid)
        
        if robot_node != self.current_robot_node:
            self.previous_robot_node = self.current_robot_node
            self.current_robot_node = robot_node
        if self.previous_robot_node is None:
            self.previous_robot_node = self.current_robot_node

        if len(self.a_star_path) <= self.a_star_index or not self.check_path(grid):
            direction = utilities.substractLists(self.current_robot_node, self.previous_robot_node)
            if is_traversable(grid, self.current_robot_node):
                possible_nodes = bfs(grid, self.current_robot_node, 100)
            else:
                possible_nodes = bfs(grid, self.previous_robot_node, 100)

            #print("Possible nodes:", possible_nodes)
            if len(possible_nodes):
                self.best_node = self.get_best_node(possible_nodes)
            else:
                self.best_node = self.find_start_node(grid)

            best_path = a_star(grid, self.current_robot_node, self.best_node)

            if len(best_path) > 1:
                self.a_star_path = best_path[1:]
                self.a_star_index = 0

        for node in self.a_star_path:
            grid.get_node(node).mark1 = True
        grid.print_grid()

        move = utilities.substractLists(self.a_star_path[self.a_star_index], self.current_robot_node)
        move = utilities.multiplyLists(move, [0.5, 0.5])

        if self.current_robot_node == list(self.a_star_path[self.a_star_index]):
            self.a_star_index += 1

        print("Best node:", self.best_node)
        print("Start node:", self.current_robot_node)
        print("AStar path: ", self.a_star_path)


        return [int(m) for m in move]
        