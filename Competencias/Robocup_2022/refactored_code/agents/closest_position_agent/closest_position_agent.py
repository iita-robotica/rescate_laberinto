from agents.agent import Agent
import utilities

from agents.closest_position_agent.pathfinder import aStar, bfs, isTraversable


class ClosestPositionAgent(Agent):
    def __init__(self):
        super().__init__(["up", "down", "left", "right"])
        self.current_start = self.previous_start = None
        self.best_node = [None, None]
        self.a_star_path = []
        self.a_star_index = 0

    def find_start_node(self, grid):
        for y, row in enumerate(grid.grid):
            for x, node in enumerate(row):
                if node.is_robots_position:
                    return [x - grid.offsets[0], y - grid.offsets[1]]

    def get_best_node(self, possibleNodes, startNode, orientation):
        if len(possibleNodes) > 0:
            bestNode = possibleNodes[0]
            if bestNode[:2] == list(startNode):
                bestNode = possibleNodes[1]

            """
            for posNode in possibleNodes:
                diff = utilities.substractLists(startNode, posNode[:2])
                #print("Diff:", diff)
                #print("Multiplied orientation: ", multiplyLists(orientation, [-2, -2]))
                if posNode[2] > 1:
                    break
                
                elif diff == utilities.multiplyLists(orientation, [-2, -2]):
                    bestNode = posNode
                    break
            """
        else:
            bestNode = startNode
        #return possibleNodes[-1][:2]
        return bestNode[:2]
    
    def predict(self, grid):
        start_node = self.find_start_node(grid)
        
        if start_node != self.current_start:
            self.previous_start = self.current_start
            self.current_start = start_node
        if self.previous_start is None:
            self.previous_start = self.current_start

        if len(self.a_star_path) <= self.a_star_index:
            direction = utilities.substractLists(self.current_start, self.previous_start)
            if isTraversable(grid, self.current_start):
                possible_nodes = bfs(grid, self.current_start, 100)
            else:
                possible_nodes = bfs(grid, self.previous_start, 100)

            #print("Possible nodes:", possible_nodes)
            if len(possible_nodes):
                self.best_node = self.get_best_node(possible_nodes, self.current_start, direction)



            best_path = aStar(grid, self.current_start, self.best_node)

            if len(best_path) > 1:
                
                self.a_star_path = best_path[1:]
                self.a_star_index = 0

        for node in self.a_star_path:
            grid.get_node(node).mark1 = True
        grid.print_grid()

        move = utilities.substractLists(self.a_star_path[self.a_star_index], self.current_start)
        move = utilities.multiplyLists(move, [0.5, 0.5])

        if self.current_start == list(self.a_star_path[self.a_star_index]):
            self.a_star_index += 1

        print("Best node:", self.best_node)
        print("Start node:", self.current_start)
        print("AStar path: ", self.a_star_path)


        return [int(m) for m in move]
        