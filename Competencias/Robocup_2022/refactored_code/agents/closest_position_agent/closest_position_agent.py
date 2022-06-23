from agents.agent import Agent

class ClosestPositionAgent(Agent):
    def __init__(self):
        super().__init__(["up", "down", "left", "right"])
    
    def predict(self, grid) -> str:
        possible_nodes = bfs(grid)
        