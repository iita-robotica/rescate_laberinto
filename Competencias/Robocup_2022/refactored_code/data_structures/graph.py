class Graph:
    def __init__(self) -> None:
        self.graph = {}
    
    def isAdjacent(self, pos1, pos2):
        diff = []
        for p1, p2 in zip(pos1, pos2):
            diff = p1 - p2
            if diff > 1 or diff < -1:
                return False
        return True

    def block(self, pos1, pos2):
        if pos1 in self.graph:
            self.graph[pos1].append(pos2)
        else:
            self.graph[pos1] = [pos2, ]
        if pos2 in self.graph:
            self.graph[pos2].append(pos2)
        else:
            self.graph[pos2] = [pos1, ]
    
    def check(self, pos1, pos2):
        if not self.isAdjacent(pos1, pos2):
            return False
        if pos1 in self.graph:
            return pos2 not in self.graph[pos1]
        else:
            return True
    
    def unblock(self, pos1, pos2):
        if pos1 in self.graph:
            self.graph[pos1].remove(pos2)
        if pos2 in self.graph:
            self.graph[pos2].remove(pos2)

if __name__ == "__main__":
    my_graph = Graph()

    my_graph.block((0, 0), (0, 1))

    print(my_graph.graph)

    print("puedo pasar?", my_graph.check((0,2), (0,1)))