class PathSmoother:
    def __init__(self, strenght) -> None:
        self.strenght = strenght

    def smooth(self, path):
        new_path = []
        for index, node in enumerate(path):
            prior = path[max(index - 1, 0)]
            next = path[min(index + 1, len(path) - 1)]

            avg_x = (node[0] + prior[0] * self.strenght + next[0] * self.strenght) / (1 + self.strenght * 2)
            avg_y = (node[1] + prior[1] * self.strenght + next[1] * self.strenght) / (1 + self.strenght * 2)

            new_path.append([avg_x, avg_y])
        
        return new_path