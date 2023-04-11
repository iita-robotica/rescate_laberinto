

class BFSAlgorithm:
    def __init__(self, found_function) -> None:
        self.found_function = found_function
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]

    def get_neighbours(self, node):
        for a in self.adjacents:
            yield [node[0] + a[0], node[1] + a[1]]
    
    def bfs(self, array, start_node):
        open_list = []
        open_list.append(start_node)

        while len(open_list) > 0:
            node = open_list.pop(0)

            value = array[node[0], node[1]]

            if self.found_function(value):
                return node

            for n in self.get_neighbours(node):
                if not n in open_list:
                    open_list.append(n)


class NavigatingBFSAlgorithm:
    def __init__(self, found_function, traversable_function) -> None:
        self.found_function = found_function
        self.traversable_function = traversable_function
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]

    def get_neighbours(self, node):
        for a in self.adjacents:
            yield [node[0] + a[0], node[1] + a[1]]
    
    def bfs(self, found_array, traversable_array, start_node):
        open_list = []
        open_list.append(start_node)

        while len(open_list) > 0:
            node = open_list.pop(0)

            for n in self.get_neighbours(node):
                value = found_array[n[0], n[1]]

                if self.found_function(value):
                    return n
                
                if self.traversable_function(traversable_array[node[0], node[1]]):
                    if not n in open_list:
                        open_list.append(n)

