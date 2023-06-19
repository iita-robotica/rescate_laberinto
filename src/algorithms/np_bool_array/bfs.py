import numpy as np

import math

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
    def __init__(self, found_function, traversable_function, max_result_number=1) -> None:
        self.found_function = found_function
        self.traversable_function = traversable_function
        self.adjacents = ((0, 1), (0, -1), (-1, 0), (1, 0))
        self.max_result_number = max_result_number

    def get_neighbours(self, node):
        for a in self.adjacents:
            yield (node[0] + a[0], node[1] + a[1])
    
    def bfs(self, found_array, traversable_array, start_node):
        open_list = []
        open_list.append(tuple(start_node))

        closed_set = set()
        closed_set.add(tuple(start_node))

        results = []

        while len(open_list) > 0:
            node = open_list.pop(0)

            if node[0] < 0 or node[1] < 0 or node[0] >= traversable_array.shape[0] or node[1] >= traversable_array.shape[1]:
                continue

            if not self.traversable_function(traversable_array[node[0], node[1]]):
                continue

            value = found_array[node[0], node[1]]

            if self.found_function(value):
                results.append(node)
                if len(results) >= self.max_result_number:
                    return results

            for n in self.get_neighbours(node):
                if n not in closed_set:
                    open_list.append(n)
                    closed_set.add(n)    
        
        return results
    
class NavigatingLimitedBFSAlgorithm:
    def __init__(self, found_function, traversable_function, max_result_number=1, limit=math.inf) -> None:
        self.limit = limit
        self.found_function = found_function
        self.traversable_function = traversable_function
        self.adjacents = ((0, 1), (0, -1), (-1, 0), (1, 0))
        self.max_result_number = max_result_number

    def get_neighbours(self, node):
        for a in self.adjacents:
            yield (node[0] + a[0], node[1] + a[1])
    
    def bfs(self, found_array, traversable_array, start_node):
        self.loops = 0
        open_list = []
        open_list.append(tuple(start_node))

        closed_set = set()
        closed_set.add(tuple(start_node))

        results = []

        while len(open_list) > 0:
            self.loops += 1
            if self.loops > self.limit:
                break
            node = open_list.pop(0)

            if node[0] < 0 or node[1] < 0 or node[0] >= traversable_array.shape[0] or node[1] >= traversable_array.shape[1]:
                continue

            if not self.traversable_function(traversable_array[node[0], node[1]]):
                continue

            value = found_array[node[0], node[1]]

            if self.found_function(value):
                results.append(node)
                if len(results) >= self.max_result_number:
                    return results

            for n in self.get_neighbours(node):
                if n not in closed_set:
                    open_list.append(n)
                    closed_set.add(n)    

            
        
        return results

