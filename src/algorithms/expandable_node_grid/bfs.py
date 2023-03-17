from algorithms.expandable_node_grid.traversable import is_traversable

def is_bfs_addable(grid, index):
    node = grid.get_node(index, expand=False, phantom=True)
    if node.node_type == "vortex":
        for adj in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            adjacent = [index[0] + adj[0], index[1] + adj[1]]
            if not grid.get_node(adjacent, expand=False, phantom=True).explored:
                return True
        return False
    else:
        return False

# Breath First Search algorithm
# Returns the tiles in order and with the distance of each one
def bfs(grid, start, limit="undefined"):
    visited = []
    queue = []
    found = []
    start = [start[0], start[1], 0]
    visited.append(start)
    queue.append(start)
    while queue:
        if len(found) > 100:
            break
        coords = queue.pop(0)
        y = coords[1]
        x = coords[0]
        dist = coords[2]
        if limit != "undefined":
            if dist > limit:
                break
        
        if is_bfs_addable(grid, coords[:2]):
            found.append(coords)

        for newPosition in ((0, 1), (0, -1), (-1, 0), (1, 0)):
            neighbour = [x + newPosition[0] * 2, y + newPosition[1] * 2, dist + 1]
            inList = False
            for node in visited:
                if node[0] == neighbour[0] and node[1] == neighbour[1]:
                    inList = True
                    break
            if inList:
                continue

            # Make sure walkable terrain
            if is_traversable(grid, neighbour[:2]):
                visited.append(neighbour)
                queue.append(neighbour)

    return found