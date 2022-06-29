def is_traversable(grid, index):
    node = grid.get_node(index, expand=False, phantom=True)
    if node.node_type == "vortex":
        if node.status == "occupied":
            return False
        traversable = True
        for adjacentIndex in ((-1, 1), (1, -1), (1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)):
            adjacent = grid.get_node((index[0] + adjacentIndex[0], index[1] + adjacentIndex[1]), expand=False, phantom=True)
            
            if adjacent.node_type == "tile":
                if adjacent.tile_type == "hole" or adjacent.status == "occupied":
                    traversable = False
                    
            elif adjacent.node_type == "wall":
                if adjacent.status == "occupied":
                    traversable = False
            else:
                raise ValueError((f"invalid instance: {node.node_type}"))
        return traversable
    else:
        raise ValueError((f"invalid instance: {node.node_type}"))