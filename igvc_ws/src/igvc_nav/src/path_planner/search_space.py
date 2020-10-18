"""

"""

from node import Node

class SearchSpace:

    def __init__(self, width, height):
        # Graph properties
        self.W = width
        self.H = height

        # Create a graph for searching
        self.grid = []

        # Populate the grid
        for i in range(self.W):
            row = []
            for j in range(self.H):
                row.append(Node(i,j))
            self.grid.append(row)

    def get_node(self, pos):
        """ Gets a node at a given (row, col) position from the grid """
        return self.grid[pos[0]][pos[1]]

    def get_successors(self, node):
        succ = []
        for i in range(node.row-1, node.row+2):
            for j in range(node.col-1, node.col+2):
                if i >= 0 and i < self.H and j >= 0 and j < self.W and (i != node.row or j != node.col):
                    succ.append(self.grid[i][j])

        return succ

    def print_search_space_rhs(self):
        for i in range(self.H):
            for j in range(self.W):
                rhs = self.grid[i][j].rhs
                if rhs < Node.INFINITY:
                    print("(" + str(i) + "," + str(j) + "): " + str(rhs))
        print "----"

    def get_search_space_rhs_map(self):
        rhs_map_data = [0] * self.W * self.H
        for i in range(self.H):
            for j in range(self.W):
                rhs = self.grid[i][j].rhs
                if rhs < Node.INFINITY:
                    rhs_map_data[(i * self.W) + j] = int(rhs)
        return rhs_map_data

    def load_search_space_from_map(self, map_data):
        for i in range(self.H):
            for j in range(self.W):
                cost = map_data[(self.H * i) + j]
                if cost >= 100 or cost < 0:
                    self.grid[i][j].set_cost(Node.INFINITY)
                else:
                    self.grid[i][j].set_cost(cost)

    def get_deleteable_nodes(self, start_node):
        # Define the lists for this search
        parent_set = {start_node}
        frontier = [start_node]
        explored_set = set()
        reset_set = set()

        # Expand the frontier using BFS until there is nothing left to explore
        while len(frontier) > 0:
            # Take the next node off the frontier list
            node = frontier.pop(0)
            explored_set.add(node)

            # Look through the successors of this node
            for succ in self.get_successors(node):
                # If this successor has a parent in the parent set, we cannot delete it
                # Since it is part of the subtree, we should add it to the parent set as well
                if succ.par in parent_set:
                    parent_set.add(succ)
                # Otherwise, this node is not in the subtree rooted at the start node, so add
                # it to the list of nodes to reset
                else:
                    reset_set.add(succ)

                # As long as the node has a non infinite RHS, we should add it to the frontier
                if succ.rhs < Node.INFINITY and (succ not in explored_set) and (succ not in frontier):
                    frontier.append(succ)

        print(len(reset_set), len(explored_set))

        # Return the nodes that can be reset as a list
        return list(reset_set)


    def update_map(self, new_map):
        """ updates the map for the search space """
        # Create a list for nodes with changed cost
        changed_nodes = []

        # Go through and update each cell
        for i in range(self.H):
            for j in range(self.W):
                # Get the old cost
                old_cost = self.grid[i][j].cost

                # Update the cost of the node
                if new_map[(self.H * i) + j] != 0:
                    self.grid[i][j].set_cost(Node.INFINITY)
                else:
                    self.grid[i][j].set_cost(0)

                # Detect if the node changed costs
                if self.grid[i][j].cost != old_cost:
                    changed_nodes.append(self.grid[i][j])

        # Return the nodes that have changed so the edge costs can be updated
        return changed_nodes

