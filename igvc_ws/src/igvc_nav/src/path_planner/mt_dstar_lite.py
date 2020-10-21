
from math import sqrt

from node import Node
from search_space import SearchSpace
from open_list import OpenList


# Algorithm: http://idm-lab.org/bib/abstracts/papers/aamas10a.pdf
class mt_dstar_lite:

    def __init__(self):
        """ Initialize the Moving Target D* Lite algorithm """
        # Moving Target
        self.km = 0
        self.search_space = None

        # Lists
        self.open_list = None
        self.deleted_list = None
        self.path = None

        # Nodes
        self.start_node = None
        self.goal_node = None
        self.old_start = None
        self.old_goal = None


    def initialize(self, width, height, robot_pos, goal_pos, map_data):
        """ Initialize the Moving Target D* Lite algorithm """
        # Search variables
        self.km = 0
        self.open_list = OpenList()
        self.deleted_list = []
        self.path = []

        # Create a search space
        self.search_space = SearchSpace(width, height)
        self.search_space.load_search_space_from_map(map_data)

        # Get the node the robot is on (row, col is the pos argument)
        self.start_node = self.search_space.get_node(robot_pos)

        # Get the node the goal is on (row, col is the pos argument)
        self.goal_node = self.search_space.get_node(goal_pos)

        # Set the robot's node's rhs = 0
        self.start_node.set_rhs(0)

        # Add the robot's node to the open list
        self.start_node.set_key(self.calculate_key(self.start_node))
        self.open_list.insert(self.start_node)


    def calculate_key(self, node):
        """ Calculates a node key based on its G and RHS values, as well as the distance to the goal """
        key_1 = min(min(node.G, node.rhs) + self.heuristic(node, self.goal_node) + self.km, Node.INFINITY)
        key_2 = min(min(node.G, node.rhs), Node.INFINITY)
        return (key_1, key_2)


    def heuristic(self, node1, node2):
        """ Compute the distance from one node to another """
        return sqrt((node2.row - node1.row)**2 + (node2.col - node1.col)**2)

    def cost(self, node1, node2):
        """ Computer actual cost incurred by moving from one node to another """
        if node1.cost < Node.INFINITY and node2.cost < Node.INFINITY:
            return self.heuristic(node1, node2) + node1.cost + node2.cost
        else:
            return Node.INFINITY


    def update_state(self, node):
        """ Updates a node's status based on the progression of the search """
        # If the node is on the open list and is inconsistent, update it
        if node.G != node.rhs and self.open_list.contains(node):
            node.set_key(self.calculate_key(node))
            self.open_list.update(node)

        # If the node is not on the open list, but is inconsistent, then add it to the open list
        elif node.G != node.rhs and not self.open_list.contains(node):
            node.set_key(self.calculate_key(node))
            self.open_list.insert(node)

        # If the node is on the open list and is consistent, close it by removing it from the open list
        elif node.G == node.rhs and self.open_list.contains(node):
            self.open_list.delete(node)


    def compute_cost_minimal_path(self):
        """ Finds the best path from the start state to the goal state """
        while (self.open_list.top_key() < self.calculate_key(self.goal_node)) or (self.goal_node.rhs > self.goal_node.G):
            # Get the highest priority node from the open list
            u_node = self.open_list.top()

            # Compute keys for this node
            old_key = u_node.key
            new_key = self.calculate_key(u_node)

            # If the new key is greater than the old key, update the open list
            if old_key < new_key:
                u_node.set_key(new_key)
                self.open_list.update(u_node)

            # Otherwise, if the node's G value is higher than its RHS value, close the node and update its successors
            elif u_node.G > u_node.rhs:
                # Make the node consistent and remove it from the open list
                u_node.set_g(u_node.rhs)
                self.open_list.delete(u_node)

                # Update successors
                for s_node in self.search_space.get_successors(u_node):
                    if s_node != self.start_node and s_node.rhs > u_node.G + self.cost(u_node, s_node):
                        s_node.set_rhs(u_node.G + self.cost(u_node, s_node))
                        s_node.set_par(u_node)
                        self.update_state(s_node)


            # Otherwise make the node inconsistent and calculate the parent nodes of its successors
            else:
                # Set G value to infinity
                u_node.set_g(Node.INFINITY)

                # Update successors
                succ = self.search_space.get_successors(u_node)
                succ.append(u_node)
                for s_node in succ:
                    if s_node != self.start_node and s_node.par == u_node:
                        # Set the RHS to be the minimum one-step lookahead
                        s_rhs = min([node.G + self.cost(node, s_node) for node in self.search_space.get_successors(s_node)])
                        s_node.set_rhs(s_rhs)

                        if s_node.rhs >= Node.INFINITY:
                            s_node.set_rhs(Node.INFINITY)
                            s_node.set_par(None)
                        else:
                            # Set the parent node pointer based on the parent node that has the minimum path cost to goal from this node
                            par_nodes = [node for node in self.search_space.get_successors(s_node)]
                            s_par = [node.G + self.cost(node, s_node) for node in par_nodes]
                            s_par_node_idx = s_par.index(min(s_par))
                            s_par_node = par_nodes[s_par_node_idx]
                            s_node.set_par(s_par_node)
                    self.update_state(s_node)


    def optimized_deletion(self):
        """ """
        # Initialize deletion
        self.deleted_list = []
        self.start_node.set_par(None)

        # Go through the nodes in the search tree that aren't on the path from the current start node to the goal node
        for s_node in self.search_space.get_deleteable_nodes(self.start_node):
            # Reset the node's data
            s_node.set_par(None)
            s_node.set_rhs(Node.INFINITY)
            s_node.set_g(Node.INFINITY)

            # Get this node off the open list and add it to the deleted list
            if self.open_list.contains(s_node):
                self.open_list.delete(s_node)

            # Add this node to the deleted list
            self.deleted_list.append(s_node)

        # Go through the deleted list and update its costs
        for node in self.deleted_list:
            # Get the best cost from the successors
            for succ_node in self.search_space.get_successors(node):
                if node.rhs > succ_node.G + self.cost(succ_node, node):
                    node.set_rhs(succ_node.G + self.cost(succ_node, node))
                    node.set_par(succ_node)

            # Add the node to the open list if it has a finite cost
            if node.rhs < Node.INFINITY:
                node.set_key(self.calculate_key(node))
                self.open_list.insert(node)


    def get_best_path(self):
        """ Get the best path using the parent pointers """
        # Start at the start and keep track of the footsteps
        node = self.goal_node
        path = [(node.row, node.col)]

        # Go until the goal is found
        while node != self.start_node:
            # Add the next node to the path
            next_node = node.par
            path.append((next_node.row, next_node.col))
            node = next_node

        # The best path should be found by traversing the pointers
        return path



    # NOTE: we assume that the hunter never catches the target. We also assume that the edge costs change
    # during every configuration space update for now - the SLAM node will (in the future) provide new maps only when the
    # robot has moved. Since the target cannot move off the path unless the map updates, and the map only updates to show new
    # edge costs, we don't need to check for the conditions in the while loop that waits for the hunter to follow the target,
    # as described in the original algorithm

    def plan(self):
        """ Plan the robot's initial path """
        # Set the last robot positions
        self.old_start = self.start_node
        self.old_goal = self.goal_node

        # Find a low cost path from the start to the goal
        self.compute_cost_minimal_path()

        # If the goal has a RHS of infinity, then there is no path
        if self.goal_node.rhs >= Node.INFINITY:
            #self.search_space.print_search_space_rhs()
            print("Get rekt " + str(self.goal_node.rhs))
            return None

        # Figure out path by following the parent pointers (par) to the goal node
        self.path = self.get_best_path()
        return self.path

    # NOTE: Per the note above, we must unwrap the main while loop since there is a discontinuity caused by the "hunter follows target"
    # while loop. Therefore, plan() should only be called by an external class the first time we want to plan,
    # and all subsequent external planning requests should be to the following replan() function

    def replan(self, robot_pos, goal_pos, new_map):
        # Get the node the robot is on (row, col is the pos argument)
        self.start_node = self.search_space.get_node(robot_pos)

        # Get the node the goal is on (row, col is the pos argument)
        self.goal_node = self.search_space.get_node(goal_pos)

        # Update the km search parameter
        self.km = self.km + self.heuristic(self.goal_node, self.old_goal)

        # If the robot has moved, delete the nodes that need updating and revalue them
        if self.old_start != self.start_node:
            # Perform optimized deletion to update the out-of-date parts of the search tree
            self.optimized_deletion()

            # Shift the map anyways because our environment grows
            # TODO

        # Update all changed edge costs from the map update
        for c_node in self.search_space.update_map(new_map):
            for succ in self.search_space.get_successors(c_node):
                # If the old cost was bigger than the new cost (i.e., the current cost is low now)
                if c_node.cost < Node.INFINITY:
                    if succ != self.start_node and succ.rhs > c_node.G + self.cost(c_node, succ):
                        succ.set_par(c_node)
                        succ.set_rhs(c_node.G + self.cost(c_node, succ))
                # Otherwise the cost has increased and we need to reroute the successors that map to this node
                else:
                    # Only update successors that map to the current node (and that aren't the start node)
                    if succ != self.start_node and succ.par is c_node:
                        # Set the RHS to be the minimum one-step lookahead
                        succ_rhs = min([node.G + self.cost(node, succ) for node in self.search_space.get_successors(succ)])
                        succ.set_rhs(succ_rhs)

                        if succ.rhs >= Node.INFINITY:
                            succ.set_rhs(Node.INFINITY)
                            succ.set_par(None)
                        else:
                            # Set the parent node pointer based on the parent node that has the minimum path cost to goal from this node
                            par_nodes = [node for node in self.search_space.get_successors(succ)]
                            s_par = [node.G + self.cost(node, succ) for node in par_nodes]
                            s_par_node_idx = s_par.index(min(s_par))
                            s_par_node = par_nodes[s_par_node_idx]
                            succ.set_par(s_par_node)

                        # update the state of the successor
                        self.update_state(succ)

        # Plan the path after adjustments
        return self.plan()

    def get_search_space_map(self):
        return self.search_space.get_search_space_rhs_map()
