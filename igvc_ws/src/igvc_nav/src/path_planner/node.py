"""

"""

class Node:

    INFINITY = 1000000000

    def __init__(self, row, col, cost = 0):
        self.row = row
        self.col = col

        # Default node value
        self.G = self.INFINITY
        self.rhs = self.INFINITY
        self.par = None
        self.key = (None, None)

        # Cost (for obstacles and such)
        self.cost = cost

    def set_g(self, G):
        if G > self.INFINITY:
            G = self.INFINITY
        self.G = G

    def set_rhs(self, rhs):
        if rhs > self.INFINITY:
            rhs = self.INFINITY
        self.rhs = rhs

    def set_par(self, par):
        self.par = par

    def set_key(self, key):
        self.key = key

    def set_cost(self, cost):
        self.cost = cost

    def __cmp__(self, other):
        """ Sort keys with lowest priority to the top of the list"""
        # Sort by the first key
        comp_val = cmp(self.key[0], other.key[0])
        if comp_val != 0:
            return comp_val

        # If there was a tie, use the second key as a tiebreaker
        return cmp(self.key[1], other.key[1])

    def __lt__(self, other):
        comp_val = (self.key[0] < other.key[0])

        if comp_val is True:
            return True
        elif self.key[0] == other.key[0]:
            return self.key[1] < other.key[1]

        return False

    def __gt__(self, other):
        comp_val = (self.key[0] > other.key[0])

        if comp_val is True:
            return True
        elif self.key[0] == other.key[0]:
            return self.key[1] > other.key[1]

        return False

    def __eq__(self, other):
        if other == None:
            return False
        return (self.row == other.row) and (self.col == other.col)

    def __hash__(self):
        return hash((self.row, self.col))
