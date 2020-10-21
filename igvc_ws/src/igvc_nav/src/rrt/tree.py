from rtree import index

class Tree(object):
    def __init__(self, X):
        """
        Tree representation
        :param X: Search space
        """
        p = index.Property()
        p.dimension = X.dimensions
        self.V = index.Index(interleaved=True, properties=p)
        self.V_count = 0
        self.E = {}
