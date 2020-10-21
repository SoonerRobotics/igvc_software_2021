import heapq
from node import Node


class OpenList:

    def __init__(self):
        """ Initialize the open list """
        # Set up an empty min heap using list and heapq
        self.min_heap = []
        heapq.heapify(self.min_heap)

    def update(self, node):
        """ Update a node entry in the open list """
        if node in self.min_heap:
            self.delete(node)
            self.insert(node)

    def insert(self, node):
        """ Add a node to the queue """
        # Push the node to the heap
        heapq.heappush(self.min_heap, node)

    def delete(self, node):
        """ Delete a node from the open list """
        if node in self.min_heap:
            self.min_heap.remove(node)
            heapq.heapify(self.min_heap)

    def top_key(self):
        """ Get the top value in the heap """
        if len(self.min_heap) == 0:
            return (Node.INFINITY, Node.INFINITY)
        # The top key will be the lowest priority
        return min(self.min_heap).key

    def top(self):
        """ Get the top value from the list """
        if len(self.min_heap) == 0:
            return None
        return self.min_heap[0]

    def contains(self, val):
        return val in self.min_heap
