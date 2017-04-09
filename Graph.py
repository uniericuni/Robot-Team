import numpy as np
import openravepy

from config import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

# bounding ball radius < range of cliff
BOUNDARY = CLIFF

# node in graph
class Node:
    
    def __init__(self, config):
        self.val = config
        self.neighbors = []

    def getVal(self):
        return self.val

    def extendNeighbors(self, node):
        self.neighbors.append(node)

# PRM graph
class Graph:

    def __init__(self):
        self.nodes = []

    # instantiate init/goal configuration as node an store
    def __init__(self, query):
        init_node = Node(query[0])
        goal_node = Node(query[1])
        self.nodes = [init_node, goal_node]
        self.init_node = init_node
        self.goal_node = goal_node

    # add config as node into graph and connect with nodes
    def addConfig(self, config, neighbors='ball'):
        self.addNode(Node(config), neighbors)

    # add node into graph and connect with other nodes
    def addNode(self, node, neighbors='ball'):

        # extend all neighbors nodes within a boundary ball
        # note: the boundary is set to l2-norm while the real case is l1-norm in supsapce
        if neighbors == 'ball':
            for v in self.nodes:
                if np.linalg.norm(v.getVal()-node.getVal()) < BOUNDARY:
                    node.extendNeighbors(v)
                    v.extendNeighbors(node)

        # add all nodes as neighbors
        if neighbors == 'all':
            for v in self.nodes:
                node.extendNeighbors(v)
                v.extendNeighbors(node)

        # append node to graph
        self.nodes.append(node)

# Test the connectivity to the goal_node
def isConnect(node, goal_node, visited={}):

    # goal test
    if node==goal_node:
        return True

    # expand neighbors 
    visited[node] = True
    rtn = False
    func = lambda x:np.linalg.norm(node.getVal()-x.getVal())
    sorted_neighbors = sorted(node.neighbors, key=func)
    for neighbor in sorted_neighbors:
        if neighbor in visited:
            continue
        rtn = rtn | isConnect(neighbor, goal_node, visited=visited)

    return rtn
