# ====================================================================================
# Graph.py
#   Graph module describes node, connectitivity of the nodes in macro-planner.
# ====================================================================================
import numpy as np
import openravepy

from config import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

# bounding ball radius < range of cliff
BOUNDARY = CLIFF

# ====================================================================================
# Node:
# ====================================================================================
class Node:
    
    def __init__(self, config, mode, transition_pair=None):
        self.val = config
        self.neighbors = []
        self.trans_pair = transition_pair
        self.mode = mode

    def getVal(self):
        return self.val

    def extendNeighbors(self, node):
        self.neighbors.append(node)

# ====================================================================================
# PRM Graph
# ====================================================================================
class Graph:

    def __init__(self):
        self.nodes = []

    # instantiate init/goal configuration as node an store
    def __init__(self, query, mode):
        self.mode = mode
        init_node = Node(query[0], mode)
        goal_node = Node(query[1], mode)
        self.nodes = [init_node, goal_node]
        self.init_node = init_node
        self.goal_node = goal_node

    # add config as node into graph and connect with nodes
    def addConfig(self, config, neighbors='ball'):
        self.addNode(Node(config, self.mode), neighbors)

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

# ====================================================================================
# nodePair( config0, config1, mode0, mode1 ):
# ====================================================================================
def nodePair( config0, config1, mode0, mode1 ):
    node0 = Node(config0, mode0)
    node1 = Node(config1, mode1)
    node0.trans_pair = node1
    node1.trans_pair = node0
    node0.extendNeighbors(node1)
    node1.extendNeighbors(node0)
    return node0,node1

# ====================================================================================
# isConnect(node, goal_node, visited):
#   Test the connectivity to the goal_node
# ====================================================================================
# TODO: make it non-recursive
def isConnect(node, goal_node, visited):

    # goal test
    if node==goal_node:
        return True

    # expand neighbors 
    rtn = False
    func = lambda x:np.linalg.norm(node.getVal()-x.getVal())
    sorted_neighbors = sorted(node.neighbors, key=func)
    for neighbor in sorted_neighbors:
        if neighbor in visited:
            continue
        visited[neighbor] = node
        rtn = isConnect(neighbor, goal_node, visited=visited)
        if rtn:
            break 
    return rtn
