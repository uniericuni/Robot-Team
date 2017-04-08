import numpy as np
import openravepy

from HomogeneousRobotTeam import *
from config import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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
        self.V = []

    def __init__(self, query_nodes):
        self.V = query_nodes
        self.init_node = query_nodes[0]
        self.goal_node = query_nodes[1]

    # add node into graph and connect with KNN
    def addNode(self, node, neighbors='ball'):

        # extend all neighbors nodes within a boundary ball
        if neighbors == 'ball':
            for v in self.V:
                if np.linalg.norm(v.val-node.val) < BOUNDARY:
                    node.extendNeighbors(v)

        # add all nodes as neighbors
        if neighbors == 'all':
            for v in self.V:
                node.extendNeighbors(v)

        # append node to graph
        self.V.append(node)

    # TODO: return transition nodes, should do exhausted search?
    def isConnect(visited={}, node=self.goal_node):

        # goal test
        if node==self.init_node:
            return True

        # expand neighbors 
        visited[node] = True
        rtn = False
        for neighbor  in node.neighbors:
            if neighbor in visited:
                continue
            rtn = rtn | self.isConnect(visited=visited, node=neighbor)
        return rtn
