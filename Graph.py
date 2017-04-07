import numpy as np
import openravepy

from sklearn.neighbors import KDTree
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
        self.k = 10

    # add node into graph and connect with KNN
    def addNode(self, node, neighbors='k'):

        # TODO: KNN
        # TODO: rejection connection
        pass

        # add all nodes as neighbors
        if neighbors = 'all':
            for v in self.V:
                node.extendNeighbors(v)

        # append node to graph
        self.V.append(node)

def isConnect(query, maps, modes_map, rtn):

    # hash visited nodes
    visited = {}

    # DFS tests connectivity
    pass

