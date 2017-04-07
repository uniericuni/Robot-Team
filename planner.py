import numpy as np
import openravepy

from HomogeneousRobotTeam import *
from Graph import *
from config import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

# planner for 2D plannar trajectory
def plannarPlanner(query, env, robot):

    # generate the ik solver
    iktype = IkParameterization.Type.TranslationXY2D
    ikmodel = databases.inversekinematics.InverseKinematicsModel( robot,
                                                                  iktype=iktype )
    if not ikmodel.load():
        ikmodel.autogenerate()

    # generate ik solution solution
    with env:
        while True:
            ikparam = IkParameterization(query, iktype)
            solutions = ikmodel.manip.FindIKSolutions( ikparam, IkFilterOptions.CheckEnvCollisions )
            if solutions is not None and len(solutions)>0: break

    # return trajectory
    return solutions

# multi-modal planner
def multiModalPlanner(query, env, robot, modal_samplers, trans_samplers):

    # initiate adjacent matrix to store connections and list of graph to store connection in one mode
    # TODO: graph object, sampler object
    num_of_modes = len(modal_samplers)
    modes_map = [[0]*num_of_modes for _ in range(num_of_modes)]
    maps = [Graph()]*num_of_modes    

    # connecting modes
    for _ in range(MM_MAX_ITER):

        # sample each mode
        for i,sampler in enumerate(modal_samplers):
            sample = sampler.makeSample()
            node = Node(sample)
            maps[i].addNode(node)

        # sample transition
        for sampler in trans_samplers:
            mode0,mode1 = sampler.getTransPair()
            samples = sampler.makeSample()
            node0 = Node(samples[0])
            node1 = Node(samples[1])
            node0.extendNeighbors(node1)
            node1.extendNeighbors(node0)
            maps[mode0].addNode(node0, 'all')
            maps[mode1].addNode(node1, 'all')
            modes_map[mode0][mode1].extend([node0,node1])
            modes_map[mode1][mode0].extend([node1,node0])

        # connection test, return transition points if connected
        rtn = []
        if isConnect(query, maps, modes_map, rtn):
            return rtn

    return False
