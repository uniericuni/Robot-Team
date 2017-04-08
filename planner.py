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
def multiModalPlanner(query, robot_team, modal_samplers, trans_samplers):

    # initiate adjacent matrix to store connections and list of graph to store connection in one mode
    num_of_modes = len(modal_samplers)
    transition_map = [[{}]*num_of_modes for _ in range(num_of_modes)]
    maps = [Graph(query)] + [Graph]*(num_of_modes-1)

    # connecting modes
    rtn = False
    for i in range(MM_MAX_ITER):                            # test M times
        for j in range(MAX_SAMPLING_TIME):                  # sampling N times
            
            # sample each mode
            for k,sampler in enumerate(modal_samplers):

                # sample joint configs for UNLCOK
                if sampler.mode==0:
                    sample = np.array([sampler.makeSample()]*robot_team.instance_number)
                    shape = np.shape(sample)
                    sample = np.reshpae( sample, (shape[0]*shape[1],1) )
                    maps[k].addConfig(sample)
                
                # sample base configs for LOCK_Base0, LOCK_BaseN
                else:
                    sample = sampler.makeSample()
                    maps[k].addConfig(sample)
 
            # sample transition
            for sampler in trans_samplers:
                
                # add transitions samples to map
                mode0,mode1 = sampler.getTransPair()        # make potentially connectable transition sample pairs
                sample_pair = sampler.makeSample()
                node0 = Node(sample_pair[0])                # instantiate node pair
                node1 = Node(sample_pair[1]) 
                node0.extendNeighbors(node1)                # connect node pair
                node1.extendNeighbors(node0) 
                maps[mode0].addNode(node0)                  # add node pair to map
                maps[mode1].addNode(node1) 

                # add transitions sample to dictionary 
                transition_map[mode0][mode1][node0] = [node0,node1]
                transition_map[mode1][mode0][node1] = [node1,node0]

        # connection test, return transition points if connected
        rtn = []
        init_node = Node(query[0])
        goal_node = Node(query[1])
        if isConnect(init_node, goal_node):
            rtn = True

    return rtn
