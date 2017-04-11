import numpy as np
import openravepy
import sys
import utility

from HomogeneousRobotTeam import *
from Graph import *
from config import *
from planning import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

# astar for 2D plannar trajectory
def astarPlanner(query, env, robot):
    start = query[0]
    goal = query[1]
    distMethod = utility.eDist
    expandMethod = utility.eightConnected
    goalNode, explored, collided = astar(start, goal, distMethod, expandMethod, robot, env)
    robot.SetActiveDOFValues(goalNode.pos)

    # parse trajectory
    # Create trajectory
    '''
    traj = RaveCreateTrajectory(env, '')
    config = robot.GetActiveConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    node = goalNode
    while node!=None:
        step = list(node.pos)+[TIME_DELTA]
        traj.Insert(0, step)
        node = node.prev
    planningutils.RetimeActiveDOFTrajectory(traj, robot)

    # Init controller 
    controller = robot.GetController()
    controller.SetPath(traj)
    '''

# planner for 2D plannar trajectory
def plannarPlanner(query, env, robot):

    # generate the ik solver
    robot.SetActiveDOFValues([1,2])
    iktype = IkParameterization.Type.TranslationXY2D
    ikmodel = databases.inversekinematics.InverseKinematicsModel( robot,
                                                                  iktype=iktype )
    if not ikmodel.load():
        ikmodel.autogenerate()

    raw_input("Press enter to exit...")
    # generate ik solution solution
    with env:
        while True:
            ikparam = IkParameterization(query, iktype)
            solutions = ikmodel.manip.FindIKSolutions( ikparam, IkFilterOptions.CheckEnvCollisions )
            if solutions is not None and len(solutions)>0: break

    # return trajectory
    return solutions

# homemade planner
def rotationPlanner(query, env, robot, ee):

    # pruning redundant dimension
    if len(query)==3:
        query = query[0:2]

    with env:
        for dof in range(robot.GetActiveDOF()):

            min_val = float('inf')
            min_id = 0
            pos = np.array(robot.GetLinks()[dof].GetTransform())[0:2,3]

            # iteratively check for best
            for i in range(36):
                
                value = robot.GetActiveDOFValues()
                if dof==0:
                    value[dof] = i*PI/18
                else:
                    value[dof] = i*PI/36
                robot.SetActiveDOFValues(value)
                
                # check self collision
                if robot.CheckSelfCollision():
                    continue

                # iteratively check for best
                if np.linalg.norm(pos-query) < min_val:
                    min_val = np.linalg.norm(pos-query)
                    min_id = i
            
            # set dof value
            value = robot.GetActiveDOFValues()
            if dof==0:
                value[dof] = min_id*PI/18
            else:
                value[dof] = min_id*PI/36
            robot.SetActiveDOFValues(value)

    # return trajectory, if fail, run again
    pos = np.array(robot.GetLink(ee).GetTransform())[0:2,3]
    return robot.GetActiveDOFValues()

    '''
    if np.linalg.norm(pos-query) < ROT_ERR:
        return robot.GetActiveDOFValues()
    else:
        raw_input("Press enter to exit...")
        rotationPlanner(query, env, robot, ee)
    '''

# multi-modal planner
def multiModalPlanner(query, robot_team, modal_samplers, trans_samplers):

    # initiate adjacent matrix to store connections and list of graph to store connection in one mode
    num_of_modes = len(modal_samplers)
    transition_map = [[{}]*num_of_modes for _ in range(num_of_modes)]
    maps = [Graph(query, 0)]
    for i in range(1,num_of_modes):
         maps.append( Graph(query,i) )

    # connecting modes
    rtn = False
    for i in range(MM_MAX_ITER):                            # test M times
        for j in range(MAX_SAMPLING_TIME):                  # sampling N times

            # sample each mode
            for k,sampler in enumerate(modal_samplers):

                # sample single configs for the BASE for UNLCOK
                if sampler.mode==0:
                    sample = sampler.makeSample()
                    maps[k].addConfig(sample)
                
                # sample base configs for LOCK_Base0, LOCK_BaseN
                else:
                    sample = sampler.makeSample()
                    maps[k].addConfig(sample)

                sys.stdout.flush()

            # sample transition
            for sampler in trans_samplers:
                
                # add transitions samples to map
                mode0,mode1 = sampler.getTransPair()        # make potentially connectable transition sample pairs
                sample_pair = sampler.makeSample()
                node0,node1 = nodePair( sample_pair[0],     # instantiate node pair
                                        sample_pair[1],
                                        mode0, mode1) 
                maps[mode0].addNode(node0)                  # add node pair to map
                maps[mode1].addNode(node1) 

                # add transitions sample to dictionary 
                transition_map[mode0][mode1][node0] = [node0,node1]
                transition_map[mode1][mode0][node1] = [node1,node0]

        # connection test, return transition points if connected
        rtn_tbl = {maps[0].init_node:None}
        if isConnect(maps[0].init_node, maps[0].goal_node, rtn_tbl): break

    return rtn_tbl,maps[0].init_node,maps[0].goal_node

