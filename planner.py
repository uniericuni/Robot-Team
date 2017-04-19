# ====================================================================================
# Plannery.py
#   The main purpose of this module is to wrap different planner models into single 
#   functions.
# ====================================================================================
import numpy as np
import openravepy
import sys
import utility

from HomogeneousRobotTeam import *
from Graph import *
from Sampler import *
from config import *
from planning import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

# ====================================================================================
# astarPlanner(query, env, robot):
#   Astar for 2D.
# Inputs:
#   query: List or set of 1 or 2, (init and) goal configuration.
#   env:   Openrave environment object.
#   robot: Openrave robot object.
# Outputs:
#   rtn:   List of np.array, the trajectory of from init to goal.
# ====================================================================================
def astarPlanner(query, env, robot):

    # Input format check
    assert(evn!=None)
    assert(robot!=None)

    # Planner init
    if len(query)==1:
        init = robot.GetActiveDOFValues()
        goal = query
    else:
        init = query[0]
        goal = query[1]

    # Planner
    goalNode, explored, collided = astar(   init,                   # init config
                                            goal,                   # goal config
                                            utility.eDist,          # distance metric
                                            utility.eightConnected, # neighbor expansion
                                            robot, env)             # environment
    robot.SetActiveDOFValues(goalNode.pos)

    # Create trajectory
    rtn = []
    node = goalNode
    while node!=None:
        rtn.append(np.array(node.pos))
        node = node.prev
    rtn.append(np.array(start))
    return rtn[::-1]

# ====================================================================================
# rotationPlanner(query, env, robot, ee):
#   IK solver for 2D chain-bot.
# Inputs:
#   query: List or set of 1 or 2, (init and) goal configuration.
#   env:   Openrave environment object.
#   robot: Openrave robot object.
#   ee:    String, name for end-effector link.
# Outputs:
#   rtn:   List of np.array, the trajectory of from init to goal.
# ====================================================================================
def rotationPlanner(query, env, robot, ee=None):

    # Input format check
    assert(evn!=None)
    assert(robot!=None)

    # Planner init
    if len(query)==1:
        init = robot.GetActiveDOFValues()
        goal = query
    else:
        init = query[0]
        goal = query[1]
    if len(goal)==3:
        goal = goal[0:2]
    if ee==None:
        ee = robot.GetLinks()[-1]

    # Planner
    with env:
        
        # Iterate over all dof
        for dof in range(robot.GetActiveDOF()):

            min_val = float('inf')
            min_id = 0

            # Exhaust to minimize cost
            for i in range(-30,31):
                
                value = robot.GetActiveDOFValues()
                if dof==0:
                    value[dof] = i*PI/30
                else:
                    value[dof] = i*PI/180
                robot.SetActiveDOFValues(value)
                
                # Check self collision
                if robot.CheckSelfCollision():
                    continue

                # Check minimizer
                pos = np.array(robot.GetLink(ee).GetTransform())[0:2,3]
                if np.linalg.norm(pos-goal) < min_val:
                    min_val = np.linalg.norm(pos-query)
                    min_id = i
            
            # Set dof value
            value = robot.GetActiveDOFValues()
            if dof==0:
                value[dof] = min_id*PI/30
            else:
                value[dof] = min_id*PI/180
            robot.SetActiveDOFValues(value)

    # return trajectory, if fail, run again
    step = ( np.array(robot.GetActiveDOFValues()) - np.array(init) ) / ROT_RES
    rtn = [init+step*i for i in range(ROT_RES+1)]
    return rtn

# ====================================================================================
# rotationPlacer(query, env, robot, ee):
#   IK solver for transition from 2D chain-bot to unlocked.
# Inputs:
#   query: List or set of 1 or 2, (init and) goal configuration.
#   env:   Openrave environment object.
#   robot: Openrave robot object.
#   ee:    String, name for end-effector link.
# Outputs:
#   rtn:   List of np.array, the trajectory of from init to goal.
# ====================================================================================
def rotationPlacer(query, env, robot, ee=None):

    # Input format check
    assert(evn!=None)
    assert(robot!=None)

    # Planner init
    if len(query)==1:
        init = robot.GetActiveDOFValues()
        goal = query
    else:
        init = query[0]
        goal = query[1]
    if len(goal)==3:
        goal = goal[0:2]
    if ee==None:
        ee = robot.GetLinks()[-1]

    # Determine rotation direction
    rot_dir = np.where( np.array(init)>0, 1, -1 )

    # Planner
    with env:

        dof = 0
        robot.SetActiveDOFValues([0]*robot.GetActiveDOF())

        # Iterate over all dof to check if placed
        while dof < robot.GetActiveDOF():

            isPlaced = False

            # Exhaust to check placement
            for i in range(-30,31):
                
                value = robot.GetActiveDOFValues()
                if dof==0:
                    value[dof] += PI/30*rot_dir[i]
                else:
                    value[dof] += PI/180*rot_dir[i]
                robot.SetActiveDOFValues(value)
                
                # Check self collision
                if robot.CheckSelfCollision():
                    continue

                # Check placement
                pos = np.array(robot.GetLinks()[dof+1].GetTransform())[0:2,3]
                if not isRejected(pos, mode==UNLOCK):
                    isPlaced = True
                    break

            # If placed, next adjust next dof; if not, adjust pervious
            if isPlaced == True:
                dof += 1
            else:
                dof -= 1

    # Create trajectory
    step = ( np.array(robot.GetActiveDOFValues()) - np.array(init) ) / ROT_RES
    rtn = [init+step*i for i in range(ROT_RES+1)]
    return rtn

# ====================================================================================
# effMultiModalPlanner(query, robot_team, modal_samplers, trans_samplers):
# ====================================================================================
def effMultiModalPlanner(   query,
                            robot_team,
                            trans_samplers, pr0, pr1 ):

    # Initiate graphs
    num_of_modes = len(trans_samplers)
    map_unlock = Graph(query, 0)
    map_lock = Graph(query, 1)
    rtn = []

    # Add anchors for several times
    for i in range(MM_MAX_ITER):

        # Modal sampling
        rtn_tbl = {map_unlock.init_node:None}
        for j in range(MAX_SAMPLING_TIME):

            # Sampling for both sub-mode
            for sampler in trans_samplers:
                
                # Get transition mode pair
                mode0,mode1 = sampler.getTransPair()

                # Make sample
                sample_pair = sampler.makeSample()

                # Create nodes
                node0,node1 = nodePair( sample_pair[0],
                                        sample_pair[1],
                                        mode0, mode1 )

                # Add nodes to graphs
                if mode0==UNLOCK:
                    map_unlock.addNode(node0, pr=pr0)
                    map_lock.addNode(node1, pr=pr1, anchor=True)
                else:
                    map_unlock.addNode(node1, pr=pr0)
                    map_lock.addNode(node0, pr=pr1, anchor=True)

        # Test connectivity of anchors
        print isConnect(map_unlock.init_node, map_unlock.goal_node, rtn_tbl)
        graphPruning(map_unlock.init_node, map_unlock.goal_node, {})

    return map_unlock.init_node,map_unlock.goal_node

# ====================================================================================
# effMultiModalPlanner(query, robot_team, modal_samplers, trans_samplers):
# ====================================================================================

# ====================================================================================
# multiModalPlanner(query, robot_team, modal_samplers, trans_samplers):
# ====================================================================================
def multiModalPlanner(query, robot_team, modal_samplers, trans_samplers):

    # Initiate adjacent matrix to store connections and list of graph to store connection in one mode
    num_of_modes = len(modal_samplers)
    transition_map = [[{}]*num_of_modes for _ in range(num_of_modes)]
    maps = [Graph(query, 0)]
    for i in range(1,num_of_modes):
         maps.append( Graph(query,i) )

    # Connecting modes
    rtn = False
    for i in range(MM_MAX_ITER):                            # test M times
        for j in range(MAX_SAMPLING_TIME):                  # sampling N times

            # Modal Sampler
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

# ====================================================================================
# multiModalPlanner(query, robot_team, modal_samplers, trans_samplers)
# ====================================================================================

# =========================================================================
# Under Construction ...
# =========================================================================

# planner for 2D plannar trajectory
def plannarPlanner(query, env, robot):

    # generate the ik solver
    robot.SetActiveDOFValues([1,2])
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

# =========================================================================
# Under Construction ...
# =========================================================================
