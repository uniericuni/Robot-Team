import xml.etree.ElementTree as ET
import numpy as np
import time
import openravepy
import heapq as hq

from HomogeneousRobotTeam import *
from config import *
from Planner import *
from Sampler import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    env.Load('data/robotTeamTest.env.xml')
    time.sleep(0.1)
    final_task = [ np.array([8.5, (i%14)*1.2-6.5, 20.3]) for i in range(INSTANCE_NUM) ]

    # generate robot team
    query = np.array([-2.2, 0])
    with env:
        init_configs = [   [[ 1,0,0,-8.5],
                            [0,1,0,(i%14)*1.2-6.5],
                            [0,0,1,20.3]] for i in range(INSTANCE_NUM) ] # TODO: random init
        robots = HomogeneousRobotTeam(  env,
                                        INSTANCE_ROBOT,
                                        INSTANCE_NUM,
                                        TEMPORARY_LOCK_ROBOT, 
                                        LOCK_ROBOT_TEMPLATE,
                                        [],
                                        (DOFAffine.X|DOFAffine.Y|DOFAffine.Z),
                                        'chain',
                                        init_configs )
       
    raw_input("Press enter to exit...")

    # =================================================
    # PHASE I: scan for transition configurations
    # =================================================

    with env:
    
        # instantitate multi-modal sampler
        query = [np.array([-5, 0, 20.3]), np.array(final_task[0])]
        n = float(robots.instance_number-1)
        pr = (float(RADIUS*2)*(n-2))/100
        pair0 = (UNLOCK,LOCK0)
        pair1 = (UNLOCK,LOCKN)
        sampler01 = Sampler(mode=3, is_trans=True, pair=pair0, pair_range=pr)
        sampler02 = Sampler(mode=4, is_trans=True, pair=pair1, pair_range=pr)
        trans_samplers = [sampler01]
        #trans_samplers = [sampler01, sampler02]

        # multiModalPlanning
        init_node, goal_node = effMultiModalPlanner(   query,
                                                       robots,
                                                       trans_samplers,
                                                       CLIFF,
                                                       pr)

        # heuristic search
        frontier = [(0, init_node)]
        parent = {init_node:(0,None)}
        while True:
            cost,node = hq.heappop(frontier)
            if node==goal_node:
                break

            for neighbor in node.neighbors:
                parent[neighbor] = (cost,node)
                if node.mode==UNLOCK and neighbor.mode==UNLOCK:
                    total_cost = cost+np.linalg.norm(neighbor.getVal()-node.getVal())
                elif (node.mode==LOCK0 or node.mode==LOCKN) and (neighbor.mode==LOCK0 or neighbor.mode==LOCKN):
                    total_cost = cost+np.linalg.norm(neighbor.getVal()-node.getVal())
                else:
                    total_cost = 4+cost
                hq.heappush( frontier,
                             (total_cost, neighbor) )
        
        # display search result
        print 'samples ...'
        print '-'*20
        anchors = []
        while node!=None:
            print "config: ", node.getVal(), " | mode: ", node.mode, " | cost: ", cost
            anchors.append(node)
            cost,node = parent[node]
        anchors = anchors[::-1]
                    
    # =================================================
    # PHASE II: cost evaluation and heuristic planning
    # =================================================

    # Plan for transition and section
    for i in range(len(anchors)-1):
    
        # Get init,goal configuratino pair
        init_anchor,goal_anchor = anchors[i:i+2]
        query = [goal_anchor.getVal()]
        mode0 = init_anchor.mode
        mode1 = goal_anchor.mode
        print '\nfrom mode%d to mode%d'%(mode0,mode1), '\nquery: ', init_anchor.getVal().tolist(), goal_anchor.getVal().tolist()
        print '-'*20

        # Planner for section
        if mode0==mode1:
            if mode0==UNLOCK:
                with env:
                    robots.setPlanner(astarPlanner, query)
                    robots.planning()
                robots.release()
            elif mode0==LOCK0:
                with env:
                    robots.setPlanner(rotationPlanner, query)
                    robots.planning()
                robots.release()
                robots.lock( enforced=False )
            raw_input("Press enter to exit...")
 
        # Planner for transition
        else:
            if mode1==UNLOCK:                                   # from lock to unlock
                with env:
                    robots.setPlanner(rotationPlacer, [])
                    robots.planning()
                robots.release()
                raw_input("Press enter to exit...")
                robots.unlock(enforced=True)
            else:
                robots.lock(enforced=True)                      # from unlock to lock
            raw_input("Press enter to exit...")

    raw_input("Press enter to exit...")
