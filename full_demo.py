import xml.etree.ElementTree as ET
import numpy as np
import time
import openravepy

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

    # generate robot team
    query = np.array([-2.2, 0])
    with env:
        init_configs = [   [[ 1,0,0,-8.5],
                            [0,1,0,(i%12)*1.3-6.2],
                            [0,0,1,20.3]] for i in range(INSTANCE_NUM) ] # TODO: random init
        robots = HomogeneousRobotTeam(  env,
                                        INSTANCE_ROBOT,
                                        INSTANCE_NUM,
                                        TEMPORARY_LOCK_ROBOT, 
                                        [],
                                        (DOFAffine.X|DOFAffine.Y|DOFAffine.Z),
                                        'chain',
                                        init_configs )
       
    raw_input("Press enter to exit...")

    # =================================================
    # PHASE I: scan for transition configurations
    # =================================================

    with env:
        
        # instantitate single mode sampler
        # TODO: make mode binary so analytic
        query = [np.array([-5, 0, 20.3]), np.array([5, 0, 20.3])]
        sampler0 =  Sampler(mode=0)
        sampler1 =  Sampler(mode=1)
        sampler2 =  Sampler(mode=2)
        modal_samplers = [sampler0, sampler1, sampler2]
 
        # instantitate multi-modal sampler
        n = float(robots.instance_number-1)
        pr = (float(RADIUS*2)*(n-1)-TRANS_ERR)/100
        sampler01 = Sampler(mode=3, is_trans=True, pair=(0,1), pair_range=pr)
        sampler02 = Sampler(mode=4, is_trans=True, pair=(0,2), pair_range=pr)
        sampler12 = Sampler(mode=5, is_trans=True, pair=(1,2), pair_range=pr)
        trans_samplers = [sampler01, sampler02, sampler12]
 
        # multiModalPlanning
        rtn_tbl, init_node, goal_node = multiModalPlanner(query, robots, modal_samplers, trans_samplers)
 
        # demo samples
        print 'samples ...'
        print '-'*20
        node = goal_node
        while node!=None:
            print node.getVal()
            node = rtn_tbl[node]
 
        # collect transition pair
        node = goal_node
        anchors = [goal_node]
        while node!=None:
 
            # append init node
            if rtn_tbl[node] == None:
                anchors.append(node)
            
            # append anchors with transition configs
            if node.trans_pair != None:
                
                # heuristicly pruning transition to single mode config
                # TODO: use joint state instead of base-represented to prune
                # TODO: explaining transition pruning
                # TODO: not pruning mode-0-to-1 or 2
                if node.trans_pair.getVal()[0]*node.getVal()[0] < 0:
                    anchors.append(node)
                    anchors.append(node.trans_pair)
                    node = node.trans_pair
            node = rtn_tbl[node]
 
        # demo anchors
        anchors = anchors[::-1]
        print '\nanchors ...'
        print '-'*20
        for v in anchors:
            print v.getVal().tolist()
        
    raw_input("Press enter to exit...")

    # =================================================
    # PHASE II: cost evaluation and heuristic planning
    # =================================================

    # plan for each transition
    print '\nplanning ...'
    print '-'*20
    for i in range(len(anchors)-1):
        
        # get anchor pair as init and goal
        init_anchor,goal_anchor = anchors[i:i+2]
        query = [init_anchor.getVal(), goal_anchor.getVal()]
        mode0 = init_anchor.mode
        mode1 = goal_anchor.mode
        print 'from mode%d to mode%d'%(mode0,mode1), ' | query: ', init_anchor.getVal().tolist(), goal_anchor.getVal().tolist()


        query_pair = query
        query = query[1]

        # use UNLOCK planner to realize the path
        if mode0 == 0:
            with env:
                robots.unlock()
                robots.setPlanner(astarPlanner, query)
                robots.planning()

        # use LOCK_BASE0 planner to realize the path
        elif mode0 == 1:
            with env:
                robots.lock( LOCK_ROBOT_TEMPLATE, LOCK0 )
                robots.setPlanner(rotationPlanner, query)
            raw_input("Press enter to exit...")
            with env:
                robots.planning()

        '''
        # use LOCK_BASEN planner to realize the path
        elif mode0 == 2:
            robots.lock( LOCK_ROBOT_TEMPLATE, LOCK1 )
            robots.setPlanner(plannarPlanner, query)
            robots.planning()
        '''
        raw_input("Press enter to exit...")
                                    
    raw_input("Press enter to exit...")
