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


    found = 0
    for _ in range(100):

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
            init_node, goal_node, c = effMultiModalPlanner(query,
                                                           robots,
                                                           trans_samplers,
                                                           CLIFF,
                                                           pr)
            if c: found += 1

    print MAX_SAMPLING_TIME, float(found)/float(100)

