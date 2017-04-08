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
        init_configs = [ [[ 1,0,0,-2.2],[0,1,0,2*i],[0,0,1,20.3]] for i in range(INSTANCE_NUM) ] # TODO: random init
        robots = HomogeneousRobotTeam(  env,
                                        INSTANCE_ROBOT,
                                        INSTANCE_NUM,
                                        TEMPORARY_LOCK_ROBOT, 
                                        [],
                                        (DOFAffine.X|DOFAffine.Y|DOFAffine.Z),
                                        'chain',
                                        init_configs )
        lock_robot = robots.lock( LOCK_ROBOT_TEMPLATE )
        robots.setPlanner(plannarPlanner, query)
        robots.planning()
        
        # instantitate single mode sampler
        # TODO: make mode binary so analytic
        sampler0 =  Sampler(mode=0)
        sampler1 =  Sampler(mode=1)
        sampler2 =  Sampler(mode=2)
        modal_samplers = [sampler0, sampler1, sampler2]

        # instantitate multi-modal sampler
        sampler01 = Sampler(mode=3, is_trans=True, pair=(0,1), pair_range=RADIUS*lock_robot*(instance_number-1)/10)
        sampler02 = Sampler(mode=4, is_trans=True, pair=(0,2), pair_range=RADIUS*lock_robot*(instance_number-1)/10)
        sampler12 = Sampler(mode=5, is_trans=True, pair=(1,2), pair_range=RADIUS*lock_robot*(instance_number-1)/10)
        trans_samplers = [sampler01, sampler02, sampler12]

        # multiModalPlanning
        isConnective = multiModalPlanner(query, lock_robot, modal_samplers, trans_samplers)
                                    
    raw_input("Press enter to exit...")
