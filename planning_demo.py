import xml.etree.ElementTree as ET
import numpy as np
import time
import openravepy

from HomogeneousRobotTeam import *
from config import *

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
        lock_robot.SetActiveManipulator(0)

    # generate the ik solver
    iktype = IkParameterization.Type.Translation3D
    ikmodel = databases.inversekinematics.InverseKinematicsModel( lock_robot,
                                                                  iktype=iktype )
    if not ikmodel.load():
        ikmodel.autogenerate()
    
    # generate ik solution solution
    fix_target = np.array([2.2, 5.2, 20.3])
    with env:
        while True:
            target = fix_target + (np.random.rand(3)-2)*0.1
            ikparam = IkParameterization(target, iktype)
            solutions = ikmodel.manip.FindIKSolutions( ikparam, IkFilterOptions.CheckEnvCollisions )
            if solutions is not None and len(solutions)>0: break
            print "[sys] solver not found for: " + str(target.tolist)
                                    
    raw_input("Press enter to exit...")
