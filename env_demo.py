import time
import openravepy
import xml.etree.ElementTree as ET

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

    robots = HomogeneousRobotTeam(  env,
                                    INSTANCE_ROBOT, 4,
                                    TEMPORARY_LOCK_ROBOT, 
                                    'chain',
                                    [ [[ 1,0,0,-9.0],[0,1,0,2*i],[0,0,1,20.3]] for i in range(4) ])
    robots.lock( LOCK_ROBOT_TEMPLATE, [])

    raw_input("Press enter to exit...")

