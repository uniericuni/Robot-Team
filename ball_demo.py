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
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    robot = env.GetRobots()[0]

    Tz = matrixFromAxisAngle([0,0,numpy.pi/6])

    robots = HomogeneousRobotTeam(  env,
                                    INSTANCE_ROBOT, 4,
                                    TEMPORARY_LOCK_ROBOT, 
                                    'chain')
    robots.lock( LOCK_ROBOT_TEMPLATE, [])

    raw_input("Press enter to exit...")

