import openravepy
import xml.etree.ElementTree as ET
import os

from config import *

class HomogeneousRobotTeam:

    # initialization
    def __init__(self, env,                     # openrave environment
                       instance_xml,            # file path to the model of homogeneous robot
                       instance_number,         # number of robot to initaite
                       lock_xml,                # file path to store temporary robot team model
                       joint_type ):            # joint type: chain, central distributed, network 

        # system parameters
        self.env = env
        self.instance_xml = instance_xml
        self.instance_number = instance_number
        self.lock_xml = lock_xml
        self.joint_type = joint_type
        self.status = 'unlock'

        # instantiation
        for i in range(instance_number):
            robot_instance = self.env.ReadRobotURI(instance_xml)
            robot_instance.SetName('robot%d'%i)
            self.env.AddRobot(robot_instance)
        self.robots = self.env.GetRobots()

    # TODO: destructor calling unlock()

    # lock robot team into a single .xml and bind by joints
    def lock(self, xml_template, base_config, configs=None, enforced=True, print_out=False):

        # enforce robots to certain configurations
        if enforced:
            self.enforce()

        # set the configuration of each robot
        if configs:
            pass

        # xml element tree initialization
        # tree = ET.parse(LOCK_ROBOT_TEMPLATE)
        tree = ET.parse(self.instance_xml)
        root = tree.getroot()
        base = ET.Element('Kinbody', {'name': 'robot0'})
        prev_node = base

        # lock robots
        # TODO: instantiate virtual robot0
        # TODO: joint type
        for i,robot in enumerate(self.robots):
     
            # create elements
            print robot.GetTransform()
            node = ET.Element('Kinbody', {'name': 'robot%d'%(i+1),
                                          'file': self.instance_xml})
            joint = ET.Element('Joint',  {'enable': 'false',
                                          'type': 'hinge',
                                          'name': '%d-to-%d'%(i,i+1)} )
            body1 = ET.Element('Body')
            body1.text = 'robot%d'%(i)
            body2 = ET.Element('Body')
            body2.text = 'robot%d'%(i+1)
            limit = ET.Element('limiits')
            limit.text = '%f %f'%(PI,PI)

            # TODO: set initial configuration
           
            # assemble elements 
            joint.append(body1)
            joint.append(body2)
            joint.append(limit)
            root.append(node)
            root.append(joint)
      
            # robot iter
            prev_node = node

        tree.write(self.lock_xml)
        if print_out:
            ET.dump(root)
     
        # TODO: deinfe manipulator
     
        # return lockded system
        print self.lock_xml
        lock_robot = self.env.ReadRobotURI(TEMPORARY_LOCK_ROBOT)
        
        self.env.AddRobot(lock_robot)
        self.lock_robot = self.env.GetRobots()[0]
     
    # unlock robot team and destroy temporary .xml
    def unLockRobots(self):
        
        # delete the temporary robot group model
        os.remove(self.lock_xml)

        # restore robot configuration

    # TODO: enforce configurations
    def enforce(self):
        pass
    
