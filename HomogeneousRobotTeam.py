import xml.etree.ElementTree as ET
import numpy as np
import openravepy
import os
import Log

from config import *

class HomogeneousRobotTeam:

    # initialization
    def __init__(self, env,                     # openrave environment
                       instance_xml,            # file path to the model of homogeneous robot
                       instance_number,         # number of robot to initaite
                       lock_xml,                # file path to store temporary robot team model
                       active_joint_dofs,       # set active dofs by joints
                       active_affine_dofs,      # set active dofs by affine
                       joint_type='chain',      # joint type: chain, central distributed, network <--- should change a name, lol
                       configs=None):           # assigned initial configuration for each bot 

        # define system parameters
        self.env = env
        self.instance_xml = instance_xml
        self.instance_number = instance_number
        self.lock_xml = lock_xml
        self.joint_type = joint_type
        self.status = UNLOCK
        self.log = Log.Log(ISDISPLAY)
        self.planner = None
        self.query = None

        # instantialize homogeneous robots
        self.robots = []
        for i in range(instance_number):
            robot_instance = self.env.ReadRobotURI(instance_xml)
            robot_instance.SetName('robot%d'%i)
            if configs:
                tf = configs[i]
                robot_instance.SetTransform(tf)
            robot_instance.SetActiveDOFs(active_joint_dofs, active_affine_dofs)
            self.env.AddRobot(robot_instance)
        self.robots = self.env.GetRobots()

    # lock robot team into a single .xml and bind by joints
    def lock(self, xml_template,                # file path to robot team model template 
                   enforced=True,               # enforce to align according to joint_type
                   print_out=False):            # print out generated xml tree

        # enforce robots to certain team configuration
        if enforced:
            self.enforce()

        # xml element tree initialization
        tree = ET.parse(xml_template)
        root = tree.getroot()

        # lock robots
        # TODO: joint type
        wrapper_joint = ET.Element('Robot')
        wrapper_kinbody = ET.Element('Kinbody')
        joint_list = []
        for i,robot in enumerate(self.robots):
     
            # create robot node
            wrapper = ET.Element('Robot')
            kinbody = ET.Element('Robot', { 'prefix': '%d_'%(i),
                                            'file': self.instance_xml})
            transform = robot.GetTransform()
            translation = ET.Element('translation'); translation.text = ' '.join( str(x[3]) for x in transform[0:3] );
            rotationmat = ET.Element('rotationmat'); rotationmat.text = ' '.join( str(x) for x in np.ravel(transform[0:3,0:3]) );
            wrapper.append(kinbody) 
            wrapper.append(translation)
            wrapper.append(rotationmat)
            root.append(wrapper)

            # avoid circular link
            if i==0: continue
            
            # create joints
            joint = ET.Element('Joint',  {'enable': 'true',
                                          'type': 'hinge',
                                          'name': '%d-to-%d'%(i-1,i)} )
            joint_list.append('%d-to-%d'%(i-1,i))
            body1 = ET.Element('Body'); body1.text = '%d_Base'%(i-1);
            body2 = ET.Element('Body'); body2.text = '%d_Base'%(i);
            limit = ET.Element('limitdegs'); limit.text = '%f %f'%(DEG_LIMIT1, DEG_LIMIT2);
            joint.append(body1)
            joint.append(body2)
            joint.append(limit)
            wrapper_kinbody.append(joint)

        # define manipulator for chain
        manipulator = ET.Element('Manipulator', {'name': 'chain'})
        base = ET.Element('base'); base.text = '0_Base';
        eff = ET.Element('effector'); eff.text = '%d_Base'%(self.instance_number-1)
        manipulator.append(eff)
        manipulator.append(base)
        wrapper_joint.append(wrapper_kinbody)
        wrapper_joint.append(manipulator)
        root.append(wrapper_joint)
        
        # write the model to xml
        tree.write(self.lock_xml)
        if print_out:
            ET.dump(root)
     
        # return lockded system
        lock_robot = self.env.ReadRobotURI(self.lock_xml)
        self.env.AddRobot(lock_robot)
        self.lock_robot = self.env.GetRobots()[self.instance_number]

        # set all joint dofs active
        self.lock_robot.SetActiveDOFs( [i for i in range(len(self.lock_robot.GetJoints()))] )

        # system log
        self.log.msg('Sys', 'robot team modeled')
        self.status = LOCK
        return self.lock_robot
     
    # TODO: unlock robot team and destroy temporary .xml
    def unLockRobots(self):
        
        # delete the temporary robot group model
        os.remove(self.lock_xml)

        # restore robot configuration

    # enforce team configurations
    def enforce(self):

        # get the robots' radius
        try:
            radius = self.robots[0].GeometryInfo.Get.SphereRadius()
        except:
            radius = RADIUS 

        # bound robots in radius
        prev_robot = self.robots[0]
        for robot in self.robots[1:]:
            self.moveUntil(prev_robot, robot, radius)
            prev_robot = robot
      
    # enforce one kinbody move towards to another as near as possible
    def moveUntil(self, obj1,                   # target kinbody
                        obj2,                   # enforced kinbody
                        rad):                   # assumed kinbody volume sphere

        diff = np.array(obj1.GetTransform()) - np.array(obj2.GetTransform())
        while np.linalg.norm(diff) > rad*2/100+TRANS_ERR:
            obj2.SetTransform( np.array(obj2.GetTransform()) + (diff*TRANS_ERR/2)/np.linalg.norm(diff) )
            diff = np.array(obj1.GetTransform()) - np.array(obj2.GetTransform())

        # system log
        self.log.msg('Sys', 'robot assemblied')

    # setup planner
    # TODO: implement hiearchical planning + series of query binding with lock/unlock status
    def setPlanner(self, planner, query):
        self.query = query
        self.planner = planner
        self.log.msg('Sys', 'planner setup')

    # planning
    def planning(self):

        # raised error when planner hasn't bee setup
        if not self.planner or not self.query:
            self.log.msg('Error', 'please call setPlanner() first to setup planner')
            return 

        # plan for locked robot
        if self.status==LOCK:
            try:
                self.planner(query, env, self.lock_robot)
            except:
                self.log.msg('Error', 'planner function and arguments not match')

        # plan for unlocked robot
        else:
            try:
                self.planner(query, env, self.robots)
            except:
                self.log.msg('Error', 'planner function and arguments not match')
