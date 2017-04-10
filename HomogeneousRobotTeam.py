import xml.etree.ElementTree as ET
import numpy as np
import openravepy
import os

from config import *
from Sampler import *
from Log import * 

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
        np.random.seed()
        self.env = env
        self.instance_xml = instance_xml
        self.instance_number = instance_number
        self.lock_xml = lock_xml
        self.joint_type = joint_type
        self.status = UNLOCK
        self.log = Log(ISDISPLAY)
        self.planner = None
        self.query = None
        self.trajectories = {}

        # instantialize homogeneous robots
        self.robots = []
        for i in range(instance_number):
            self.trajectories[i] = []
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
                   lock_mode,                   # LOCK BASE 0 or BASE N
                   enforced=True,               # enforce to align according to joint_type
                   print_out=False):            # print out generated xml tree

        # enforce robots to certain team configuration
        if enforced:
            self.enforce()

        # xml element tree initialization
        tree = ET.parse(xml_template)
        root = tree.getroot()

        # decide lock base
        if lock_mode == LOCK0:
            iterator = self.robots[:]
        else:
            iterator = self.robots[::-1]
        
        # lock robots
        # TODO: joint type
        wrapper_joint = ET.Element('Robot')
        wrapper_kinbody = ET.Element('Kinbody')
        joint_list = []
        for i,robot in enumerate(iterator):
     
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
            # TODO: redefine the joint DOF so to achiev 3DOF in whole
            diff = np.array(self.robots[i-1].GetTransform()) - np.array(self.robots[i].GetTransform())
            anchor_unit_vec = (diff/np.linalg.norm(diff)).tolist()
            joint = ET.Element('Joint',  {'enable': 'true',
                                          'circular': 'true',
                                          'type': 'hinge',
                                          'name': '%d-to-%d'%(i-1,i)} )
            joint_list.append('%d-to-%d'%(i-1,i))
            body1 = ET.Element('Body'); body1.text = '%d_Base'%(i-1);
            body2 = ET.Element('Body'); body2.text = '%d_Base'%(i);
            offset = ET.Element('offsetfrom'); offset.text = '%d_Base'%(i-1);
            axis = ET.Element('axis'); axis.text = '0 0 1'
            #anchor = ET.Element('Anchor'); anchor.text = ' '.join([str(v) for v in anchor_unit_vec])
            #anchor = ET.Element('Anchor'); anchor.text = ' '.join([str(v) for v in anchor_unit_vec])
            #joint.append(anchor)
            joint.append(body1)
            joint.append(body2)
            joint.append(offset)
            joint.append(axis)
            wrapper_kinbody.append(joint)

        # define manipulator for chain
        manipulator = ET.Element('Manipulator', {'name': 'chain'})
        if lock_mode == LOCK0:
            base = ET.Element('base'); base.text = '%d_Base'%(self.instance_number-1)
            eff = ET.Element('effector'); eff.text = '0_Base'
        else:
            base = ET.Element('base'); base.text = '0_Base'
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

        # set manipulator
        self.lock_robot.SetActiveDOFs( [i for i in range(len(self.lock_robot.GetJoints()))] )
        self.lock_robot.SetActiveManipulator('chain')

        # system log
        self.log.msg('Sys', 'robot team modeled')
        self.status = lock_mode
        return self.lock_robot
     
    # TODO: unlock robot team and destroy temporary .xml
    def unlock(self):
        pass
        
        # delete the temporary robot group model
        # os.remove(self.lock_xml)
        # restore robot configuration

    # enforce team configurations
    def enforce(self):

        # get the robots' radius
        try:
            radius = self.robots[0].GeometryInfo.Get.SphereRadius()
        except:
            radius = RADIUS 

        # bound robots in radius
        if self.status == LOCK0:
            prev_robot = self.robots[0]
            for robot in self.robots[1:]:
                self.moveUntil(prev_robot, robot, radius)
                prev_robot = robot

        else:
            prev_robot = self.robots[-1]
            for robot in self.robots[-2::-1]:
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
    def setPlanner(self, planner, query, mode=0):
        self.query = query
        self.planner = planner
        self.log.msg('Sys', 'planner setup')

    # planning
    def planning(self):

        # raised error when planner hasn't bee setup
        if self.planner is None or self.query is None:
            self.log.msg('Error', 'please call setPlanner() first to setup planner')
            return 

        # plan for locked robot
        if self.status==LOCK0:
            try:
                solutions = self.planner(self.query, self.env, self.lock_robot)
                self.log.msg('Sys', 'path found for ' + ' '.join([str(s) for s in self.query]))
            except:
                self.log.msg('Error', 'planner function and arguments not match')

        # plan for unlocked robot
        else:
            pos = np.array(self.robots[0].GetActiveDOFValues())
            query = self.query
            self.planner( [pos, query], self.env, self.robots[0] )
            self.log.msg('Sys', 'path found for robot %d'%0)

            for i in range(1,self.instance_number):
            #for i in range(1,14):

                # catch current config
                pos = np.array(self.robots[i].GetActiveDOFValues())
                
                # check for target collision
                count = 0
                while True:
                    if count%3==0:
                        r = np.array([0, RADIUS*3/100, 0])
                    elif count%3==1:
                        r = np.array([0, -RADIUS*3/100, 0])
                    else:
                        r = np.array([-RADIUS*3/100, 0, 0])
                    count += 1
                    #r = (pos-query) * (float(RADIUS)/100)*6 * np.random.random() / np.linalg.norm(pos-query)
                    #r = np.random.random(3)*RADIUS*float(8)/100+RADIUS*float(4)/100; r[2] = 0;
                    #if count%100==0: query = query - [0.5,0,0]
                    pos_ = query + r
                    self.robots[i].SetActiveDOFValues(pos_)
                    self.log.msg('Sys', 'check target collision')
                    print pos_
                    if (pos_[0]>X_MIN1 and pos_[0]<X_MAX1 and pos_[1]>Y_MIN and pos_[1]<Y_MAX) and ( not self.env.CheckCollision(self.robots[i]) ):
                        break

                # planning
                query = pos_
                self.planner( [pos, query], self.env, self.robots[i] )
                self.log.msg('Sys', 'path found for robot %d'%i)

            try:
                self.planner(self.query, self.env, self.robots[i])
                self.log.msg('Sys', 'path found for query')
            except:
                self.log.msg('Error', 'planner function and arguments not match')
