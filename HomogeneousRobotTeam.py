# Conceptual workflow: query -> planning -> release resource -> visualize
# planning for transition: planner -> release -> lock/unlock
# planning for section: planner -> release

import xml.etree.ElementTree as ET
import numpy as np
import openravepy
import os, time

from config import *
from Sampler import *
from Log import * 

class HomogeneousRobotTeam:

    # initialization
    def __init__(self, env,                     # openrave environment
                       instance_xml,            # file path to the model of homogeneous robot
                       instance_number,         # number of robot to initaite
                       lock_xml,                # file path to store temporary robot team unlocked model
                       lock_template_xml,       # file path to the template for locked robot
                       active_joint_dofs,       # set active dofs by joints
                       active_affine_dofs,      # set active dofs by affine
                       joint_type='chain',      # joint type: chain, central distributed, network <--- should change a name, lol
                       configs=None ):          # assigned initial configuration for each bot 

        # Define system parameters
        np.random.seed()                        # iniate random seed
        self.status = UNLOCK                    # default the robot team as UNLOCK
        self.env = env
        self.instance_xml = instance_xml
        self.instance_number = instance_number
        self.lock_template_xml = lock_template_xml
        self.lock_xml = lock_xml
        self.joint_type = joint_type
        self.log = Log(ISDISPLAY)
        self.active_dofs = [ active_joint_dofs,
                             active_affine_dofs ]

        # Initiate system parameters
        self.ACC = 0
        self.trajectory = {}
        self.robots = []
        self.planner = None
        self.query = None
        self.lock_robot = None
        
        # Instantiate the robot team 
        declareUnlockRobot(self, configs=configs)

    # Instantialize homogeneous robots
    def declareUnlockRobot(self, configs=None):

        # Instantialize single robot
        for i in range(self.instance_number):
            self.trajectory[i] = []
            robot_instance = self.env.ReadRobotURI(self.instance_xml)
            robot_instance.SetName('robot%d'%(self.ACC))
            self.ACC += 1
            if configs:
                tf = configs[i]
                robot_instance.SetTransform(tf)
            robot_instance.SetActiveDOFs( self.active_dofs[0], 
                                          self.active_dofs[1] )
            self.robots.append(robot_instance)
            self.env.AddRobot(robot_instance)

        # Remove locked robot instance
        if self.lock_robot!=None:
            self.env.Remove(self.lock_robot)

    # Declare lock robot model
    def declareLockRobot(self):

        # Xml element tree initialization
        tree = ET.parse(self.lock_template_xml)
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
            if i==0: 
                continue
            
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
            joint.append(body1)
            joint.append(body2)
            joint.append(offset)
            joint.append(axis)
            wrapper_kinbody.append(joint)

        # Define manipulator for chain
        manipulator = ET.Element('Manipulator', {'name': 'chain'})
        base = ET.Element('base'); base.text = '0_Base'
        eff = ET.Element('effector'); eff.text = '%d_Base'%(self.instance_number-1)
        manipulator.append(base)
        manipulator.append(eff)
        wrapper_joint.append(wrapper_kinbody)
        wrapper_joint.append(manipulator)
        root.append(wrapper_joint)
        
        # Write the model to xml
        tree.write(self.lock_xml)
        if print_out:
            ET.dump(root)
     
        # Return lockded system
        self.lock_robot = self.env.ReadRobotURI(self.lock_xml)
        self.lock_robot.SetName('robot%d'%(self.ACC))
        self.ACC += 1
        self.env.AddRobot(self.lock_robot)

        # Remove single robot
        for i,robot in enumerate(iterator):
            self.env.Remove(robot)
        self.robots = []

        # set manipulator
        self.lock_robot.SetActiveDOFs( [i for i in range(len(self.lock_robot.GetJoints()))] )
        self.lock_robot.SetActiveManipulator('chain')

    # Transition from lock/unlock to lock
    def lock(self, lock_mode,                   # LOCK BASE 0 or BASE N
                   enforced=True,               # enforce to assemble
                   print_out=False):            # print out generated xml tree

        # Unlock the robot first if the robot is locked
        if self.status == LOCK0 or self.status == LOCKN:
            self.unlock(enforced=False)

        # Enforce robots to assemble
        if enforced:
            self.enforce()
            self.release()

        # Declare lock robot model
        self.status = lock_mode
        declareLockRobot()

        # system log
        self.log.msg('Sys', 'robot team modeled')
        return self.lock_robot

    # Transition from lock to unlock
    def unlock(self, enforced=False,            # enforce to dessemble
                     print_out=False):          # print out generated xml tree

        # If already unlock, return
        if self.status == UNLOCK:
            return 

        # Enforce robots to dessemble
        if enforced:
            self.enforce(away=True)
            self.release()

        # If already unlock, return
        self.status = UNLOCK
        self.robots = []
        links =  self.lock_robot.GetLinks()
        configs = [link.GetTransform() for link in links]
        declareUnlockRobot(self, configs=configs)

        # system log
        self.log.msg('Sys', 'robot instance modeled')
        return self.robots

    # enforce team configurations
    def enforce(self, away=False):

        # Get the robots' radius
        try:
            radius = self.robots[0].GeometryInfo.Get.SphereRadius()
        except:
            radius = RADIUS 

        # Enforce robot to assembly/dessembly
        prev_robot = self.robots[0]
        for i,robot in enumerate(self.robots[1:]):
            rtn = self.moveUntil(prev_robot, robot, radius, away=away)
            self.trajectory[i].extend(rtn)
            prev_robot = robot

        # Recover the configuration before planning
        for i,robot in enumerate(self.robots[1:]):
            robot.SetActiveDOFValues(self.trajectory[i][0])
      
    # enforce one kinbody move towards to another as near as possible
    def moveUntil(self, obj1,                   # target kinbody
                        obj2,                   # enforced kinbody
                        rad,                    # assumed kinbody volume sphere
                        away=False):             # move away

        rtn = [obj2.GetActiveDOFValues()]

        # Move close
        if not away:
            diff = np.array(obj1.GetTransform()) - np.array(obj2.GetTransform())
            while np.linalg.norm(diff) > rad*2/100+TRANS_ERR:
                obj2.SetTransform( np.array(obj2.GetTransform()) + (diff*TRANS_ERR/2)/np.linalg.norm(diff) )
                diff = np.array(obj1.GetTransform()) - np.array(obj2.GetTransform())
                rtn.append(obj2.GetActiveDOFValues())

            # system log
            self.log.msg('Sys', 'robot assemblied')

        # move away
        else:
            diff = np.array(obj2.GetTransform()) - np.array(obj1.GetTransform())
            while np.linalg.norm(diff) < rad*2/100+TRANS_ERR:
                obj2.SetTransform( np.array(obj2.GetTransform()) + (diff*TRANS_ERR/2)/np.linalg.norm(diff) )
                diff = np.array(obj2.GetTransform()) - np.array(obj1.GetTransform())
                rtn.append(obj2.GetActiveDOFValues())

            # system log
            self.log.msg('Sys', 'robot departed')
        
        return rtn

    # Setup planner
    def setPlanner(self, planner, query, mode=0):
        self.query = query
        self.planner = planner
        self.log.msg('Sys', 'planner setup')

    # planning
    def planning(self, is_distributed = False):

        # Raised error when planner hasn't bee setup
        if self.planner is None or self.query is None:
            self.log.msg('Error', 'please call setPlanner() first to setup planner')
            exit()

        self.trajectory = {}

        # Plan for locked robot
        if self.status==LOCK0 or self.status==LOCKN:

            # Call the planner
            rtn = self.planner( self.query,
                                self.env,
                                self.lock_robot,
                                '%d_Base'%(self.instance_number-1) )
            self.log.msg('Sys', 'path found for ' + ' '.join([str(s) for s in self.query]))
            self.trajectory[0] = rtn
            self.log.msg('Error', 'planner function and arguments not match')
            exit()

            # Recover the configuration before planning
            self.lock_robot.setActiveDOFValues(self.trajectory[0][0])

        # Plan for unlocked robot
        else:

            # Restore the configuration before planning
            origin = []
            for i,robot in enumerate(self.robots):
                origin.append(robot.GetActiveDOFValues())

            # Extend query
            if len(query)==1:
                prev_pos = query
                self.robots[0].SetActiveDOFValues(prev_pos)
                query = []
                for i in range(1,self.instance_number):
                    pos = np.array(self.robots[i].GetActiveDOFValues())
                    while True:
                        r = (pos-prev_pos) * (float(RADIUS)/100)*4 * np.random.random() / np.linalg.norm(pos-query)
                        pos_ = prev_pos + r
                        self.robots[i].SetActiveDOFValues(pos_)
                        if not isRejected(pos_) and not self.env.CheckCollision(self.robots[i]):
                            break
                    query.append(pos_)
                    robot[i].SetActiveDOFValues(pos_)

            # Recover the configuration
            for i,robot in enumerate(self.robots):
                robot.SetActiveDOFValues(self.origin[i])
                
            # TODO: decide leading head
            #for i in range(0,self.instance_number)[::-1]:
            for i in range(0,self.instance_number):
                self.trajectory[i] = []
                pos = np.array(self.robots[i].GetActiveDOFValues())
                query = self.query[i]
                rtn = self.planner( [pos, query], self.env, self.robots[i] )
                self.log.msg('Sys', 'path found for robot %d'%i)
                self.trajectory[i].extend(rtn)
                self.trajectory[i].append(list(query))

            # Recover the configuration
            for i,robot in enumerate(self.robots):
                robot.SetActiveDOFValues(self.trajectory[i][0])
                

    # ====================================================================================
    # release(self, inverse=False):
    #   Release trajectory buffer to visualize on GUI.
    # Inputs:
    #   inverse: Bool, if the display should start from tail.
    # ====================================================================================
    def release(self, inverse=False):
       
        if self.status==UNLOCK:

            # Starting from head
            if not inverse:
                for i in range(0,self.instance_number)[::-1]:
                    while len(self.trajectory[i])>0:
                        pos = self.trajectory[i].pop(0)
                        self.robots[i].SetActiveDOFValues(pos)
                        time.sleep(TIME_DELTA)

            # Starting from tail
            else:
                for i in range(self.instance_number):
                    while len(self.trajectory[i])>0:
                        pos = self.trajectory[i].pop(0)
                        self.robots[i].SetActiveDOFValues(pos)
                        time.sleep(TIME_DELTA)

        else:

            # Move simultaneously
            while len(self.trajectory[0])>0:
                pos = self.trajectory[0].pop(0)
                self.lock_robot.SetActiveDOFValues(pos)
                time.sleep(TIME_DELTA)

        # Ensure trajectory buffer is clear
        self.trajectory = {}
