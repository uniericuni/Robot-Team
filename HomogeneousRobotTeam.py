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
        self.trajectory = {}
        self.active_dofs = [ active_joint_dofs,
                             active_affine_dofs ]
        self.ACC = 0

        # instantialize homogeneous robots
        self.robots = []
        for i in range(instance_number):
            self.trajectory[i] = []
            robot_instance = self.env.ReadRobotURI(instance_xml)
            robot_instance.SetName('robot%d'%(self.ACC) )
            self.ACC += 1
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

        # unlock the robot first if the robot is locked
        if self.status == LOCK0 or self.status == LOCKN:
            self.unlock()

        # enforce robots to certain team configuration
        if enforced:
            self.release()
            self.enforce()
        self.status = lock_mode

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
        base = ET.Element('base'); base.text = '0_Base'
        eff = ET.Element('effector'); eff.text = '%d_Base'%(self.instance_number-1)
        #joints = ET.Element('joints'); joints.text = '0-to-1 1-to-2';
        manipulator.append(base)
        manipulator.append(eff)
        #manipulator.append(joints)
        wrapper_joint.append(wrapper_kinbody)
        wrapper_joint.append(manipulator)
        root.append(wrapper_joint)
        
        # write the model to xml
        tree.write(self.lock_xml)
        if print_out:
            ET.dump(root)
     
        # return lockded system
        self.lock_robot = self.env.ReadRobotURI(self.lock_xml)
        self.lock_robot.SetName('robot%d'%(self.ACC) )
        self.ACC += 1
        self.env.AddRobot(self.lock_robot)
        for i,robot in enumerate(iterator):
            self.env.Remove(robot)
        self.robots = []

        # set manipulator
        self.lock_robot.SetActiveDOFs( [i for i in range(len(self.lock_robot.GetJoints()))] )
        self.lock_robot.SetActiveManipulator('chain')

        # system log
        self.log.msg('Sys', 'robot team modeled')
        return self.lock_robot
     
    # TODO: unlock robot team and destroy temporary .xml
    def unlock(self):

        if self.status == UNLOCK:
            return 

        self.release()
        self.robots = []
        self.status = UNLOCK
        links =  self.lock_robot.GetLinks()
        for i in range(self.instance_number):
            robot_instance = self.env.ReadRobotURI(self.instance_xml)
            robot_instance.SetName('robot%d'%(self.ACC) )
            self.ACC += 1
            robot_instance.SetTransform(links[i].GetTransform())
            robot_instance.SetActiveDOFs( self.active_dofs[0], self.active_dofs[1] )
            self.robots.append(robot_instance)
            self.env.AddRobot(robot_instance)
        self.env.Remove(self.lock_robot)

    # enforce team configurations
    def enforce(self):

        # get the robots' radius
        try:
            radius = self.robots[0].GeometryInfo.Get.SphereRadius()
        except:
            radius = RADIUS 

        prev_robot = self.robots[0]
        for i,robot in enumerate(self.robots[1:]):
            rtn = self.moveUntil(prev_robot, robot, radius)
            self.trajectory[i].extend(rtn)
            prev_robot = robot
      
    # enforce one kinbody move towards to another as near as possible
    def moveUntil(self, obj1,                   # target kinbody
                        obj2,                   # enforced kinbody
                        rad,                    # assumed kinbody volume sphere
                        away=False):             # move away

        # move close
        rtn = []
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

    # setup planner
    def setPlanner(self, planner, query, mode=0):
        self.query = query
        self.planner = planner
        self.log.msg('Sys', 'planner setup')

    # planning
    def planning(self, is_distributed = False):

        # raised error when planner hasn't bee setup
        if self.planner is None or self.query is None:
            self.log.msg('Error', 'please call setPlanner() first to setup planner')
            return 

        # plan for locked robot
        if self.status==LOCK0 or self.status==LOCKN:
            #try:
            rtn = self.planner(self.query, self.env, self.lock_robot, '%d_Base'%(self.instance_number-1))
            self.log.msg('Sys', 'path found for ' + ' '.join([str(s) for s in self.query]))
            self.trajectory[0] = rtn
            #except:
                #self.log.msg('Error', 'planner function and arguments not match')

        # plan for unlocked robot
        elif not is_distributed:
            pos = np.array(self.robots[0].GetActiveDOFValues())
            query = self.query
            with self.env:
                rtn = self.planner( [pos, query], self.env, self.robots[0] )
            self.log.msg('Sys', 'path found for robot %d'%0)
            self.trajectory[0].extend(rtn)

            for i in range(1,self.instance_number):
            #for i in range(1,14):

                # catch current config
                origin = self.robots[i].GetActiveDOFValues()
                pos = np.array(self.robots[i].GetActiveDOFValues())
                
                # check for target collision
                count = 0
                while True:

                    r = (pos-query) * (float(RADIUS)/100)*4 * np.random.random() / np.linalg.norm(pos-query)
                    pos_ = query + r
                    self.robots[i].SetActiveDOFValues(pos_)
                    self.log.msg('Sys', 'check target collision')
                    if (pos_[0]>X_MIN1 and pos_[0]<X_MAX1 and pos_[1]>Y_MIN and pos_[1]<Y_MAX) and ( not self.env.CheckCollision(self.robots[i]) ):
                        break

                # planning
                query = pos_
                rtn = self.planner( [pos, query], self.env, self.robots[i] )
                self.log.msg('Sys', 'path found for robot %d'%i)
                self.trajectory[i].extend(rtn)
                self.trajectory[i].append(list(pos_))
    
        # astar, distributedly
        else:
            self.trajectory = {}
            for i in range(0,self.instance_number)[::-1]:
                self.trajectory[i] = []
                if i-1>=0:
                    rtn = self.moveUntil( self.robots[i-1], self.robots[i], RADIUS*2, away=True)            
                    self.trajectory[i].extend(rtn)
                pos = np.array(self.robots[i].GetActiveDOFValues())
                query = self.query[i]
                rtn = self.planner( [pos, query], self.env, self.robots[i] )
                self.log.msg('Sys', 'path found for robot %d'%i)
                self.trajectory[i].extend(rtn)
                self.trajectory[i].append(list(query))
                

    # planning
    def release(self, inverse=False):
       
        if self.status==UNLOCK:
            if not inverse:
                for i in range(0,self.instance_number)[::-1]:
                    while len(self.trajectory[i])>0:
                        pos = self.trajectory[i].pop(0)
                        self.robots[i].SetActiveDOFValues(pos)
                        time.sleep(TIME_DELTA)
            else:
                for i in range(self.instance_number):
                    while len(self.trajectory[i])>0:
                        pos = self.trajectory[i].pop(0)
                        self.robots[i].SetActiveDOFValues(pos)
                        time.sleep(TIME_DELTA)

        else:
            while len(self.trajectory[0])>0:
                pos = self.trajectory[0].pop(0)
                self.lock_robot.SetActiveDOFValues(pos)
                time.sleep(TIME_DELTA)
