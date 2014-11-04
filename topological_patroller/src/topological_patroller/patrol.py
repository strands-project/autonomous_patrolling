#!/usr/bin/env python

from time import sleep
import os
import rospy
import smach
import smach_ros
import datetime

from random import shuffle
import scitos_docking.msg
from actionlib import *
from actionlib.msg import *
import topological_navigation.msg
import scitos_ptu.msg


class CheckPointAction(object):
    def __init__(self, name, args):
        self.name = name
        self.args = args.strip(' ')

class CheckPoint(object):
    def __init__(self, name):
        self.name = name
        self.actions=[]
        
    def _insert_actions(self, action_name):
        print "Inserting:"
        print action_name
        act=CheckPointAction(action_name[0], action_name[1])
        self.actions.append(act)
        
        
        
class PatrolCheckpoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'], input_keys=['task_name','pch_patrol_points','pch_next_node'])


    def execute(self, userdata):

        targ  = userdata.pch_next_node
        rospy.loginfo('Executing state PATROL_CHECKPOINT')

        nav_client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        nav_client.wait_for_server()
        navgoal = topological_navigation.msg.GotoNodeGoal()
        print "Requesting Navigation to %s" %targ
        navgoal.target = targ
    
        # Sends the goal to the action server.
        nav_client.send_goal(navgoal)
    
        # Waits for the server to finish performing the action.
        nav_client.wait_for_result()
        # Prints out the result of executing the action
        result = nav_client.get_result()  # A FibonacciResult
        print "result"
        print result
        print result.success        
        if result.success == False :
            #userdata.pch_current_node="Unknown"
            return 'aborted'
        else :
            #userdata.pch_current_node=userdata.pch_next_node
            print "navigation Succeeded"
        for i in userdata.pch_patrol_points :
            if i.name == targ:
                for j in i.actions:
                    print "executing!!!!!"
                    print j.name
                    if j.name == 'sleep' :
                        print j.args
                        c = int(j.args)
                        sleep(c)
                    if j.name == '3Dsnapshot' :
                        filename = "3D_%s_%s.bag" %(userdata.task_name,targ)
                        bashCommand = "timeout 10 rosbag record %s -l 1 -O ~/storage/%s" %(j.args,filename)
                        print bashCommand
                        os.system(bashCommand)
                    if j.name == 'scitos_ptu' :
                        print "Scitos PTU:"
                        print "Creating Action Server"
                        ptu_client = actionlib.SimpleActionClient('ptu_pan_tilt', scitos_ptu.msg.PanTiltAction)
                        print "Done"
                        ptu_client.wait_for_server()
                        ptugoal = scitos_ptu.msg.PanTiltGoal()
                        argums = j.args.split(',') 
                        ptugoal.pan_start=int(argums[0])
                        ptugoal.pan_step=int(argums[1])
                        ptugoal.pan_end=int(argums[2])
                        ptugoal.tilt_start=int(argums[3])
                        ptugoal.tilt_step=int(argums[4])
                        ptugoal.tilt_end=int(argums[5])
                        ptu_client.send_goal(ptugoal)
                    
                        # Waits for the server to finish performing the action.
                        ptu_client.wait_for_result()
                        # Prints out the result of executing the action
                        result_ptu = nav_client.get_result()  # A FibonacciResult
                        print "result"
                        print result_ptu
        return 'succeeded'


class PointChoose(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_waypoint','back_to_home'], input_keys=['pc_patrol_points','task_name'], output_keys=['pc_next_node'] )
        self.counter = 0
        self.nodes = []

    def execute(self, userdata):
        print "Executing patrol %s" %userdata.task_name
        rospy.loginfo('Executing state POINT_CHOOSE')
        if self.counter == 0 :
            for i in userdata.pc_patrol_points :
                self.nodes.append(i.name)
            #shuffle(self.nodes)

        if len(self.nodes) > self.counter :
            userdata.pc_next_node = self.nodes[self.counter]
            print "Going to:"
            print self.nodes[self.counter]
            self.counter=self.counter+1
            return 'next_waypoint'
        else:
            userdata.pc_next_node = "ChargingPoint"
            print "Going to:"
            print "ChargingPoint"
            self.counter=0
            self.nodes=[]
            return 'back_to_home'


class RetryPoint(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['retry','next_waypoint'], input_keys=['pc_next_node'])
        self.counter = 0
        self.lastnode = 'none'

    def execute(self, userdata):
        rospy.loginfo('Executing state Retry Nav')
        if self.lastnode != userdata.pc_next_node :
            self.lastnode = userdata.pc_next_node
            self.counter = 0

        if self.counter < 5 :
            self.counter = self.counter+1
            return 'retry'
        else:
            self.counter=0
            return 'next_waypoint'
