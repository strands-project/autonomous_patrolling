#!/usr/bin/env python

#import roslib; roslib.load_manifest('gather_smach')
from time import sleep
import os
import rospy
import smach
import smach_ros
import datetime

import scitos_apps_msgs.msg
from actionlib import *
from actionlib.msg import *
import topological_navigation.msg


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
        act=CheckPointAction(action_name[0],action_name[1])
        self.actions.append(act)
        
        
        
class PatrolCheckpoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded','aborted'], 
                             input_keys=['task_name','pch_patrol_points','pch_next_node'])


    def execute(self, userdata):
        #print "point set:"
        #print userdata.pch_patrol_points
        #print "current node:"
        #print userdata.pch_current_node
        targ  = userdata.pch_next_node
        print "Going to:"
        print userdata.pch_next_node
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
        return 'succeeded'