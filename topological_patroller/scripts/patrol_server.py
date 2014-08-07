#!/usr/bin/env python


import rospy
import smach
import smach_ros
import datetime
from time import sleep
from datetime import datetime
import time
import json
import sys


from actionlib import *
from actionlib.msg import *



#from topological_patroller.patroller import *
from topological_patroller.msg import *
from ros_datacentre.message_store import MessageStoreProxy

import patrol_snapshot.msg
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf.msg import *
import topological_navigation.msg
import scitos_ptu.msg
import scitos_ptu_sweep.msg
import tf
import rosbag



class PointChoose(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_waypoint','back_to_home'], input_keys=['patrol_points','task_name'], output_keys=['next_node'] )
        self.counter = 0
        self.nodes = []

    def execute(self, userdata):
        print "Executing patrol %s" %userdata.task_name
        rospy.loginfo('Executing state POINT_CHOOSE')
        if self.counter == 0 :
            for i in userdata.patrol_points :
                self.nodes.append(i.waypoint)
            #shuffle(self.nodes)

        if len(self.nodes) > self.counter :
            for i in userdata.patrol_points :
                if self.nodes[self.counter] == i.waypoint :
                    userdata.next_node = i
            print "Going to:"
            print self.nodes[self.counter]
            self.counter=self.counter+1
            return 'next_waypoint'
        else:
            userdata.next_node = "none"
            print "all Done"
            self.counter=0
            self.nodes=[]
            return 'back_to_home'


class PatrolCheckpoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'], input_keys=['task_name','patrol_points','next_node'])


    def execute(self, userdata):

        targ  = userdata.next_node.waypoint
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
        print "navigation result"
        print result
        print result.success        
        if result.success == False :
            return 'aborted'
        else :          
            print "navigation Succeeded"
        
        for j in userdata.next_node.action :
            print "executing!!!!!"
            print j.name
            if j.name == 'sleep' :
                
                print 'sleep '+j.args[0]
                c = int(j.args[0])
                sleep(c)
            if j.name == '3Dsnapshot' or j.name == 'snapshot' :
                print "snapshot"
                snap_client = actionlib.SimpleActionClient('patrol_snapshot', patrol_snapshot.msg.PatrolSnapshotAction)
    
                snap_client.wait_for_server()
                rospy.loginfo(" ... Init done")
            
                goal = patrol_snapshot.msg.PatrolSnapshotGoal()
            
                # Sends the goal to the action server.
                snap_client.send_goal(goal)
            
                # Waits for the server to finish performing the action.
                snap_client.wait_for_result()
            
                # Prints out the result of executing the action
                print client.get_result()
                #filename = "3D_%s_%s.bag" %(userdata.task_name,targ)
                #bashCommand = "timeout 10 rosbag record %s -l 1 -O ~/storage/%s" %(j.args[0],filename)
                #print bashCommand
                #os.system(bashCommand)
            if j.name == 'scitos_ptu' or j.name == 'ptu_sweep' :
                print "Scitos PTU:"
                print "Creating Action Server"
                ptu_client = actionlib.SimpleActionClient('PTUSweep', scitos_ptu_sweep.msg.PTUSweepAction)
                print "Done"
                ptu_client.wait_for_server()
                print "Waiting for client done"
                
                ptugoal = scitos_ptu_sweep.msg.PTUSweepGoal()
            
                ptugoal.max_pan = float(sys.argv[1])
                ptugoal.max_tilt = float(sys.argv[2])
                ptugoal.min_pan = float(sys.argv[3])
                ptugoal.min_tilt = float(sys.argv[4])
                ptugoal.pan_step = float(sys.argv[5])
                ptugoal.tilt_step = float(sys.argv[6])
            
                # Sends the goal to the action server.
                ptu_client.send_goal(ptugoal)
            
                # Waits for the server to finish performing the action.
                ptu_client.wait_for_result()
                
                # Prints out the result of executing the action
                result_ptu = ptu_client.get_result()  # A FibonacciResult
                #print "result"
                print result_ptu
        return 'succeeded'



def loadTask(task_name):
    print "openning %s" %task_name

    msg_store = MessageStoreProxy(collection='patrol_tasks')
    
    query_meta = {}
    query_meta["type"] = "task_definition"
    query_meta["task"] = task_name
    #query_meta["stored_type"] = "strands_navigation_msgs/TopologicalNode"

    available = len(msg_store.query(topological_patroller.msg.PatrolTask._type, {}, query_meta)) > 0
    if available <= 0 :
        rospy.logerr("Desired task '"+task_name+"' not in datacentre")
        rospy.logerr("Available pointsets: "+str(available))
        raise Exception("Can't find waypoints.")
    else :
        query_meta = {}
        query_meta["type"] = "task_definition"
        query_meta["task"] = task_name
        message_list = msg_store.query(topological_patroller.msg.PatrolTask._type, {}, query_meta)

        print message_list[0][0]
        #   print i
    print "DONE"
    return  message_list[0][0]



class PatrolServer(object):

    _feedback = topological_patroller.msg.DoPatrolFeedback()
    _result   = topological_patroller.msg.DoPatrolResult()

    def __init__(self, name):

        self.cancelled = False
        self._action_name = name
        
        rospy.loginfo("Creating action servers.")
        print name        
        
        self._as = actionlib.SimpleActionServer('patrol_server', topological_patroller.msg.DoPatrolAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)

        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Ready ...")
        rospy.spin()
        
        
    def executeCallback(self, goal):
        print 'executing '+goal.task_name
        result=self.execute_patrol(goal.task_name)

        self._result.result = result
        self._as.publish_feedback(self._feedback)
        self._as.set_succeeded(self._result)


    def execute_patrol(self, task_name) :
        patrol_points_list = loadTask(task_name)
        patrol_points = patrol_points_list.patrol
    
        print "SMACH"
    
        sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        sm0.userdata.patrol_points=patrol_points
        sm0.userdata.task_name=task_name
        # Open the container
    
        with sm0:
            # POINT CHOOSE
            smach.StateMachine.add('POINT_CHOOSE', PointChoose(), transitions={'next_waypoint':'PATROL_CHECKPOINT', 'back_to_home':'succeeded'})
                    
            #GO TO CHECKPOINT
            smach.StateMachine.add('PATROL_CHECKPOINT', PatrolCheckpoint(), transitions={'succeeded':'POINT_CHOOSE', 'aborted':'POINT_CHOOSE'})
    
        # Execute SMACH plan
        outcome = sm0.execute()
        return outcome

    
    def preemptCallback(self):
        self.cancelled = True
        self._result.result = False
        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node('patrol_server')
    server = PatrolServer('patrol_server')