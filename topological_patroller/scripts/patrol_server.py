#!/usr/bin/env python

from time import sleep
import rospy
import smach
import smach_ros
import datetime
import json
import sys


from actionlib import *
from actionlib.msg import *

#from topological_patroller.patroller import *
from topological_patroller.msg import *
from ros_datacentre.message_store import MessageStoreProxy

import sensor_msgs
import topological_navigation.msg
import scitos_ptu.msg
import scitos_ptu_sweep.msg

class PTUSweepClient(object):

    def __init__(self, task, waypoint):
        print "PTU Sweep:"
        self.save_next=False
        current_time = datetime.now()
        self.dt_text= current_time.strftime('%A, %B %d, at %H:%M hours')
        self.task=task
        self.waypoint=waypoint
        
        print "Creating Action Server"
        self.ptus_client = actionlib.SimpleActionClient('PTUSweep', scitos_ptu_sweep.msg.PTUSweepAction)
        print "Done"
      
        
        self.ptus_client.wait_for_server()
        
        self.ptu_subs1 = rospy.Subscriber('/ptu_sweep/depth/points', sensor_msgs.PointCloud2, self.dpth_callback,  queue_size=1)
        self.ptu_subs2 = rospy.Subscriber('/transform_pc2/depth/points', sensor_msgs.PointCloud2, self.tpc_callback,  queue_size=1)
        self.ptu_subs3 = rospy.Subscriber('/head_xtion/depth_registered/points', sensor_msgs.PointCloud2, self.rgpc_callback,  queue_size=1)

        
    def execute_action(self, args) :

        ptusgoal = scitos_ptu_sweep.msg.PTUSweepGoal()
        #argums = j.args.split(',') 
        
        ptusgoal.max_pan = float(args[0])
        ptusgoal.max_tilt = float(args[1])
        ptusgoal.min_pan = float(args[2])
        ptusgoal.min_tilt = float(args[3])
        ptusgoal.pan_step = float(args[4])
        ptusgoal.tilt_step = float(args[5])

        ptus_client.send_goal(ptusgoal)
    
        # Waits for the server to finish performing the action.
        ptus_client.wait_for_result()
        # Prints out the result of executing the action
        result_ptus = ptus_client.get_result()  # A FibonacciResult
        #print "result"
        sleep(1)

        self.ptu_subs1.unregister()
        self.ptu_subs2.unregister()
        self.ptu_subs3.unregister()

        return result_ptus        


    def dpth_callback(self, msg):
   
        meta = {}
        meta["task"] = self.task
        meta["waypoint"] = self.waypoint
        meta["time"] = self.dt_text
        meta["topic"] = '/ptu_sweep/depth/points'
               
        msg_store = MessageStoreProxy(collection='patrol_data')
        msg_store.insert(msg,meta)
        self.save_next=True


    def tpc_callback(self, msg):
   
        meta = {}
        meta["task"] = self.task
        meta["waypoint"] = self.waypoint
        meta["time"] = self.dt_text
        meta["topic"] = '/transform_pc2/depth/points'
        
        msg_store = MessageStoreProxy(collection='patrol_data')
        msg_store.insert(msg,meta)

    def rgpc_callback(self, msg):
        if self.save_next :
            meta = {}
            meta["task"] = self.task
            meta["waypoint"] = self.waypoint
            meta["time"] = self.dt_text
            meta["topic"] = '/head_xtion/depth_registered/points'
            
            msg_store = MessageStoreProxy(collection='patrol_data')
            msg_store.insert(msg,meta)
            self.save_next=False
        


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
            if j.name == '3Dsnapshot' :
                filename = "3D_%s_%s.bag" %(userdata.task_name,targ)
                bashCommand = "timeout 10 rosbag record %s -l 1 -O ~/storage/%s" %(j.args[0],filename)
                print bashCommand
                os.system(bashCommand)
            if j.name == 'scitos_ptu' :
                print "Scitos PTU:"
                print "Creating Action Server"
                ptu_client = actionlib.SimpleActionClient('ptu_pan_tilt', scitos_ptu.msg.PanTiltAction)
                print "Done"
                ptu_client.wait_for_server()
                ptugoal = scitos_ptu.msg.PanTiltGoal()
                #argums = j.args.split(',') 
                ptugoal.pan_start=int(j.args[0])
                ptugoal.pan_step=int(j.args[1])
                ptugoal.pan_end=int(j.args[2])
                ptugoal.tilt_start=int(j.args[3])
                ptugoal.tilt_step=int(j.args[4])
                ptugoal.tilt_end=int(j.args[5])
                ptu_client.send_goal(ptugoal)
            
                # Waits for the server to finish performing the action.
                ptu_client.wait_for_result()
                # Prints out the result of executing the action
                result_ptu = nav_client.get_result()  # A FibonacciResult
                #print "result"
                print result_ptu
            if j.name == 'ptu_sweep' :
                sweep = PTUSweep_client(userdata.task_name, userdata.next_node.waypoint)
                res = sweep.execute_action(j.args)
                print res
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
            smach.StateMachine.add('PATROL_CHECKPOINT', PatrolCheckpoint(), transitions={'succeeded':'POINT_CHOOSE', 'aborted':'aborted'})
    
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