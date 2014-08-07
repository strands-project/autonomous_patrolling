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
import topological_navigation.msg
import scitos_ptu.msg



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
            userdata.next_node = self.nodes[self.counter]
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

        targ  = userdata.next_node
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
            return 'aborted'
        else :          
            print "navigation Succeeded"
        for i in userdata.patrol_points :
            if i.waypoint == targ:
                for j in i.action :
                    print "executing!!!!!"
                    print j.name
                    if j.name == 'sleep' :
                        print j.args[0]
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
                        print "result"
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

        #for i in message_list:
         #   print i

    return  message_list[0][0]


def execute_patrol(task_name) :
    patrol_points_list = loadTask(task_name)
    #print patrol_points
    patrol_points = patrol_points_list.patrol
    for i in patrol_points :
        print i.waypoint
        for j in i.action:
            print j.name,j.args

    print "SMACH"

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm0.userdata.patrol_points=patrol_points
    sm0.userdata.task_name=task_name
    # Open the container

    with sm0:
        # POINT CHOOSE
        smach.StateMachine.add('POINT_CHOOSE', PointChoose(), transitions={'next_waypoint':'PATROL_CHECKPOINT', 'back_to_home':'aborted'})
                
        #GO TO CHECKPOINT
        smach.StateMachine.add('PATROL_CHECKPOINT', PatrolCheckpoint(), transitions={'succeeded':'POINT_CHOOSE', 'aborted':'aborted'})


    sis = smach_ros.IntrospectionServer('server_name', sm0, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm0.execute()
    sleep(10)
    rospy.signal_shutdown('All done.')
    sis.stop()


if __name__ == '__main__':

    task_name = sys.argv[1]
    rospy.init_node('patrol')
    execute_patrol(task_name)
    


    
