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


from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf.msg import *
import topological_navigation.msg
import scitos_ptu.msg
import scitos_ptu_sweep.msg
import tf
import rosbag





class PatrolPTU(object):
    _feedback = topological_patroller.msg.DoPatrolFeedback()
    _result   = topological_patroller.msg.DoPatrolResult()

    def __init__(self, name):

        self.cancelled = False
        self._action_name = name
        
        rospy.loginfo("Creating action servers.")
        self._as = actionlib.SimpleActionServer('patrol_ptu', topological_patroller.msg.DoPatrolAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)

        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Ready ...")
        rospy.spin()

        self.save_sweeps = rospy.get_param('/topological_patroller/save_sweeps')
        print "PTU Sweep:"
        if self.save_sweeps :
            print "Saving"
        else :
            print "Not saving"

      
    def executeCallback(self, goal):

        self.save_next=False
        current_time = datetime.now()
        self.dt_text= current_time.strftime('%A, %B %d, at %H:%M hours')
        hourname= current_time.strftime('%d-%m-%y-%H-%M')
        self.task=task
        bagname = '/localhome/strands/storage/Patrol_'+task+'_'+waypoint+'_'+hourname+'.bag'
        self.waypoint=waypoint

        
        print "Creating Action Server Client"
        self.ptus_client = actionlib.SimpleActionClient('PTUSweep', scitos_ptu_sweep.msg.PTUSweepAction)
        print "Done"
        self.msg_store = MessageStoreProxy(collection='patrol_data')
        
        self.bag = rosbag.Bag(bagname, 'w')
        self.ptus_client.wait_for_server()
        
        #self.ptu_subs1 = rospy.Subscriber('/ptu_sweep/depth/points', sensor_msgs.msg.PointCloud2, self.dpth_callback,  queue_size=1)
        self.ptu_subs2 = rospy.Subscriber('/transform_pc2/depth/points', sensor_msgs.msg.PointCloud2, self.tpc_callback,  queue_size=1)
        self.ptu_subs3 = rospy.Subscriber('/head_xtion/depth_registered/points', sensor_msgs.msg.PointCloud2, self.rgpc_callback,  queue_size=1)
        self.pos_sub   = rospy.Subscriber('/robot_pose', geometry_msgs.msg.Pose, self.pose_callback,  queue_size=1)
        #self.tf_sub    = rospy.Subscriber('/tf', tf.msg.tfMessage, self.tf_callback,  queue_size=1)



        ptusgoal = scitos_ptu_sweep.msg.PTUSweepGoal()
        
        ptusgoal.max_pan = float(args[0])
        ptusgoal.max_tilt = float(args[1])
        ptusgoal.min_pan = float(args[2])
        ptusgoal.min_tilt = float(args[3])
        ptusgoal.pan_step = float(args[4])
        ptusgoal.tilt_step = float(args[5])

        self.ptus_client.send_goal(ptusgoal)
    
        self.ptus_client.wait_for_result()

        result_ptus = self.ptus_client.get_result()  # A FibonacciResult

        sleep(1)

        #self.ptu_subs1.unregister()
        self.ptu_subs2.unregister()
        self.ptu_subs3.unregister()
        self.pos_sub.unregister()
        #self.tf_sub.unregister()
        #self.bag.close()
        
        return result_ptus



        self._result.result = result
        self._as.publish_feedback(self._feedback)
        self._as.set_succeeded(self._result)


    def pose_callback(self,msg) :
#        print "P"
        if self.save_sweeps :
            meta = {}
            meta["task"] = self.task
            meta["action"] = 'ptu_sweep'
            meta["waypoint"] = self.waypoint
            meta["time"] = self.dt_text
            meta["topic"] = '/robot_pose'
            self.msg_store.insert(msg,meta)
            #self.msg_store.insert(msg)
            #self.bag.write('robot_pose', msg)
        self.pos_sub.unregister()

#    def tf_callback(self, msg):
#        meta = {}
#        meta["task"] = self.task
#        meta["action"] = 'ptu_sweep'
#        meta["waypoint"] = self.waypoint
#        meta["time"] = self.dt_text
#        meta["topic"] = '/tf'
#        self.msg_store.insert(msg,meta)
#        #self.bag.write('ptu_sweep/depth/points', msg)
#        self.msg_store.insert(msg)


    def tpc_callback(self, msg):
#        print "s2"
#        meta = {}
#        meta["task"] = self.task
#        meta["action"] = 'ptu_sweep'
#        meta["waypoint"] = self.waypoint
#        meta["time"] = self.dt_text
#        meta["topic"] = '/transform_pc2/depth/points'    
#        self.msg_store.insert(msg,meta)
        self.save_next=True
        #self.bag.write('transform_pc2/depth/points', msg)

    def rgpc_callback(self, msg):
        if self.save_next and self.save_sweeps :
#            print "s3"
            meta = {}
            meta["task"] = self.task
            meta["action"] = 'ptu_sweep'
            meta["waypoint"] = self.waypoint
            meta["time"] = self.dt_text
            meta["topic"] = '/head_xtion/depth_registered/points'
            self.msg_store.insert(msg,meta)
            self.save_next=False


    
    def preemptCallback(self):
        self.cancelled = True
        self._result.result = False
        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node('patrol_ptu')
    server = PatrolPTU('patrol_ptu')