#!/usr/bin/env python

import rospy
from time import sleep
from datetime import datetime
import actionlib
#import flir_pantilt_d46.msg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import scitos_ptu_sweep.msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

#from strands_perception_people_msgs.msg import PedestrianLocations
#from strands_perception_people_msgs import PedestrianTrackingArray
from ros_datacentre.message_store import MessageStoreProxy
from patrol_snapshot.msg import *

class patrolSnap():
    # Create feedback and result messages
    #_feedback = scitos_ptu_sweep.msg.PTUSweepFeedback()
    #_result   = scitos_ptu_sweep.msg.PTUSweepResult()
    _result = patrol_snapshot.msg.PatrolSnapshotResult()
    
    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, patrol_snapshot.msg.PatrolSnapshotAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)

        current_time = datetime.now()
        self.dt_text= current_time.strftime('%A, %B %d, at %H:%M hours')

        self.msg_store = MessageStoreProxy(collection='patrol_data')
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")        

    def executeCallback(self, goal):
        
        current_time = datetime.now()
        self.dt_text= current_time.strftime('%A, %B %d, at %H:%M hours')
        print "New snap requested"
        #Subscribers
        print self.dt_text
        self.waypoint = 'None'
        
        print "/current_node"
        nod_rec=True
        try:
            msg = rospy.wait_for_message('/current_node', String, timeout=1.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get current node")
            nod_rec=False
        if nod_rec:
            self.waypoint = msg.data
            meta = {}
            meta["action"] = 'patrol_snapshot'
            meta["waypoint"] = self.waypoint
            meta["time"] = self.dt_text
            self.msg_store.insert(msg, meta)

        
        print "/robot_pose"
        nod_rec=True
        try:
            msg = rospy.wait_for_message('/robot_pose', Pose, timeout=1.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get robot pose")
            nod_rec=False
        if nod_rec:
            self.msg_store.insert(msg,meta)
        
  
        print "/scan"
        nod_rec=True
        try:
            msg = rospy.wait_for_message('/scan', LaserScan, timeout=1.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get scan")
            nod_rec=False
        if nod_rec:
            self.msg_store.insert(msg,meta)        
        
                

        print "/head_xtion/rgb/image_color"
        nod_rec=True
        try:
            msg = rospy.wait_for_message('/head_xtion/rgb/image_color', Image, timeout=1.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get image_color")
            nod_rec=False
        if nod_rec:
            self.msg_store.insert(msg,meta)


        print "/head_xtion/depth/image_rect"
        nod_rec=True
        try:
            msg = rospy.wait_for_message('/head_xtion/depth/image_rect_meters', Image, timeout=1.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get image_color")
            nod_rec=False
        if nod_rec:
            self.msg_store.insert(msg,meta)


        print '/head_xtion/depth/points'
        received = True
        try:
            msg = rospy.wait_for_message('/head_xtion/depth/points', PointCloud2, timeout=1.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get point cloud")
            received = False
        if received :        
            self.msg_store.insert(msg,meta)
        
        
        self._result.success = True
        self._as.set_succeeded(self._result)



    def preemptCallback(self):
        self._result.success = False
        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node("patrol_snapshot")
    ps = patrolSnap(rospy.get_name())
    rospy.spin()
