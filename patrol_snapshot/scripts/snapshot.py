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

from strands_perception_people_msgs.msg import PedestrianLocations
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
        print "New snap requested"
        #Subscribers
        
        self.received = False
        count = 0
        self.node_sub = rospy.Subscriber('/current_node', String, self.nodeCallback, None, 1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.node_sub.unregister()
        
        self.received = False
        count = 0
        self.pose_sub = rospy.Subscriber('/robot_pose', Pose, self.Callback, None, 1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.pose_sub.unregister()

        self.received = False
        count = 0
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.Callback, None, 1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.laser_sub.unregister()

        self.received = False
        count = 0
        self.img_sub = rospy.Subscriber('/head_xtion/rgb/image_color', Image, self.Callback, None, 1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.img_sub.unregister()

        self.received = False
        count = 0
        self.depth_img_sub = rospy.Subscriber('/head_xtion/depth/image_rect', Image, self.Callback, None, 1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.depth_img_sub.unregister()
        
        self.received = False
        count = 0
        self.pc2_sub = rospy.Subscriber('/head_xtion/depth/points', PointCloud2, self.Callback, None, 1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.pc2_sub.unregister()

     
        self.received = False
        count = 0
        self.reg_sub = rospy.Subscriber('/head_xtion/depth_registered/points', PointCloud2, self.Callback,  queue_size=1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.reg_sub.unregister()

        
        self.received = False
        count = 0
        self.ped_sub = rospy.Subscriber('/pedestrian_localisation/localisations', PedestrianLocations, self.Callback, None, 1)
        while not self.received or count < 1000:
            sleep(0.01)
            count += 1
        self.ped_sub.unregister()

        self._result.success = True
        self._as.set_succeeded(self._result)


    def nodeCallback(self, msg):
        self.waypoint = msg.data
        meta = {}
        meta["action"] = 'patrol_snapshot'
        meta["waypoint"] = self.waypoint
        meta["time"] = self.dt_text
        self.msg_store.insert(msg,meta)
        self.node_sub.unregister()
        self.received = True


    def Callback(self, msg):
        if not self.received :
            meta = {}
            meta["action"] = 'patrol_snapshot'
            meta["waypoint"] = self.waypoint
            meta["time"] = self.dt_text
            self.msg_store.insert(msg,meta)
            self.received = True

    def preemptCallback(self):
        self._result.success = False
        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node("patrol_snapshot")
    ps = patrolSnap(rospy.get_name())
    rospy.spin()
