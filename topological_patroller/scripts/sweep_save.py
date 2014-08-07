#!/usr/bin/env python


import rospy
import datetime
from time import sleep
from datetime import datetime
import time
import json
import sys


from ros_datacentre.message_store import MessageStoreProxy

from std_msgs.msg import String
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf.msg import *
import tf


class SweepSave():

    def __init__(self):
        self.task = 'none'
        self.waypoint = 'None'
        
        rospy.Subscriber('/patrol_server/current_task', String, self.task_callback,  queue_size=1)
        rospy.Subscriber('/ptu_sweep/current_node', String, self.node_callback,  queue_size=1)
        rospy.Subscriber('/ptu_sweep/robot_pose', geometry_msgs.msg.Pose, self.callback,  queue_size=1)
        rospy.Subscriber('/transform_pc2/depth/points', sensor_msgs.msg.PointCloud2, self.callback,  queue_size=1)
        rospy.Subscriber('/ptu_sweep/depth/points', sensor_msgs.msg.PointCloud2, self.callback,  queue_size=1)
        rospy.Subscriber('/ptu_sweep/rgb/image_color', sensor_msgs.msg.Image, self.callback,  queue_size=1)
        rospy.Subscriber('/ptu_sweep/joint_state', sensor_msgs.msg.JointState, self.callback,  queue_size=1)
        

    def task_callback(self,msg) :
        self.task = msg.data

    def node_callback(self,msg) :
        self.waypoint = msg.data
        
    def callback(self,msg) :
        current_time = datetime.now()
        dt_text= current_time.strftime('%A, %B %d %Y, at %H:%M hours')
        meta = {}
        meta["task"] = self.task
        meta["action"] = 'ptu_sweep'
        meta["waypoint"] = self.waypoint
        meta["time"] = dt_text
        #meta["topic"] = '/ptu_sweep/robot_pose'
        msg_store = MessageStoreProxy(collection='patrol_data')
        msg_store.insert(msg, meta)


if __name__ == '__main__':
    rospy.init_node("SweepSave")
    ps = SweepSave()
    rospy.spin()