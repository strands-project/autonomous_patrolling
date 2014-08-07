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

from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf.msg import *
import topological_navigation.msg
import scitos_ptu.msg
import scitos_ptu_sweep.msg
import tf
import rosbag

def loadTask():
    print "openning "
 
    msg_store = MessageStoreProxy(collection='patrol_data')
    
    query_meta = {}
    query_meta["action"] = 'ptu_sweep'
    #query_meta["task"] = task_name
    #query_meta["stored_type"] = "strands_navigation_msgs/TopologicalNode"

    available = len(msg_store.query(geometry_msgs.msg.Pose, {}, query_meta)) > 0
    print  available
#    if available <= 0 :
#        rospy.logerr("Desired task '"+task_name+"' not in datacentre")
#        rospy.logerr("Available pointsets: "+str(available))
#        raise Exception("Can't find waypoints.")
#    else :
#        query_meta = {}
#        query_meta["type"] = "task_definition"
#        query_meta["task"] = task_name
#        message_list = msg_store.query(topological_patroller.msg.PatrolTask._type, {}, query_meta)
        
    #for i in message_list:
     #   print i

if __name__ == '__main__':
    loadTask()
