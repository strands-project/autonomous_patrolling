#!/usr/bin/env python

#import roslib; roslib.load_manifest('gather_smach')
from time import sleep
import rospy
import smach
import smach_ros
import datetime

from checkpoint import *


class PointChoose(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_waypoint','back_to_home'], input_keys=['pc_patrol_points','task_name'], output_keys=['pc_next_node'] )
        self.counter = 0


    def execute(self, userdata):
        #print "point set:"
        #print userdata.pc_patrol_points.name
        #print "current node:"
        print "Executing patrol %s" %userdata.task_name
        rospy.loginfo('Executing state POINT_CHOOSE')
        if len(userdata.pc_patrol_points) > self.counter :
            userdata.pc_next_node = userdata.pc_patrol_points[self.counter].name
            print "Going to:"
            print userdata.pc_patrol_points[self.counter].name
            self.counter=self.counter+1
            return 'next_waypoint'
        else:
            userdata.pc_next_node = "ChargingPoint"
            print "Going to:"
            print "ChargingPoint"
            self.counter=0
            return 'back_to_home'