#!/usr/bin/env python

import roslib; roslib.load_manifest('gather_smach')
from time import sleep
import rospy
import smach
import smach_ros
import datetime


class CheckPoint(object):
    def __init__(self, name):
        self.name = name
        self.actions=[]
        
    def _insert_actions(self, action_name):
        self.actions.append(action_name)