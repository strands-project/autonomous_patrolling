#!/usr/bin/env python

#import roslib; roslib.load_manifest('gather_smach')
from time import sleep
import rospy
import smach
import smach_ros
import datetime

from scitos_msgs.msg import BatteryState
import scitos_apps_msgs.msg


# define Initial State
class StartingState(smach.State):
    _charger_received=False
    _at_charger=False
    def __init__(self):
        smach.State.__init__(self, outcomes=['undock','go_to_charge'], output_keys = ['task_name'])
        self.counter = 0

    def execute(self, userdata):
        self._charger_received=False
        chargsub = rospy.Subscriber("/battery_state", BatteryState, self.charger_callback)
        rospy.loginfo('Executing Starting State')
        self.counter = 0
        timeout=0
        while (not self._charger_received) and timeout < 100:
            sleep(0.1)
            timeout=timeout+1
        if timeout >= 100 :
            rospy.loginfo('NO CHARGING INFORMATION RECEIVED')
        chargsub.unregister()
        sleep(1)
        if self._at_charger :
            wait_for_it=datetime.datetime.now()
            while (int(wait_for_it.minute)%2) != 0 :
                print "%d:%d" %(wait_for_it.minute, wait_for_it.second)
                wait_for_it=datetime.datetime.now()
                sleep(1)
            print 'the time has come'
            print wait_for_it
            patrol_name = wait_for_it.strftime('%Y-%m-%d_%H-%M')
            print "Starting Patrol %s" %patrol_name
            userdata.task_name=patrol_name
            return 'undock'
        else:
            return 'go_to_charge'

    def charger_callback(self, data) :
        rospy.loginfo(rospy.get_name() + ": I heard %s" % data.charging)
        self._at_charger=data.charging
        self._charger_received=True


