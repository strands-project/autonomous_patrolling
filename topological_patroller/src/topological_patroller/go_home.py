#!/usr/bin/env python

from time import sleep
import rospy
import smach
import smach_ros
import datetime

from scitos_msgs.msg import BatteryState
import scitos_apps_msgs.msg


class CheckForHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retry','abort','succeeded'])
        self.counter = 0
        print "retrying %d" %self.counter

    def execute(self, userdata):
        rospy.loginfo('Executing state RETRY_TASK')
        chargres=self.get_charger_state()
        if chargres and self._at_charger :
                print "Successfuly reached charging station"
                self.counter = 0
                return 'succeeded'
        else:
            print "retrying %d" %self.counter
            if self.counter < 3:
                self.counter += 1
                return 'retry'
            else:
                print "Couldn't reach Charging station Aborting"
                self.counter = 0
                return 'abort'

    def get_charger_state(self):
        self._charger_received=False
        chargsub = rospy.Subscriber("/battery_state", BatteryState, self.charger_callback)
        timeout=0
        while (not self._charger_received) and timeout < 100:
            sleep(0.1)
            timeout=timeout+1
        chargsub.unregister()
        if timeout >= 100 :
            rospy.loginfo('NO CHARGING INFORMATION RECEIVED')
            return False
        return True

    def charger_callback(self, data) :
        rospy.loginfo(rospy.get_name() + ": I heard %s" % data.charging)
        self._at_charger=data.charging
        self._charger_received=True