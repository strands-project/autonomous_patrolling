#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import *


class PatrolControl():

    def __init__(self):
        s_on = rospy.Service('/patrol/resume', Empty, self.unpause_patrol)
        s_off = rospy.Service('/patrol/pause', Empty, self.pause_patrol)


    def pause_patrol(self, req):
        rospy.wait_for_service('/sweep_save/off')
        rospy.wait_for_service('/save_robot_pose/off')
        try:
            s = rospy.ServiceProxy('/sweep_save/off', Empty)
            s()
            e = rospy.ServiceProxy('/save_robot_pose/off', Empty)
            e()
            rospy.set_param('/topological_patroller/execute',False)
            print "Patrol paused!"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return EmptyResponse()

    def unpause_patrol(self, req):
        rospy.wait_for_service('/sweep_save/on')
        rospy.wait_for_service('/save_robot_pose/on')
        try:
            s = rospy.ServiceProxy('/sweep_save/on', Empty)
            s()
            e = rospy.ServiceProxy('/save_robot_pose/on', Empty)
            e()
            rospy.set_param('/topological_patroller/execute',True)
            print "Patrol resumed!"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('patrol_control')
    pt = PatrolControl()
    rospy.spin()

