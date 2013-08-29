#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ap_msgs.msg import NavStatus





class NavMonitor(object):
    def __init__(self):
        self.goal_z=0
        self.current_z=0
        
        self.n_fails=0
        self.MAX_FAILS=100
        
        rospy.init_node('nav_monitor')        
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)   
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.pub = rospy.Publisher('nav_status', NavStatus)
        self.pub_msg=NavStatus()


    def vel_callback(self,msg):
        self.goal_z=msg.angular.z
        if self.goal_z != 0 and self.current_z==0:
            self.n_fails=self.n_fails+1
        else:
            self.n_fails=0
        if self.n_fails>self.MAX_FAILS:
            self.pub_msg.carpet_stuck=True
        else:
            self.pub_msg.carpet_stuck=False
        self.pub.publish(self.pub_msg)

    


    def odom_callback(self,msg):
        self.current_z=msg.twist.twist.angular.z

        
    

    
    
    
if __name__ == '__main__':


    NavMonitor()
    
    
    rospy.spin()    
