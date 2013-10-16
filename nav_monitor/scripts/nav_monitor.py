#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from ap_msgs.msg import NavStatus
from ap_msgs.srv import PauseResumePatroller




class NavMonitor(object):
    def __init__(self):
        self.goal_z=0
        self.current_z=0
        
        self.n_fails=0
        self.MAX_FAILS=100
        
        
        rospy.init_node('nav_monitor')
        
        #stuck in carpet
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)   
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        #pause_nav
        self.pad_paused=False
        rospy.Subscriber("/teleop_joystick/joy",Joy,self.pad_callback)
        
        self.pub = rospy.Publisher('nav_status', NavStatus)
        self.pub_msg=NavStatus()
        
        #pause service
        self.service_pause=False        
        self.pause_service = rospy.Service('pause_resume_patroller', PauseResumePatroller, self.pause_service_handler)



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
            

    def odom_callback(self,msg):
        self.current_z=msg.twist.twist.angular.z
        
        
    def pad_callback(self,msg):
        if msg.buttons[4]==0:
            self.pad_paused=False
        else:
            self.pad_paused=True
            
    def pause_service_handler(self, req):
        self.service_pause=not self.service_pause
        if self.service_pause:
            return 'paused'
        else:
            return 'resumed'
            
    

    def publisher(self):
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.pub_msg.is_paused=self.pad_paused or self.service_pause
            self.pub.publish(self.pub_msg)
            r.sleep()
            
            
    

    
    
    
if __name__ == '__main__':


    monitor=NavMonitor()
    
    monitor.publisher()
    
    
    rospy.spin()    
