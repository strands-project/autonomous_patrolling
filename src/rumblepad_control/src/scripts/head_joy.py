#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, Joy


class HeadJoy():
	"A class to command the Scitos head from a joystick"

	PAN_INCREMENT=2
	TILT_INCREMENT=1
	MAX_PAN=80
	MIN_PAN=-80
	MAX_TILT=10
	MIN_TILT=-10

	def __init__(self):
		rospy.init_node('head_joy')
		self.pub = rospy.Publisher('/head/commanded_state', JointState)
		rospy.Subscriber("/joy", Joy, self.callback) 
		rospy.logdebug(rospy.get_name() + " setting up")
		self.currentPan=0
		self.currentTilt=0
		self.command = JointState() 
		self.command.name=["HeadPan", "HeadTilt"] 
		self.command.position=[self.currentPan, self.currentTilt] 

	def callback(self, joy): 
		rospy.logdebug(rospy.get_name() + ": I heard %s" % joy) 
		print joy.axes[3:5] 

		# update value from axes
		self.currentPan += joy.axes[3] * self.PAN_INCREMENT
		self.currentTilt += joy.axes[4] * self.TILT_INCREMENT
	
		# threshold value
		self.currentPan = self.currentPan if self.currentPan < self.MAX_PAN else self.MAX_PAN
		self.currentPan = self.currentPan if self.currentPan > self.MIN_PAN else self.MIN_PAN
		self.currentTilt = self.currentTilt if self.currentTilt < self.MAX_TILT else self.MAX_TILT
		self.currentTilt = self.currentTilt if self.currentTilt > self.MIN_TILT else self.MIN_TILT
		
		#update command
		self.command.position=[self.currentPan, self.currentTilt] 

		# publish
		self.pub.publish(self.command)

if __name__ == '__main__':
    head_joy = HeadJoy()
    rospy.spin()
