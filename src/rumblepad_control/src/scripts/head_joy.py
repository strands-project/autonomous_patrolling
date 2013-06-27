#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, Joy


class HeadJoy():
	"A class to command the Scitos head from a joystick"

	PAN_INCREMENT=5
	TILT_INCREMENT=2
	MAX_PAN=90
	MIN_PAN=-90
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
		print joy.axes[3:4] 

		# update value from axes
		self.currentPan += joy.axes[3]*PAN_INCREMENT
		self.currentTilt += joy.axes[4]*TILT_INCREMENT
	
		# threshold value
		self.currentPan = self.currentPan if self.currentPan < MAX_PAN else MAX_PAN
		self.currentPan = self.currentPan if self.currentPan > MIN_PAN else MIN_PAN
		self.currentTilt = self.currentTilt if self.currentTilt < MAX_TILT else MAX_TILT
		self.currentTilt = self.currentTilt if self.currentTilt > MIN_TILT else MIN_TILT

		# publish
		pub.publish(command)

if __name__ == '__main__':
    head_joy = HeadJoy()
    rospy.spin()