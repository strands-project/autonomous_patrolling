#! /usr/bin/env python
import time
import random

import roslib; roslib.load_manifest('patroller')
import rospy
import csv


import smach
import smach_ros
from smach_ros import SimpleActionState

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *

from std_msgs.msg import String


frame_id="/map"



class GoTo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['success', 'failure'],
            input_keys=['goal_pose']
            
        )

        rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient(
            'move_base',
            MoveBaseAction
        )
        self.baseClient.wait_for_server()
        rospy.loginfo("Base client initialized")

    def execute(self,userdata):

        rospy.loginfo("Moving the base.")
        if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
            return 'abort'


        self.baseClient.send_goal(userdata.goal_pose)

        self.baseClient.wait_for_result()

        result=self.baseClient.get_state()
        

    #    while result == GoalStatus.PENDING or result == GoalStatus.ACTIVE:
     #       result=self.baseClient.get_state()
      #      if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
       #         return 'abort'

        if result != GoalStatus.SUCCEEDED:
            return 'failure'
        

        time.sleep(0.3) #avoid jumping out of a state immediately after entering it - actionlib bug
        return 'success'

      


class PointReader(smach.State):
    def __init__(self, file_name):
        smach.State.__init__(self,
            outcomes    = ['goto_point'],
            output_keys=['goal_pose']
            
        )


	self.points=[]
	with open(file_name, 'rb') as csvfile:
	     	reader = csv.reader(csvfile, delimiter=',')
		for row in reader:
			current_row=[]
			for element in row:
				current_row.append(float(element))
        		self.points.append(current_row)

	
	self.current_point=0
	self.n_points=len(self.points)


    def execute(self,userdata):



	next_goal = move_base_msgs.msg.MoveBaseGoal()

	current_row=self.points[self.current_point]
	next_goal.target_pose.header.frame_id = frame_id
	next_goal.target_pose.header.stamp = rospy.Time.now()
	next_goal.target_pose.pose.position.x=current_row[0]
	next_goal.target_pose.pose.position.y=current_row[1]
	next_goal.target_pose.pose.position.z=current_row[2]
	next_goal.target_pose.pose.orientation.x=current_row[3]
	next_goal.target_pose.pose.orientation.y=current_row[4]
	next_goal.target_pose.pose.orientation.z=current_row[5]
	next_goal.target_pose.pose.orientation.w=current_row[6]

	self.current_point=self.current_point+1
	if self.current_point==self.n_points:
		self.current_point=0

	userdata.goal_pose=next_goal

     	
        return 'goto_point'








class Talker(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['success', 'failure'],         
        )

	self.pub = rospy.Publisher('/say', String)

    def execute(self,userdata):

	
	message = "Waypoint reached. Checking..." 
        self.pub.publish(String(message))
        rospy.sleep(3.0)

	message = "Clear. Moving to next waypoint." 
        self.pub.publish(String(message))
        rospy.sleep(3.0)
	
        return 'success'






def main():


    rospy.init_node('patroller')

    frame_id="/map"


    #file_name='/home/computing/autonomous_patrolling/src/waypoint_recorder/waypoints/waypoints.csv'

    file_name=rospy.get_param("/patroller/waypoints")

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])
    with sm:
        smach.StateMachine.add('POINT_READER', PointReader(file_name), 
                               transitions={'goto_point':'GOING_TO_POINT'},
                               remapping={'goal_pose':'goal_pose'})

        smach.StateMachine.add('GOING_TO_POINT', GoTo(), 
                               transitions={'success':'TALKER','failure':'aborted'},
                               remapping={'goal_pose':'goal_pose'})

        smach.StateMachine.add('TALKER', Talker(), 
                               transitions={'success':'POINT_READER','failure':'aborted'})



 
    

    # Execute SMACH plan

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()






0
