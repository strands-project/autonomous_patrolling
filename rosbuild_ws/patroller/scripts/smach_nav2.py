#! /usr/bin/env python
import time
import random

import roslib; roslib.load_manifest('strands_executive')
import rospy


#Import smach
import smach
import smach_ros
from smach_ros import SimpleActionState

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *





MAX_BATTERY=10


class GoTo(smach.State):
    def __init__(self, base_goal):
        smach.State.__init__(self,
            outcomes    = ['success', 'failure', 'low_battery' ],
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
        
        battery_level=userdata.battery_input-1
        userdata.battery_output=battery_level
        if battery_level<3:
            rospy.loginfo("Low Battery")
            return 'low_battery'

        time.sleep(0.3) #avoid jumping out of a state immediately after entering it - actionlib bug
        return 'success'

      


class PointReader(smach.State):
    def __init__(self, base_goal):
        smach.State.__init__(self,
            outcomes    = ['goto_point', 'low_battery' ],
            output_keys=['goal_pose']
            
        )

	with open('/home/computing/strands_morse/strands_executive/plans/points.txt', 'r') as f:
		self.points=f.read()



	with open('/home/computing/strands_morse/strands_executive/plans/plan.txt', 'r') as f:
		self.plan=f.read()

	self.current_point=1



    def execute(self,userdata):

	next_goal_id=self.plan[current_point]
	goal_pose=[self.points[2*next_goal_id,2*next_goal_id+1]
     	
        return 'success'




class ChargeBattery(smach.State):
    def __init__(self, base_goal):
        smach.State.__init__(self,
            outcomes    = ['goto1', 'goto2','goto3','goto4','failure'],
            input_keys=['battery_input'],
            output_keys=['battery_output']
            
        )

        rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient(
            'move_base',
            MoveBaseAction
        )
        self.base_goal=base_goal
        self.baseClient.wait_for_server()
        rospy.loginfo("Base client initialized")

    def execute(self,userdata):
        rospy.loginfo("Moving the base.")
        if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
            return 'abort'


        self.baseClient.send_goal(self.base_goal)

        self.baseClient.wait_for_result()

        result=self.baseClient.get_state()


    #    while result == GoalStatus.PENDING or result == GoalStatus.ACTIVE:
     #       result=self.baseClient.get_state()
      #      if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
       #         return 'abort'

        if result != GoalStatus.SUCCEEDED:
            return 'failure'

        battery_level=userdata.battery_input;

        battery_level=battery_level-1
        for i in xrange(0,MAX_BATTERY-battery_level):
            battery_level=battery_level+1
            rospy.loginfo("Charging Battery - Level %s", battery_level)
            time.sleep(0.5);

        userdata.battery_output=battery_level
        return 'goto'+`random.randint(1,4)`
        








def main():


    rospy.init_node('smach_nav')

    frame_id="/map"

    goal1 = move_base_msgs.msg.MoveBaseGoal()
    goal1.target_pose.header.frame_id = frame_id
    goal1.target_pose.header.stamp = rospy.Time.now()
    goal1.target_pose.pose.position.x = 0.0
    goal1.target_pose.pose.position.y = -3.0
    goal1.target_pose.pose.position.z = 0.0
    goal1.target_pose.pose.orientation.x = 0.0
    goal1.target_pose.pose.orientation.y = 0.0
    goal1.target_pose.pose.orientation.z = 0.0
    goal1.target_pose.pose.orientation.w = 1.0


    goal2 = move_base_msgs.msg.MoveBaseGoal()
    goal2.target_pose.header.frame_id = frame_id
    goal2.target_pose.header.stamp = rospy.Time.now()
    goal2.target_pose.pose.position.x = 3.0
    goal2.target_pose.pose.position.y = -3.0
    goal2.target_pose.pose.position.z = 0.0
    goal2.target_pose.pose.orientation.x = 0.0
    goal2.target_pose.pose.orientation.y = 0.0
    goal2.target_pose.pose.orientation.z = 1.0
    goal2.target_pose.pose.orientation.w = 0.0


    goal3 = move_base_msgs.msg.MoveBaseGoal()
    goal3.target_pose.header.frame_id = frame_id
    goal3.target_pose.header.stamp = rospy.Time.now()
    goal3.target_pose.pose.position.x = 3.0
    goal3.target_pose.pose.position.y = 3.0
    goal3.target_pose.pose.position.z = 0.0
    goal3.target_pose.pose.orientation.x = 0.0
    goal3.target_pose.pose.orientation.y = 0.0
    goal3.target_pose.pose.orientation.z = 0.7
    goal3.target_pose.pose.orientation.w = 0.7




    goal4 = move_base_msgs.msg.MoveBaseGoal()
    goal4.target_pose.header.frame_id = frame_id
    goal4.target_pose.header.stamp = rospy.Time.now()
    goal4.target_pose.pose.position.x = 0.0
    goal4.target_pose.pose.position.y = 3.0
    goal4.target_pose.pose.position.z = 0.0
    goal4.target_pose.pose.orientation.x = 0.0
    goal4.target_pose.pose.orientation.y = 0.0
    goal4.target_pose.pose.orientation.z = 0.9
    goal4.target_pose.pose.orientation.w = 0.4

    battery_charger = move_base_msgs.msg.MoveBaseGoal()
    battery_charger.target_pose.header.frame_id = frame_id
    battery_charger.target_pose.header.stamp = rospy.Time.now()
    battery_charger.target_pose.pose.position.x = 0.0
    battery_charger.target_pose.pose.position.y = 0.0
    battery_charger.target_pose.pose.position.z = 0.0
    battery_charger.target_pose.pose.orientation.x = 0.0
    battery_charger.target_pose.pose.orientation.y = 0.0
    battery_charger.target_pose.pose.orientation.z = 0.0
    battery_charger.target_pose.pose.orientation.w = 1.0



    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])
    sm.userdata.battery_input = MAX_BATTERY
    with sm:
        smach.StateMachine.add('GOTO1', GoTo(goal1), 
                               transitions={'success':'GOTO2','failure':'aborted','low_battery':'CHARGE_BATTERY'},
                               remapping={'battery_output':'battery_input'})

        smach.StateMachine.add('GOTO2', GoTo(goal2), 
                               transitions={'success':'GOTO3','failure':'aborted','low_battery':'CHARGE_BATTERY'},
                               remapping={'battery_output':'battery_input'})

        smach.StateMachine.add('GOTO3', GoTo(goal3), 
                               transitions={'success':'GOTO4','failure':'aborted','low_battery':'CHARGE_BATTERY'},
                               remapping={'battery_output':'battery_input'})

        smach.StateMachine.add('GOTO4', GoTo(goal4), 
                               transitions={'success':'GOTO1','failure':'aborted','low_battery':'CHARGE_BATTERY'},
                               remapping={'battery_output':'battery_input'})


        smach.StateMachine.add('CHARGE_BATTERY', ChargeBattery(battery_charger), 
                               transitions={'goto1':'GOTO1','goto2':'GOTO2','goto3':'GOTO3','goto4':'GOTO4','failure':'aborted'},
                               remapping={'battery_output':'battery_input'})


 
    

    # Execute SMACH plan

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()






0
