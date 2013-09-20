#! /usr/bin/env python
import sys
from random import shuffle
import rospy
import csv

import smach
import smach_ros


from navigation import navigation
from charging import dock_and_charge

from scitos_msgs.msg import BatteryState


from dynamic_reconfigure.server import Server
from waypoint_patroller.cfg import BatteryTresholdsConfig

#ParameterStore is a singleton class that contains all the battery tresholds. It is used so we that the updated tresholds can then be read by the battery monitor in monitor_states.py
from parameter_store import ParameterStore


import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from scitos_ptu.msg import *
from sensor_msgs.msg import JointState


#This file implements the higher level state machine for long term patrolling. It uses both the navigation and the dock and charge state machines


#reconfigures the battery tresholds
def reconfigure_callback(config, level):
    ParameterStore().CHARGED_BATTERY=config.charged_battery
    ParameterStore().LOW_BATTERY=config.low_battery
    ParameterStore().VERY_LOW_BATTERY=config.very_low_battery
    return config


class PanTilt(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['success', 'failure', 'not_defined'],
            input_keys=['goal_pose']

        )

        rospy.loginfo("Creating ptu movement client.")
        self.ptuClient = actionlib.SimpleActionClient(
            'ptu_pan_tilt',
            PanTiltAction
        )
        self.ptuClient.wait_for_server()
        rospy.loginfo("PanTilt client initialized")

    def execute(self,userdata):

        rospy.loginfo("PanTilting the Xtion.")
        if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
            return 'abort'


#        self.ptuClient.send_goal(userdata.ptu_pose)

#        self.ptuClient.wait_for_result()

#        result=self.ptuClient.get_state()

#        if result != GoalStatus.SUCCEEDED:
#            return 'failure'
	
	print 'PanTilt state', userdata.goal_pose


#the point chooser state checks the battery life, and if it is greater than CHARGE_BATTERY_TRESHOLD, sends the robot to a new patrol point. Otherwise, the robot is sent to the charging station (assumed to be the first point in the waypoints file). The ordering of visitng the patrolling points is either sequential or random, depending on a command-line argument, as is the number of iterations the robot should do until the state machine terminates with success
class PointChooser(smach.State):
    def __init__(self, waypoints_name,is_random,n_iterations):
        smach.State.__init__(self,
            outcomes    = ['patrol','go_charge','succeeded'],
            output_keys=['goal_pose','going_to_charge']
        )
        
        
        


        self.points=[]
        with open(waypoints_name, 'rb') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                for row in reader:
                        current_row=[]
                        for element in row:
                                current_row.append(float(element))
                        self.points.append(current_row)
        #takes the charging station point from the lists of points read from the csv file
        self.charging_station_pos=self.points[0]
        del(self.points[0])
        
        self.current_point=-1
        self.n_points=len(self.points)
        
        self.is_random=is_random
        self.n_iterations=n_iterations
        self.iterations_completed=0
        
        #rearranges the list of points to visit randomly
        if self.is_random:
            shuffle(self.points)

        
        self.battery_life=100
        self.battery_monitor=rospy.Subscriber("/battery_state", BatteryState, self.bat_cb)
        


        
        
        
    def bat_cb(self,msg):
        self.battery_life=msg.lifePercent
        
        

    def execute(self,userdata):    
        rospy.sleep(1)
            
        if self.battery_life>ParameterStore().LOW_BATTERY+5:
            self.current_point=self.current_point+1
            if self.current_point==self.n_points:
                self.iterations_completed=self.iterations_completed+1
                if self.iterations_completed == self.n_iterations:
                    return 'succeeded'
                self.current_point=0
                if self.is_random:
                    shuffle(self.points)
                
                     
            current_row=self.points[self.current_point]

            userdata.goal_pose=current_row
            userdata.going_to_charge=0
            return 'patrol'
        else:
            userdata.goal_pose=self.charging_station_pos
            userdata.going_to_charge=1
            return 'go_charge'
        
        


def main():



    rospy.init_node('patroller')
    
    #dynamic reconfiguration of battery tresholds
    srv = Server(BatteryTresholdsConfig, reconfigure_callback)


    
    #Check if a waypoints file was given as argument
    if len(sys.argv)<2:
      rospy.logerr("No waypoints file given. Use rosrun waypoint_patroller patroller.py [path to csv waypoints file]. If you are using a launch file, see launch/patroller.launch for an example.")
      return 1
      
    waypoints_name=sys.argv[1] #waypoints file is a csv file with goal poses. The first line of the file has the position in front of the charging station

    
    is_random=1;
    if len(sys.argv)>2:
        if sys.argv[2]=="false":
            is_random=0
            rospy.loginfo("Executing waypoint_patroller with sequential point selection.")
        else:
            rospy.loginfo("Executing waypoint_patroller with random point selection.")
    else:
        rospy.loginfo("Executing waypoint_patroller with random point selection.")
      
    n_iterations=-1 #infinite iterations
    if len(sys.argv)>3:
        if int(sys.argv[3])>0:
            n_iterations=int(sys.argv[3])
            rospy.loginfo("Executing waypoint_patroller for %d iterations.", n_iterations)
        else:
            rospy.loginfo("Executing waypoint_patroller with infinite iterations")
    else:
        rospy.loginfo("Executing waypoint_patroller with infinite iterations")

	
	
	
    # Create a SMACH state machine
    long_term_patrol_sm = smach.StateMachine(outcomes=['succeeded','aborted'])
    with long_term_patrol_sm:
        smach.StateMachine.add('POINT_CHOOSER', PointChooser(waypoints_name,is_random,n_iterations),  transitions={'patrol':'PATROL_POINT','go_charge':'GO_TO_CHARGING_STATION','succeeded':'succeeded'})
        smach.StateMachine.add('PATROL_POINT', navigation(),  
                                transitions={'succeeded':'PT_POINT', 'battery_low':'POINT_CHOOSER','bumper_failure':'aborted','move_base_failure':'POINT_CHOOSER'})
        smach.StateMachine.add('PT_POINT', PanTilt(),  
                                transitions={'succeeded':'POINT_CHOOSER', 'failure','POINT_CHOOSER', 'not_defined','POINT_CHOOSER'})
        smach.StateMachine.add('GO_TO_CHARGING_STATION', navigation(),  
                                transitions={'succeeded':'DOCK_AND_CHARGE', 'battery_low':'GO_TO_CHARGING_STATION','bumper_failure':'aborted','move_base_failure':'aborted'})
        smach.StateMachine.add('DOCK_AND_CHARGE',dock_and_charge() , transitions={'succeeded':'POINT_CHOOSER','failure':'aborted'})
        

    
 

    # Execute SMACH plan

    sis = smach_ros.IntrospectionServer('server_name', long_term_patrol_sm, '/SM_ROOT')
    sis.start()
    outcome = long_term_patrol_sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()






0
