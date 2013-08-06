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


#This file implements the higher level state machine for long term patrolling. It uses both the navigation and the dock and charge state machines


CHARGE_BATTERY_TRESHOLD=40


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
            
        if self.battery_life>CHARGE_BATTERY_TRESHOLD:
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
                                transitions={'succeeded':'POINT_CHOOSER', 'battery_low':'POINT_CHOOSER','bumper_failure':'aborted','move_base_failure':'POINT_CHOOSER'})
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
