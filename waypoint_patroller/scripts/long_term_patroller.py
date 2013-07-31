#! /usr/bin/env python
import sys
from random import randint
import rospy
import csv

import smach
import smach_ros


from navigation import navigation
from charging import dock_and_charge

from scitos_msgs.msg import BatteryState



CHARGE_BATTERY_TRESHOLD=20


class PointChooser(smach.State):
    def __init__(self, waypoints_name,is_random):
        smach.State.__init__(self,
            outcomes    = ['patrol','go_charge'],
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

        
        self.current_point=1
        self.n_points=len(self.points)
        
        self.is_random=is_random

        
        self.battery_life=100
        self.battery_monitor=rospy.Subscriber("/battery_state", BatteryState, self.bat_cb)

    def bat_cb(self,msg):
        self.battery_life=msg.lifePercent
        
        

    def execute(self,userdata):    
        rospy.sleep(1)
            
        if self.battery_life>CHARGE_BATTERY_TRESHOLD:
            if self.is_random:
                self.current_point=randint(1,self.n_points-1)
                current_row=self.points[self.current_point]
            else:
                current_row=self.points[self.current_point]
                self.current_point=self.current_point+1
                if self.current_point==self.n_points:
                    self.current_point=1
            userdata.goal_pose=current_row
            userdata.going_to_charge=0
            return 'patrol'
        else:
            current_row=self.points[0]
            userdata.goal_pose=current_row
            userdata.going_to_charge=1
            return 'go_charge'
        
        


def main():



    rospy.init_node('patroller')

    
    #Check if a waypoints file was given as argument
    if len(sys.argv)<2:
      rospy.logerr("No waypoints file given. Use rosrun waypoint_patroller patroller.py [path to csv waypoints file]. If you are using a launch file, see launch/patroller.launch for an example.")
      return 1
      
    waypoints_name=sys.argv[1]

    
    is_random=1;
    if len(sys.argv)>2:
      if sys.argv[2]=="false":
	is_random=0
	rospy.loginfo("Executing waypoint_patroller with sequential point selection.")
      else:
	rospy.loginfo("Executing waypoint_patroller with random point selection.")
    else:
      rospy.loginfo("Executing waypoint_patroller with random point selection.")

	  
  
	
    # Create a SMACH state machine
    long_term_patrol_sm = smach.StateMachine(outcomes=['succeeded','aborted'])
    with long_term_patrol_sm:
        smach.StateMachine.add('POINT_CHOOSER', PointChooser(waypoints_name,is_random),  transitions={'patrol':'PATROL_POINT','go_charge':'GO_TO_CHARGING_STATION'})
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
