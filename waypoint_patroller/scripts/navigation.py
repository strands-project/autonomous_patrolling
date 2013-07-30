import rospy

import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *


from monitor_states import battery_monitor
from monitor_states import bumper_monitor
from recover_states import RecoverMoveBase
from recover_states import RecoverBumper


        
                
        


def child_term_cb(outcome_map):
    if outcome_map['BUMPER_MONITOR'] == 'invalid' or outcome_map["BATTERY_MONITOR"]=="invalid" or outcome_map["MOVE_BASE_SM"]=="succeeded":
        return True
    return False

    
def out_cb(outcome_map):
    rospy.sleep(0.1)
    if outcome_map['BUMPER_MONITOR'] == 'invalid':
        return 'bumper_pressed'
    if  outcome_map["BATTERY_MONITOR"]=="invalid":
        return "battery_low"
    if outcome_map["MOVE_BASE_SM"]=="succeeded":
        return "succeeded"

        
        
def move_base_goal_cb(userdata,goal):
    
    next_goal = move_base_msgs.msg.MoveBaseGoal()            
    next_goal.target_pose.header.frame_id = "/map"
    next_goal.target_pose.header.stamp = rospy.Time.now()
    next_goal.target_pose.pose.position.x=userdata.goal_pose[0]
    next_goal.target_pose.pose.position.y=userdata.goal_pose[1]
    next_goal.target_pose.pose.position.z=userdata.goal_pose[2]
    next_goal.target_pose.pose.orientation.x=userdata.goal_pose[3]
    next_goal.target_pose.pose.orientation.y=userdata.goal_pose[4]
    next_goal.target_pose.pose.orientation.z=userdata.goal_pose[5]
    next_goal.target_pose.pose.orientation.w=userdata.goal_pose[6]
    
    return next_goal
    

def move_base_result_cb(userdata,status,result):
    if status==4: #aborted
        userdata.fails=userdata.fails+1
    else:
        if status==3: #succeeded
            userdata.fails=0    
    
   

def move_base_sm():
    sm=smach.StateMachine(outcomes=['succeeded','failure','preempted'], input_keys=['goal_pose'])
    
    with sm:
        
        sm.userdata.fails=0
        
        smach.StateMachine.add('MOVE_BASE',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal_cb=move_base_goal_cb,
                                        result_cb=move_base_result_cb,
                                        input_keys=['goal_pose','fails'],
                                        output_keys=['fails']),
                      transitions={'succeeded':'succeeded','aborted':'RECOVER_MOVE_BASE','preempted':'preempted'},
                      #remapping={'gripper_input':'userdata_input'}
                      )
        smach.StateMachine.add('RECOVER_MOVE_BASE', RecoverMoveBase(),  transitions={'succeeded':'MOVE_BASE'})
        
    return sm
        
        
def navigation(): 


        sm=smach.StateMachine(outcomes=['succeeded','bumper_failure','move_base_failure','battery_low'], input_keys=['goal_pose','going_to_charge'])
        sm.userdata.cur_tries = 0
        sm.userdata.prev_tries = 0
        with sm:
                    
            monitored_move_base=smach.Concurrence(outcomes=['bumper_pressed','battery_low','succeeded','failure'],
                                                default_outcome='failure',
                                                child_termination_cb=child_term_cb,
                                                outcome_cb = out_cb,
                                                input_keys=[ 'goal_pose','going_to_charge','n_recover_tries_in'],
                                                output_keys=['n_recover_tries_out']
                                                )
        
            with monitored_move_base:
                smach.Concurrence.add('BATTERY_MONITOR', battery_monitor())
                smach.Concurrence.add('BUMPER_MONITOR',bumper_monitor())
                smach.Concurrence.add('MOVE_BASE_SM',move_base_sm()
                            #remapping={'gripper_input':'userdata_input'}
                            )
            
            smach.StateMachine.add('MONITORED_MOVE_BASE',monitored_move_base,transitions={'bumper_pressed':'RECOVER_BUMPER','battery_low':'battery_low','succeeded':'succeeded','failure':'move_base_failure'},
                                    remapping={'n_recover_tries_out':'prev_tries', 'n_recover_tries_in':'cur_tries'})
            smach.StateMachine.add('RECOVER_BUMPER', RecoverBumper(),  transitions={'succeeded':'MONITORED_MOVE_BASE','failure':'bumper_failure'},
                                    remapping={'n_recover_tries_in':'prev_tries', 'n_recover_tries_out':'cur_tries'})
             
        return sm
          

          

          

    
    
    
    
    
    
    
    