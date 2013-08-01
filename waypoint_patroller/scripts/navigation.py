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




MOVE_BASE_EXEC_TIMEOUT=rospy.Duration(600.0)
MOVE_BASE_PREEMPT_TIMEOUT=rospy.Duration(10.0)


def child_term_cb(outcome_map):
    if outcome_map['BUMPER_MONITOR'] == 'invalid' or outcome_map["BATTERY_MONITOR"]=="invalid" or outcome_map["MOVE_BASE_SM"]=="succeeded" or outcome_map['MOVE_BASE_SM']=="failure":
        return True
    return False

    
def out_cb(outcome_map):
   # rospy.sleep(0.1)
    if outcome_map['BUMPER_MONITOR'] == 'invalid':
        return 'bumper_pressed'
    if  outcome_map["BATTERY_MONITOR"]=="invalid":
        return "battery_low"
    if outcome_map["MOVE_BASE_SM"]=="succeeded":
        return "succeeded"
    if outcome_map["MOVE_BASE_SM"]=="failure":
        return "failure"
        

        
        
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
    if status==GoalStatus.ABORTED:
        userdata.n_move_base_fails=userdata.n_move_base_fails+1
    else:
        if status==GoalStatus.SUCCEEDED:
            userdata.n_move_base_fails=0

            
    

    

    
    

def move_base_sm():
    sm=smach.StateMachine(outcomes=['succeeded','failure','preempted'], input_keys=['goal_pose'])
    
    with sm:
        
        sm.userdata.n_move_base_fails=0
        
        
        smach.StateMachine.add('MOVE_BASE',
                    smach_ros.SimpleActionState('move_base',
                                      MoveBaseAction,
                                      goal_cb=move_base_goal_cb,
                                      result_cb=move_base_result_cb,
                                      input_keys=['goal_pose','n_move_base_fails'],
                                      output_keys=['n_move_base_fails'],
                                      exec_timeout=MOVE_BASE_EXEC_TIMEOUT,
                                      preempt_timeout=MOVE_BASE_PREEMPT_TIMEOUT
                                      ),
                    transitions={'succeeded':'succeeded','aborted':'RECOVER_MOVE_BASE','preempted':'preempted'},
                      #remapping={'gripper_input':'userdata_input'}
                    )
        smach.StateMachine.add('RECOVER_MOVE_BASE', RecoverMoveBase(),  transitions={'succeeded':'MOVE_BASE','failure':'failure'})
        
    return sm
        
        
def navigation(): 


        sm=smach.StateMachine(outcomes=['succeeded','bumper_failure','move_base_failure','battery_low'], input_keys=['goal_pose','going_to_charge'])
        with sm:
                    
            monitored_move_base=smach.Concurrence(outcomes=['bumper_pressed','battery_low','succeeded','failure'],
                                                default_outcome='failure',
                                                child_termination_cb=child_term_cb,
                                                outcome_cb = out_cb,
                                                input_keys=[ 'goal_pose','going_to_charge']
                                                )
        
            with monitored_move_base:
                smach.Concurrence.add('BATTERY_MONITOR', battery_monitor())
                smach.Concurrence.add('BUMPER_MONITOR',bumper_monitor())
                smach.Concurrence.add('MOVE_BASE_SM',move_base_sm()
                            #remapping={'gripper_input':'userdata_input'}
                            )
            
            smach.StateMachine.add('MONITORED_MOVE_BASE',monitored_move_base,transitions={'bumper_pressed':'RECOVER_BUMPER','battery_low':'battery_low','succeeded':'succeeded','failure':'move_base_failure'})
            smach.StateMachine.add('RECOVER_BUMPER', RecoverBumper(),  transitions={'succeeded':'MONITORED_MOVE_BASE','failure':'bumper_failure'})
             
        return sm
          

          

          

    
    
    
    
    
    
    
    