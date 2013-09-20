import rospy

import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *


from monitor_states import battery_monitor
from monitor_states import bumper_monitor
from monitor_states import stuck_on_carpet_monitor
from recover_states import RecoverMoveBase
from recover_states import RecoverBumper
from recover_states import RecoverStuckOnCarpet


#This file contains the monitored navigation with recovery behaviours state machine



MOVE_BASE_EXEC_TIMEOUT=rospy.Duration(600.0)
MOVE_BASE_PREEMPT_TIMEOUT=rospy.Duration(10.0)


#outcome maps for the concurrence container
#The cild termination callback decided when the concurrence container should be terminated, bases on the outcomes of its children. When it outputs True, the container terminates and the concurrence container outcome callback is called
def child_term_cb(outcome_map):
    if outcome_map['BUMPER_MONITOR'] == 'invalid' or outcome_map["BATTERY_MONITOR"]=="invalid" or outcome_map["STUCK_ON_CARPET_MONITOR"]=="invalid" or outcome_map["MOVE_BASE_SM"]=="succeeded" or outcome_map['MOVE_BASE_SM']=="failure":
        return True
    return False


#The concurrence container outcome callback maps the outcomes of the container's children into an outcome for the concurrence container itself   
def out_cb(outcome_map):
   # rospy.sleep(0.1) without this sleep, sometimes the concurrence container terminates before all its children terminate, and an error is printed. However, that does not affect the evolution, and I think that with the sleep sometimes the container blocks and never terminates
    if outcome_map['BUMPER_MONITOR'] == 'invalid':
        return 'bumper_pressed'
    if  outcome_map["BATTERY_MONITOR"]=="invalid":
        return "battery_low"
    if  outcome_map["STUCK_ON_CARPET_MONITOR"]=="invalid":
        return "stuck_on_carpet"        
    if outcome_map["MOVE_BASE_SM"]=="succeeded":
        return "succeeded"
    if outcome_map["MOVE_BASE_SM"]=="failure":
        return "failure"
        

        
#callback that build the move_base goal, from the input data        
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
    
#called after the move_base state terminates. Increases the number of move_base fails or resets it to 0 according to the move_base result
def move_base_result_cb(userdata,status,result):
    if status==GoalStatus.ABORTED:
        userdata.n_move_base_fails=userdata.n_move_base_fails+1
    else:
        if status==GoalStatus.SUCCEEDED:
            userdata.n_move_base_fails=0

            
    

    

    
    
#Move base + recovery behaviour. The number of move_base fails is sent from the move_base action state to the move_base recovery state.
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
        

#adds the bumper and battery monitors running concurrently, and the bumper recovery as an extra state in the navigation state machine        
def navigation(): 


        sm=smach.StateMachine(outcomes=['succeeded','bumper_failure','move_base_failure','battery_low'], input_keys=['goal_pose','going_to_charge'], output_keys['goal_pose'])
        with sm:
            
            #move_base state machine running in parallel with the bumper and battery monitor states. The input key 'goal_pose' is used to build the new move_base goal, and the input key 'going_to_charge' is used to define the behaviour of the battery monitor        
            monitored_move_base=smach.Concurrence(outcomes=['bumper_pressed','battery_low','stuck_on_carpet','succeeded','failure'],
                                                default_outcome='failure',
                                                child_termination_cb=child_term_cb,
                                                outcome_cb = out_cb,
                                                input_keys=[ 'goal_pose','going_to_charge']
                                                )
        
            with monitored_move_base:
                smach.Concurrence.add('BATTERY_MONITOR', battery_monitor())
                smach.Concurrence.add('BUMPER_MONITOR',bumper_monitor())
                smach.Concurrence.add('STUCK_ON_CARPET_MONITOR',stuck_on_carpet_monitor())
                smach.Concurrence.add('MOVE_BASE_SM',move_base_sm()
                            #remapping={'gripper_input':'userdata_input'}
                            )
#adds the bumper recovery behaviour. This state needs to be on the same level as the concurrence container that has the bumper monitor so that the monitor can be restarted after the bumper is recovered.             
            smach.StateMachine.add('MONITORED_MOVE_BASE',monitored_move_base,transitions={'bumper_pressed':'RECOVER_BUMPER','stuck_on_carpet':'RECOVER_STUCK_ON_CARPET','battery_low':'battery_low','succeeded':'succeeded','failure':'move_base_failure'})
            smach.StateMachine.add('RECOVER_BUMPER', RecoverBumper(),  transitions={'succeeded':'MONITORED_MOVE_BASE','failure':'bumper_failure'})
            smach.StateMachine.add('RECOVER_STUCK_ON_CARPET', RecoverStuckOnCarpet(),  transitions={'succeeded':'MONITORED_MOVE_BASE', 'failure':'RECOVER_STUCK_ON_CARPET'})
             
        return sm
          

          

          

    
    
    
    
    
    
    
    
