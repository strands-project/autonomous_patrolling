import rospy

import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *


from monitor_states import BatteryMonitor, BumperMonitor
from recover_states import RecoverMoveBase,  RecoverBumper


MOVE_BASE_EXEC_TIMEOUT = rospy.Duration(600.0)
MOVE_BASE_PREEMPT_TIMEOUT = rospy.Duration(10.0)

"""
A SimpleActionState that sends a goal to the navigation stack using move_base

input keys: 	goal_pose			- geometry_msgs/Pose, goal pose in /map
                                      frame
				n_move_base_fails	- number of failures so far, will be 
	                                  incremented  and returned if fail
                                      
output keys:	n_move_base_fails	- see input.
"""
class MoveBaseActionState(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self,
                                             'move_base',
                                             MoveBaseAction,
                                             goal_cb=self.move_base_goal_cb,
                                             result_cb=self.move_base_result_cb,
                                             input_keys=['goal_pose',
                                                         'n_move_base_fails'],
                                             output_keys=['n_move_base_fails'],
                                             exec_timeout=MOVE_BASE_EXEC_TIMEOUT,
                                             preempt_timeout=MOVE_BASE_PREEMPT_TIMEOUT
                                             )
        
    """ callback that builds the move_base goal, from the input data """
    def move_base_goal_cb(self, userdata, goal):
        next_goal = move_base_msgs.msg.MoveBaseGoal()
        next_goal.target_pose.header.frame_id = "/map"
        next_goal.target_pose.header.stamp = rospy.Time.now()
        next_goal.target_pose.pose = userdata.goal_pose
    
        return next_goal
    
    
    """
    called after the move_base state terminates. Increases the number of
    move_base fails or resets it to 0 according to the move_base result
    """
    def move_base_result_cb(self, userdata, status, result):
        if status == GoalStatus.ABORTED:
            userdata.n_move_base_fails = userdata.n_move_base_fails + 1
        else:
            if status == GoalStatus.SUCCEEDED:
                userdata.n_move_base_fails = 0
              
                      

"""
Move the robot to a goal location, with some basic recovery attempts on failure.
Recovery is implemented in recover_states.RecoverMoveBase

outcomes: 	succeeded
			failed
            preempted
            
input keys:	goal_pose 
"""
class RecoverableMoveBase(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded',
                                              'failure',
                                              'preempted'],
                                    input_keys=['goal_pose'])

        #self.userdata.n_move_base_fails = 0
        with self:
            smach.StateMachine.add('MOVE_BASE',
                                   MoveBaseActionState(), 
                                   transitions={'succeeded': 'succeeded',
                                                'aborted':  'RECOVER_MOVE_BASE',
                                                'preempted': 'preempted'}
                                   )
            smach.StateMachine.add('RECOVER_MOVE_BASE',
                                   RecoverMoveBase(),
                                   transitions={'succeeded': 'MOVE_BASE',
                                                'failure': 'failure'} )
            
"""
A monitored version of RecoverableMoveBase. Adds checking for battery level
too low, and stoping move_base if bumper is pressed.

outcomes: 	bumper_pressed
			battery_low
            succeeded
            failure
            
input keys:	goal_pose
            going_to_charge
"""
class MonitoredRecoverableMoveBase(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(self,
                                   outcomes=['bumper_pressed',
                                             'battery_low',
                                             'succeeded',
                                             'failure'],
                                   default_outcome='failure',
                                   child_termination_cb=self.child_term_cb,
                                   outcome_cb=self.out_cb,
                                   input_keys=['goal_pose',
                                               'going_to_charge']
                                   )
        with self:
            smach.Concurrence.add('BATTERY_MONITOR', BatteryMonitor())
            smach.Concurrence.add('BUMPER_MONITOR', BumperMonitor())
            smach.Concurrence.add('MOVE_BASE_SM', RecoverableMoveBase())
    
    def child_term_cb(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        if ( outcome_map['BUMPER_MONITOR'] == 'invalid' or
             outcome_map["BATTERY_MONITOR"] == "invalid" or
             outcome_map["MOVE_BASE_SM"] == "succeeded" or
             outcome_map['MOVE_BASE_SM'] == "failure" ):
            return True
        return False
    
    def out_cb(self, outcome_map):
        # determine what the outcome of this machine is
        
        # rospy.sleep(0.1) without this sleep, sometimes the concurrence container
        # terminates before all its children terminate, and an error is printed.
        # However, that does not affect the evolution, and I think that with the
        # sleep sometimes the container blocks and never terminates
        if outcome_map['BUMPER_MONITOR'] == 'invalid':
            return 'bumper_pressed'
        if outcome_map["BATTERY_MONITOR"] == "invalid":
            return "battery_low"
        if outcome_map["MOVE_BASE_SM"] == "succeeded":
            return "succeeded"
        if outcome_map["MOVE_BASE_SM"] == "failure":
            return "failure"



"""
The highest level "goto position" state. This will use move_base to goto a goal
position, mean while checking the battery and the bumper. If move_base fails,
some recovary is attempted. If the bumper is pressed, but then released, it will
resume. If the battery is low, move_base action is stopped and the state exits.

outcomes:	succeeded			- got to position
			bumper_failure		- bumper is in constant contact
            move_base_failure	- move_base fails, can't recover
            battery_low			- battery is low, goal cancelled
            
input_keys:	goal_pose		- geometry_msgs/Pose, the target location in /map
							  frame
            going_to_charge	- bool, if going to charging station then don't
            				  cancel because of the battery monitor
"""
class HighLevelMoveBase(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'bumper_failure',
                                                    'move_base_failure',
                                                    'battery_low'],
                                          input_keys=['goal_pose',
                                                      'going_to_charge'] )
        with self:
            smach.StateMachine.add('MONITORED_MOVE_BASE',
                                   MonitoredRecoverableMoveBase(),
                                   transitions={'bumper_pressed': 'RECOVER_BUMPER',
                                                'battery_low': 'battery_low',
                                                'succeeded': 'succeeded',
                                                'failure': 'move_base_failure'})
            smach.StateMachine.add('RECOVER_BUMPER',
                                   RecoverBumper(),
                                   transitions={'succeeded': 'MONITORED_MOVE_BASE',
                                                'failure': 'bumper_failure'})
