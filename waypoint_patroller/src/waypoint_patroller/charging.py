import rospy

import smach
import smach_ros

from geometry_msgs.msg import Twist


from scitos_apps_msgs.srv import Charging
from scitos_apps_msgs.msg import ChargingAction, ChargingGoal


from monitor_states import BumperMonitor, BatteryMonitor 
from recover_states import RecoverBumper


"""
A smach_ros SimpleActionState state that calls the robot docking behaviour
"""
class DockToChargingStation(smach_ros.SimpleActionState):
    def __init__(self):
        dock_goal = ChargingGoal()
        dock_goal.Command = "charge"
        dock_goal.Timeout = 100
        
        smach_ros.SimpleActionState.__init__(self,
                                             'chargingServer',
                                             ChargingAction,
                                             goal=dock_goal )

"""
Undocks the robot from the station.
TODO: Does not currently use the undocking action

outcomes: 	succeeded
			failure
            preempted
            
"""
class UndockFromChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'failure',
                                       'preempted'],
                             )
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self._vel_cmd = Twist()
        self._vel_cmd.linear.x = -0.2

    def execute(self, userdata):
        for i in range(0, 20):
            self.vel_pub.publish(self._vel_cmd)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)

        return 'succeeded'

"""
A state machine that docks the robot to the charging station, waits until it is
charged, then undocks the robot. 
"""
class DockUndockBehaviour(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'failure',
                                                    'preempted'],
                                          input_keys=['going_to_charge'])
        
        with self:
            smach.StateMachine.add('DOCK_TO_CHARGING_STATION',
                                   DockToChargingStation(),
                                   transitions={'succeeded': 'CHARGING',
                                                'aborted': 'DOCK_TO_CHARGING_STATION'})

            smach.StateMachine.add('CHARGING',
                                   BatteryMonitor(),
                                   transitions={
                                       'invalid': 'UNDOCK_FROM_CHARGING_STATION',
                                       'valid': 'UNDOCK_FROM_CHARGING_STATION'})

            smach.StateMachine.add('UNDOCK_FROM_CHARGING_STATION',
                                   UndockFromChargingStation(),
                                   transitions={'succeeded': 'succeeded',
                                                'failure': 'UNDOCK_FROM_CHARGING_STATION'})

"""
A bumper aware version of DockUndockBeahviour. 
"""
class BumpMonitoredDockUndockBehaviour(smach.Concurrence):
    def __init__(self,  to_base=False):
        smach.Concurrence.__init__(self,
                                   outcomes=['bumper_pressed',
                                             'succeeded',
                                             'failure'],
                                   default_outcome='failure',
                                   child_termination_cb=self.child_term_cb,
                                   outcome_cb=self.out_cb,
                                   input_keys=['going_to_charge']
                                   )
        with self:
            smach.Concurrence.add('UNMONITORED_DOCKING', DockUndockBehaviour())
            smach.Concurrence.add('BUMPER_MONITOR', BumperMonitor())

    def child_term_cb(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        return True
    
    def out_cb(self, outcome_map):
        # determine what the outcome of this machine is
        if outcome_map['BUMPER_MONITOR'] == 'invalid':
            return 'bumper_pressed'
        if outcome_map["UNMONITORED_DOCKING"] == "succeeded":
            return "succeeded"



"""
Highest level charging behaviour. This will dock the robot to the charging
station, wait until the charge is sufficient, and then undock. The bumper is
monitored through out, and if pressed then the the robot attempts to revocer
using recover_states.RecoverBumper.

outcomes:	succeeded
			failure
            
input_keys:	going_to_charge
"""
class BumpRecoverableDockUndockBehaviour(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded',
                                              'failure'],
                                    input_keys=['going_to_charge'])

    
        with self:
            smach.StateMachine.add('MONITORED_DOCK',
                                   BumpMonitoredDockUndockBehaviour(),
                                   transitions={'succeeded': 'succeeded',
                                                'failure': 'failure',
                                                'bumper_pressed': 'RECOVER_BUMPER'
                                                }
                                   )
            smach.StateMachine.add('RECOVER_BUMPER',
                                   RecoverBumper(),
                                   transitions={'succeeded': 'MONITORED_DOCK' })
    

