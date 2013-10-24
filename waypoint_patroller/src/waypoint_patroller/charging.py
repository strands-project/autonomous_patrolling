import rospy

import smach
import smach_ros

from geometry_msgs.msg import Twist


from scitos_apps_msgs.srv import Charging
from scitos_apps_msgs.msg import ChargingAction, ChargingGoal


from monitor_states import BumperMonitor, BatteryMonitor, NavPauseMonitor
from recover_states import RecoverBumper
from logger import Loggable

"""
A smach_ros SimpleActionState state that calls the robot docking behaviour
"""
class DockToChargingStation(smach_ros.SimpleActionState, Loggable):
    def __init__(self):
        dock_goal = ChargingGoal()
        dock_goal.Command = "charge"
        dock_goal.Timeout = 100
        
        smach_ros.SimpleActionState.__init__(self,
                                             'chargingServer',
                                             ChargingAction,
                                             goal=dock_goal )
        
    def execute(self, ud):
        self.get_logger().log_charging_start()
        outcome = smach_ros.SimpleActionState.execute(self, ud)
        
        return outcome

"""
Undocks the robot from the station.

outcomes: 	succeeded
			failure
            preempted
                        
"""

class UndockFromChargingStation(smach_ros.SimpleActionState, Loggable):
    def __init__(self):
        dock_goal = ChargingGoal()
        dock_goal.Command = "undock"
        dock_goal.Timeout = 100
        
        smach_ros.SimpleActionState.__init__(self,
                                             'chargingServer',
                                             ChargingAction,
                                             goal=dock_goal )
        
    def execute(self, ud):
        self.get_logger().log_charging_start()
        outcome = smach_ros.SimpleActionState.execute(self, ud)
        
        return outcome


#class UndockFromChargingStation(smach.State, Loggable):
    #def __init__(self):
        #smach.State.__init__(self,
                             #outcomes=['succeeded',
                                       #'failure',
                                       #'preempted'],
                             #)
        #self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        #self._vel_cmd = Twist()
        #self._vel_cmd.linear.x = -0.2

    #def execute(self, userdata):
        #self.get_logger().log_charging_finish()
        #for i in range(0, 20):
            #self.vel_pub.publish(self._vel_cmd)
            #if self.preempt_requested():
                #self.service_preempt()
                #return 'preempted'
            #rospy.sleep(0.1)

        #return 'succeeded'

"""
A state machine that docks the robot to the charging station, waits until it is
charged, then undocks the robot. 
"""
class DockUndockBehaviour(smach.StateMachine, Loggable):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'failure',
                                                    'preempted'])
        
        self._battery_monitor =  BatteryMonitor(True)
        self._dock_to_charge = DockToChargingStation()
        self._undock_from_charge = UndockFromChargingStation()
        
        with self:
            smach.StateMachine.add('DOCK_TO_CHARGING_STATION',
                                   self._dock_to_charge,
                                   transitions={'succeeded': 'CHARGING',
                                                'aborted': 'DOCK_TO_CHARGING_STATION',
                                                'preempted':'preempted'})

            smach.StateMachine.add('CHARGING',
                                   self._battery_monitor,
                                   transitions={
                                       'invalid': 'UNDOCK_FROM_CHARGING_STATION',
                                       'valid': 'UNDOCK_FROM_CHARGING_STATION'})

            smach.StateMachine.add('UNDOCK_FROM_CHARGING_STATION',
                                   self._undock_from_charge,
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'succeeded',
                                                'preempted': 'preempted'})

    """ 
    Set the battery level thresholds.
    """
    def set_patroller_thresholds(self, very_low_battery, low_battery,
                               charged_battery):
        self._battery_monitor.set_patroller_thresholds(very_low_battery,
                                                   low_battery, 
                                                   charged_battery)
   

"""
A bumper aware version of DockUndockBeahviour. 
"""
class MonitoredDockUndockBehaviour(smach.Concurrence, Loggable):
    def __init__(self,  to_base=False):
        smach.Concurrence.__init__(self,
                                   outcomes=['bumper_pressed',
                                             'pause_requested',
                                             'succeeded',
                                             'failure'],
                                   default_outcome='failure',
                                   child_termination_cb=self.child_term_cb,
                                   outcome_cb=self.out_cb
                                   )
        self._dock_undock_bahaviour = DockUndockBehaviour()
        self._bumper_monitor = BumperMonitor()
        self._nav_pause_monitor=NavPauseMonitor(False)
        with self:
            smach.Concurrence.add('UNMONITORED_DOCKING', self._dock_undock_bahaviour)
            smach.Concurrence.add('BUMPER_MONITOR', self._bumper_monitor)
            smach.Concurrence.add('NAV_PAUSE_MONITOR', self._nav_pause_monitor)

    def child_term_cb(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        return True
    
    def out_cb(self, outcome_map):
        # determine what the outcome of this machine is
        if outcome_map['BUMPER_MONITOR'] == 'invalid':
            return 'bumper_pressed'
        if outcome_map["NAV_PAUSE_MONITOR"] == "invalid":
            return "pause_requested"
        if outcome_map["UNMONITORED_DOCKING"] == "succeeded":
            return "succeeded"
        
    """ 
    Set the battery level thresholds.
    """
    def set_patroller_thresholds(self, very_low_battery, low_battery,
                               charged_battery):
        self._dock_undock_bahaviour.set_patroller_thresholds(very_low_battery,
                                                   low_battery, 
                                                   charged_battery)



"""
Highest level charging behaviour. This will dock the robot to the charging
station, wait until the charge is sufficient, and then undock. The bumper is
monitored through out, and if pressed then the the robot attempts to revocer
using recover_states.RecoverBumper.

outcomes:	succeeded
			failure
            
input_keys:	going_to_charge
"""
class HighLevelDockUndockBehaviour(smach.StateMachine, Loggable):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded',
                                              'failure'])

        self._bump_monitored_dk_undk = MonitoredDockUndockBehaviour()
        self._recover_bumper =  RecoverBumper()
        self._nav_resume_monitor=NavPauseMonitor(True)
        with self:
            smach.StateMachine.add('MONITORED_DOCK',
                                   self._bump_monitored_dk_undk,
                                   transitions={'succeeded': 'succeeded',
                                                'failure': 'failure',
                                                'bumper_pressed': 'RECOVER_BUMPER',
                                                'pause_requested':'NAV_RESUME_MONITOR'
                                                }
                                   )
            smach.StateMachine.add('RECOVER_BUMPER',
                                   self._recover_bumper,
                                   transitions={'succeeded': 'MONITORED_DOCK' })
            smach.StateMachine.add('NAV_RESUME_MONITOR',
                                   self._nav_resume_monitor,
                                   transitions={'invalid': 'MONITORED_DOCK',
                                                'valid': 'MONITORED_DOCK',
                                                'preempted':'MONITORED_DOCK'})
        
    """ 
    Set the patrolelr level thresholds.
    """
    def set_patroller_thresholds(self, very_low_battery, low_battery,
                               charged_battery,max_bumper_recovery_attempts,max_move_base_recovery_attempts):
        self._bump_monitored_dk_undk.set_patroller_thresholds(very_low_battery,
                                                   low_battery, 
                                                   charged_battery)
        self._recover_bumper.set_patroller_thresholds(max_bumper_recovery_attempts)