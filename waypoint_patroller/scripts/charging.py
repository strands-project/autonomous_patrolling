import rospy

import smach
import smach_ros

from geometry_msgs.msg import Twist


from scitos_apps_msgs.srv import Charging

from monitor_states import bumper_monitor
from monitor_states import battery_monitor
from recover_states import RecoverBumper


class DockToChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure','preempted'],            
        )

        self.dock_behaviour = rospy.ServiceProxy('chargingSrv', Charging)

    def execute(self,userdata):
        self.dock_behaviour('charge',100)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        #add failure    
        return 'succeeded'
        
        
  
  

          
  
  
  
class UndockFromChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure','preempted'],            
        )
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self._vel_cmd = Twist()
        self._vel_cmd.linear.x=-0.2


    def execute(self,userdata):
        for i in range(0,20): 
            self.vel_pub.publish(self._vel_cmd)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
            
    

        return 'succeeded'
        
        
        
#states used onnly for tests. replace by battery_monitor  
class ChargingState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['invalid','valid'],            
        )
        
        #self.reset_motorstop = rospy.ServiceProxy('reset_motorstop', ResetMotorStop)


    def execute(self,userdata):
        rospy.sleep(3)

        return 'invalid'
                
  
def child_term_cb(outcome_map):
    return True
 
def out_cb(outcome_map):
    if outcome_map['BUMPER_MONITOR'] == 'invalid':
        return 'bumper_pressed'
    if  outcome_map["UNMONITORED_DOCKING"]=="succeeded":
        return "succeeded"



def monitored_docking():
    
    conc=smach.Concurrence(outcomes=['bumper_pressed','succeeded','failure'],
                                                    default_outcome='failure',
                                                    child_termination_cb=child_term_cb,
                                                    outcome_cb = out_cb,
                                                    input_keys=['going_to_charge','n_recover_tries_in'],
                                                    output_keys=['n_recover_tries_out'])
                                                    
    with conc:                                                
    
        unmonitored_dock=smach.StateMachine(outcomes=['succeeded','failure','preempted'],input_keys=['going_to_charge'])    
        with unmonitored_dock:
            smach.StateMachine.add('DOCK_TO_CHARGING_STATION',DockToChargingStation(), transitions={'succeeded':'CHARGING','failure':'DOCK_TO_CHARGING_STATION'} )
            smach.StateMachine.add('CHARGING',battery_monitor(), transitions={'invalid':'UNDOCK_FROM_CHARGING_STATION','valid':'UNDOCK_FROM_CHARGING_STATION'} )
           #smach.StateMachine.add('CHARGING',ChargingState(), transitions={'invalid':'UNDOCK_FROM_CHARGING_STATION','valid':'CHARGING'} ) #test: change afterwards
            smach.StateMachine.add('UNDOCK_FROM_CHARGING_STATION',UndockFromChargingStation(), transitions={'succeeded':'succeeded','failure':'UNDOCK_FROM_CHARGING_STATION'} )
            
        
        smach.Concurrence.add('UNMONITORED_DOCKING',unmonitored_dock)
        smach.Concurrence.add('BUMPER_MONITOR',bumper_monitor())
        
    return conc    
    
    
    
    
def dock_and_charge():
    
    docking_sm=smach.StateMachine(outcomes=['succeeded','failure'], input_keys=['going_to_charge'])
    
    with docking_sm:
        
        smach.StateMachine.add('MONITORED_DOCK',monitored_docking(), transitions={'succeeded':'succeeded','failure':'failure','bumper_pressed':'RECOVER_BUMPER'},
                                remapping={'n_recover_tries_out':'prev_tries', 'n_recover_tries_in':'cur_tries'})
        smach.StateMachine.add('RECOVER_BUMPER', RecoverBumper(),  transitions={'succeeded':'MONITORED_DOCK'},
                                remapping={'n_recover_tries_in':'prev_tries', 'n_recover_tries_out':'cur_tries'})
        
    return docking_sm
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    