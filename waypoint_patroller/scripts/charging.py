import rospy

import smach
import smach_ros

from geometry_msgs.msg import Twist


from scitos_apps_msgs.srv import Charging
from scitos_apps_msgs.msg import ChargingAction, ChargingGoal


from monitor_states import bumper_monitor
from monitor_states import battery_monitor
from recover_states import RecoverBumper


#This file contains the monitored docking with recovery behaviours state machine


#Calls the docking service. will be changed to call a docking action when one is available
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
        
        
  
  

          
  
  
#undocks. Should be changed to used the undock service (eventually action, when it is ready) instead of what is here right now  
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
        
        
        


#outcome maps for the concurrence container
#The cild termination callback decided when the concurrence container should be terminated, bases on the outcomes of its children. When it outputs True, the container terminates and the concurrence container outcome callback is called
def child_term_cb(outcome_map):
    return True

#The concurrence container outcome callback maps the outcomes of the container's children into an outcome for the concurrence container itself    
def out_cb(outcome_map):
    if outcome_map['BUMPER_MONITOR'] == 'invalid':
        return 'bumper_pressed'
    if  outcome_map["UNMONITORED_DOCKING"]=="succeeded":
        return "succeeded"



#Docking + bumper recovery behaviour. The input key 'going_to_charge' defines the battery monitor state results. In this case, it trminates when the battery life goes above the charged battery treshold
def monitored_docking():
    
    conc=smach.Concurrence(outcomes=['bumper_pressed','succeeded','failure'],
                                                    default_outcome='failure',
                                                    child_termination_cb=child_term_cb,
                                                    outcome_cb = out_cb,
                                                    input_keys=['going_to_charge'])
                                                    
    with conc:                                                
    
        unmonitored_dock=smach.StateMachine(outcomes=['succeeded','failure','preempted'],input_keys=['going_to_charge'])    
        with unmonitored_dock:
            dock_goal = ChargingGoal()
            dock_goal.Command = "charge"
            dock_goal.Timeout = 100
            
            
            smach.StateMachine.add('DOCK_TO_CHARGING_STATION',
                                   # DockToChargingStation(),
                                   smach_ros.SimpleActionState('chargingServer', ChargingAction, goal=dock_goal), transitions={'succeeded':'CHARGING','aborted':'DOCK_TO_CHARGING_STATION'} )
            
            smach.StateMachine.add('CHARGING',battery_monitor(), transitions={'invalid':'UNDOCK_FROM_CHARGING_STATION','valid':'UNDOCK_FROM_CHARGING_STATION'} )
            
            smach.StateMachine.add('UNDOCK_FROM_CHARGING_STATION',UndockFromChargingStation(), transitions={'succeeded':'succeeded','failure':'UNDOCK_FROM_CHARGING_STATION'} )
            
        
        smach.Concurrence.add('UNMONITORED_DOCKING',unmonitored_dock)
        smach.Concurrence.add('BUMPER_MONITOR',bumper_monitor())
        
    return conc    
    
    
    
#adds the bumper recovery behaviour. This state needs to be on the same level as the concurrenc container that has the bumper monitor so that the monitor can be restarted after the bumper is recovered.     
def dock_and_charge():
    
    docking_sm=smach.StateMachine(outcomes=['succeeded','failure'], input_keys=['going_to_charge'])
    
    with docking_sm:
        
        smach.StateMachine.add('MONITORED_DOCK',monitored_docking(), transitions={'succeeded':'succeeded','failure':'failure','bumper_pressed':'RECOVER_BUMPER'})
        smach.StateMachine.add('RECOVER_BUMPER', RecoverBumper(),  transitions={'succeeded':'MONITORED_DOCK'})
        
    return docking_sm
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
