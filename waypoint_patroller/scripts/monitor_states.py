import rospy

import smach
import smach_ros
from smach import *
from smach_ros import *

from scitos_msgs.msg import MotorStatus
from scitos_msgs.msg import BatteryState

#ParameterStore is a singleton class that contains all the battery tresholds. It is used so that the battery_monitor can read the tresholds, which can be dynamically reconfigured by long_term_patroller.py
from parameter_store import ParameterStore


#this file has the monitor states that will run in parallel with other behaviours, using a smach concurrence container. These states subscribe to a given topic and remain active until their callback returns False.  When that happens they terminate and return outcome 'invalid'
#There is a monitor for the bumper status and another for the battery life







#true -> continue monitor
#false -> terminate and state returns 'invalid'
def bumper_cb(ud, msg):
    #using msg.bumper_pressed does not work properly because sometimes the bumper is pressed but no change of state is published
    if  msg.motor_stopped and not msg.free_run:
        return False
    else:
        return True
                        

def battery_cb(ud, msg):
    if msg.lifePercent<ParameterStore().VERY_LOW_BATTERY:
        return False
    if ud.going_to_charge:
       return  msg.lifePercent<ParameterStore().CHARGED_BATTERY
    else:
        return  msg.lifePercent>ParameterStore().LOW_BATTERY   
   
            



def bumper_monitor():    
    state=smach_ros.MonitorState("/motor_status", MotorStatus, bumper_cb)
    return state
  
     
   
        
def battery_monitor():   
    state=smach_ros.MonitorState("/battery_state", BatteryState, battery_cb,input_keys=['going_to_charge'])
    return state
    

  
