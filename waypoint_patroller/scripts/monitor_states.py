import rospy

import smach
import smach_ros

from smach import *
from smach_ros import *

from scitos_msgs.msg import MotorStatus
from scitos_msgs.msg import BatteryState


CHARGED_BATTERY=99
LOW_BATTERY=15
VERY_LOW_BATTERY=5



#true -> continue monitor
#false -> terminate and return 'invalid'
def bumper_cb(ud, msg):
    if  msg.bumper_pressed:
        n_tries=ud.n_recover_tries_in
        ud.n_recover_tries_out=n_tries
        return False
    else:
        ud.n_recover_tries_out=0 #just to avoid warning when robot does a path without being bumped
        return True
                        

def battery_cb(ud, msg):
    if msg.lifePercent<VERY_LOW_BATTERY:
        return False
    if ud.going_to_charge:
       return  msg.lifePercent<CHARGED_BATTERY
    else:
        return  msg.lifePercent>LOW_BATTERY   
   
            



def bumper_monitor():    
    state=smach_ros.MonitorState("/motor_status", MotorStatus, bumper_cb,input_keys=['n_recover_tries_in'],output_keys=['n_recover_tries_out'])
    return state
  
     
   
        
def battery_monitor():   
    state=smach_ros.MonitorState("/battery_state", BatteryState, battery_cb,input_keys=['going_to_charge'])
    return state
    

  
