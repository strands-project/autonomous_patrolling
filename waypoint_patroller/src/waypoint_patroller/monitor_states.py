import rospy

import smach
import smach_ros
from smach import *
from smach_ros import *

from scitos_msgs.msg import MotorStatus, BatteryState
from ap_msgs.msg import NavStatus

from logger import Loggable

"""
A smach_ros  MonitorState that monitors the robot's battery. 
"""
class BatteryMonitor(smach_ros.MonitorState):
    def __init__(self, going_to_charge, very_low_battery=15, low_battery=35,
                       charged_battery=90):
        smach_ros.MonitorState.__init__(self, "/battery_state",
                                        BatteryState,
                                        self._callback)
        self.set_patroller_thresholds(very_low_battery, low_battery,
                                    charged_battery)
        self._going_to_charge=going_to_charge
        
    """ 
    Set the battery level thresholds.
    """
    def set_patroller_thresholds(self, very_low_battery, low_battery,
                               charged_battery):
        if very_low_battery is not None:
            self.VERY_LOW_BATTERY = very_low_battery
        if low_battery is not None:
            self.LOW_BATTERY = low_battery
        if charged_battery is not None:
            self.CHARGED_BATTERY = charged_battery
    
    """ Test the message and decide exit or not """
    def _callback(self,  ud,  msg):
        if msg.lifePercent < self.VERY_LOW_BATTERY:
            return False
        if self._going_to_charge:
            return msg.lifePercent < self.CHARGED_BATTERY
        else:
            return msg.lifePercent > self.LOW_BATTERY



"""
A smach_ros  MonitorState that monitors the robot's bumper. If the bumper get
pressed, this state to exit with outcome 'invalid'.
"""
class BumperMonitor(smach_ros.MonitorState, Loggable):
    def __init__(self,):
        smach_ros.MonitorState.__init__(self, "/motor_status",
                                        MotorStatus,
                                        self._callback)
    
    """ Test the message and decide exit or not """
    def _callback(self,  ud,  msg):
        # using msg.bumper_pressed does not work properly because sometimes the
        # bumper is pressed but no change of state is published
        if msg.motor_stopped and not msg.free_run:
            self.get_logger().log_bump()
            return False
        else:
            return True


class StuckOnCarpetMonitor(smach_ros.MonitorState, Loggable):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, "/nav_status",
                                        NavStatus,
                                        self._callback)
    
    """ Test the message and decide exit or not """
    def _callback(self,  ud,  msg):
        if  msg.carpet_stuck:
            self.get_logger().log_carpet_stuck()
            return False
        else:
            return True


            
class NavPauseMonitor(smach_ros.MonitorState, Loggable):
    def __init__(self,is_paused):
        self._is_paused=is_paused
        smach_ros.MonitorState.__init__(self, "/nav_status",
                                        NavStatus,
                                        self._callback,
                                        input_keys=['is_paused'])
    
    """ Test the message and decide exit or not """
    def _callback(self,  ud,  msg):
        if self._is_paused:
            if  msg.is_paused:
                return True
            else:
                #self.get_logger().log_carpet_stuck()
                return False
        else:
            if  msg.is_paused:
                #self.get_logger().log_carpet_stuck()
                return False
            else:
                return True


