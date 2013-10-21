#! /usr/bin/env python

import sys
import rospy
import csv

import smach
import smach_ros

from waypoint_patroller.patroller import WaypointPatroller
from waypoint_patroller.logger import PatrollLogger

from dynamic_reconfigure.server import Server
from waypoint_patroller.cfg import PatrollerTresholdsConfig

import strands_datacentre.util
got_pymongo = strands_datacentre.util.check_for_pymongo()
if got_pymongo:
    import pymongo
    
class LongTermPatroller(object):
    """
    Constructor.
    :param waypoints_name: str, name of waypoints as appears in the datacentre
    :param is_random: bool, should the waypoints be visited in random order
    :param n_iterations: int, how many times to visit all the waypoints
    """
    def __init__(self, waypoints_name, is_random, n_iterations):
    
        # Create the main state machine
        self.long_term_patrol_sm = WaypointPatroller(waypoints_name, is_random,
                                                n_iterations)
        
        # Create a logger
        logger =  PatrollLogger("autonomous_patrolling")
        self.long_term_patrol_sm.set_logger(logger)
        
        # dynamic reconfiguration of battery tresholds
        self.srv = Server(PatrollerTresholdsConfig, self.reconfigure_callback)
    
        pass
        
    """ Dyanmic reconfigure callback for the battery tresholds """
    def reconfigure_callback(self, config, level):
        self.long_term_patrol_sm.set_patroller_thresholds(config.very_low_battery,
                                                        config.low_battery, 
                                                        config.charged_battery,
                                                        config.max_bumper_recovery_attempts,
                                                        config.max_move_base_recovery_attempts)
        #ParameterStore().CHARGED_BATTERY = config.charged_battery
        #ParameterStore().LOW_BATTERY = config.low_battery
        #ParameterStore().VERY_LOW_BATTERY = config.very_low_battery
        return config

    
    """ The Main start point for Long Term Patroller """
    def main(self):
        # Execute SMACH plan
        sis = smach_ros.IntrospectionServer('server_name',
                                            self.long_term_patrol_sm,
                                            '/SM_ROOT')
        sis.start()
        outcome = self.long_term_patrol_sm.execute()
    
        rospy.spin()
        sis.stop()


if __name__ == '__main__':
    rospy.init_node('patroller')
    
    # Check for connection to the datacentre
    got_mongo = strands_datacentre.util.wait_for_mongo()
    if not got_pymongo:
        sys.exit(1)
    if not got_mongo:
        sys.exit(1)

    # Check if a waypoints file was given as argument
    if len(sys.argv) < 2:
        rospy.logerr("No waypoints dataset name given. Use rosrun " 
                     "waypoint_patroller patroller.py [dataset name]. If you " 
                     "are using a launch file, see launch/patroller.launch for " 
                     "an example.")
        sys.exit(1)

    # waypoints file is a csv file with goal poses. The first line of the file
    # has the position in front of the charging station
    waypoints_name = sys.argv[1]

    is_random = 1
    if len(sys.argv) > 2:
        if sys.argv[2] == "false":
            is_random = 0
            rospy.loginfo(
                "Executing waypoint_patroller with sequential point selection.")
        else:
            rospy.loginfo(
                "Executing waypoint_patroller with random point selection.")
    else:
        rospy.loginfo(
            "Executing waypoint_patroller with random point selection.")

    n_iterations = -1  # infinite iterations
    if len(sys.argv) > 3:
        if int(sys.argv[3]) > 0:
            n_iterations = int(sys.argv[3])
            rospy.loginfo(
                "Executing waypoint_patroller for %d iterations.", n_iterations)
        else:
            rospy.loginfo(
                "Executing waypoint_patroller with infinite iterations")
    else:
        rospy.loginfo("Executing waypoint_patroller with infinite iterations")

        
    l =  LongTermPatroller(waypoints_name, is_random, n_iterations)
    l.main()
