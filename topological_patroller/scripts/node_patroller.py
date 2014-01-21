#!/usr/bin/env python

import roslib; roslib.load_manifest('gather_smach')
from time import sleep
import rospy
import smach
import smach_ros
import datetime
import sys

from checkpoint import *
from starting_state import *
from point_choose import *
from scitos_msgs.msg import BatteryState
import scitos_apps_msgs.msg
from actionlib import *
from actionlib.msg import *
import topological_navigation.msg



def loadTask(inputfile):
    print "openning %s" %inputfile 
    fin = open(inputfile, 'r')

    checkpoints=[]
    line = fin.readline()
    while line:
        if line.startswith('CheckPoint:') :
            line = fin.readline()
            line = line.strip('\t')
            line = line.strip('\n')
            ckp=CheckPoint(line)
            line = fin.readline()
            if line.startswith('\tAction:') :
                line = fin.readline()
                actions=[]
                while line and not(line.startswith('CheckPoint:')) :
                    info= line.strip('\t')
                    info= info.strip('\n')
                    #inf = info.split(',',2)
                    #action = {'node':inf[0].strip(), 'action':inf[1].strip()}
                    print "Inserting:"
                    print info
                    actions.append(info)
                    line = fin.readline()
                #actions.pop(0)
                ckp._insert_actions(actions)
            checkpoints.append(ckp)
        else:
            line = fin.readline()
    fin.close()
    return checkpoints


class PatrolCheckpoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded','aborted'], 
                             input_keys=['pch_patrol_points','pch_current_node','pch_next_node'],
                             output_keys=['pch_current_node'])


    def execute(self, userdata):
        #print "point set:"
        #print userdata.pch_patrol_points
        print "current node:"
        print userdata.pch_current_node
        print "Going to:"
        print userdata.pch_next_node
        rospy.loginfo('Executing state PATROL_CHECKPOINT')

        nav_client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        nav_client.wait_for_server()
        navgoal = topological_navigation.msg.GotoNodeGoal()
        print "Requesting Navigation From %s to %s" %(userdata.pch_current_node, userdata.pch_next_node)
        navgoal.target = userdata.pch_next_node
        navgoal.origin = userdata.pch_current_node
    
        # Sends the goal to the action server.
        nav_client.send_goal(navgoal)
    
        # Waits for the server to finish performing the action.
        nav_client.wait_for_result()
        # Prints out the result of executing the action
        result = nav_client.get_result()  # A FibonacciResult
        print "result"
        print result
        print result.success        
        if result.success == False :
            userdata.pch_current_node="Unknown"
            return 'aborted'
        else :
            userdata.pch_current_node=userdata.pch_next_node
            print "navigation Succeeded"
        for i in userdata.pch_patrol_points :
            if i.name == userdata.pch_current_node:
                for j in i.actions:
                    print "executing!!!!!"
                    b = j[0].split(',',2)
                    print b[0]
                    if b[0] == 'sleep' :
                        print b[1]
                        c = int(b[1])
                        sleep(c)
        return 'succeeded'



def main():

    file_name=str(sys.argv[1])
    rospy.init_node('gather_smach')

    patrol_points = loadTask(file_name)
    for i in patrol_points:
        print i.name
        print i.actions
    #['WayPoint1','WayPoint2','WayPoint3','WayPoint4']
    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm0.userdata.sm_current_node="Unknown"
    sm0.userdata.sm_patrol_points=patrol_points
    #sm0.userdata.sm_next_node="Unknown"


    # Open the container
    with sm0:
        # Add states to the container
        smach.StateMachine.add('STARTING_STATE', StartingState(), transitions={'undock':'UNDOCKING_STATE', 'go_to_charge':'GOTO_CHARGING_POINT'}, 
                               remapping={'sts_initial_node':'sm_current_node', 'sts_current_node':'sm_current_node'})

        # POINT CHOOSE
        smach.StateMachine.add('POINT_CHOOSE', PointChoose(),
                               transitions={'next_waypoint':'PATROL_CHECKPOINT', 'back_to_home':'GOTO_CHARGING_POINT'},
                               remapping={'pc_patrol_points':'sm_patrol_points','pc_current_node':'sm_current_node','pc_next_node':'sm_next_node'})
        
        #GO TO CHECKPOINT
        smach.StateMachine.add('PATROL_CHECKPOINT', PatrolCheckpoint(),
                               transitions={'succeeded':'POINT_CHOOSE', 'aborted':'aborted'},
                               remapping={'pch_patrol_points':'sm_patrol_points','pch_current_node':'sm_current_node','pch_next_node':'sm_next_node'})


        def undocking_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.us_current_node="ChargingPoint"
                return 'succeeded'
            else :
                userdata.us_current_node="Unknown"
                return 'aborted'
                
        undocking_goal = scitos_apps_msgs.msg.ChargingGoal()
        undocking_goal.Command = 'undock'
        undocking_goal.Timeout = 1000
        smach.StateMachine.add('UNDOCKING_STATE', 
                               smach_ros.SimpleActionState('/chargingServer', 
                                                           scitos_apps_msgs.msg.ChargingAction, 
                                                           goal=undocking_goal,
                                                           result_cb=undocking_result_cb,
                                                           output_keys=['us_current_node']),
                               transitions={'succeeded':'POINT_CHOOSE', 'aborted':'aborted'},
                               remapping={'us_current_node':'sm_current_node'})


        charging_goal = scitos_apps_msgs.msg.ChargingGoal()
        charging_goal.Command = 'charge'
        charging_goal.Timeout = 1000
        smach.StateMachine.add('DOCKING_STATE', 
                               smach_ros.SimpleActionState('/chargingServer', scitos_apps_msgs.msg.ChargingAction, goal=charging_goal), 
                               {'succeeded':'STARTING_STATE',
                                'aborted':'aborted'})


        # GO TO NODE STATE
        #def goto_goal_cb(userdata, goal):
        #    goto_node_goal = topological_navigation.msg.GotoNodeGoal()
        #    goto_node_goal.target = userdata.sm_next_node
        #    goto_node_goal.origin = userdata.sm_current_node
        #    return goto_node_goal
            
        #def goto_result_cb(userdata, status, result):
        #    if status == GoalStatus.SUCCEEDED:
        #        userdata.sm_current_node=userdata.sm_next_node
        #        return 'succeeded'
        #    else :
        #        userdata.sm_current_node="Unknown"
        #        return 'aborted'
        #smach.StateMachine.add('GOTO_NODE', 
        #                       smach_ros.SimpleActionState('topological_navigation', 
        #                                                   topological_navigation.msg.GotoNodeAction, 
        #                                                   goal_cb=goto_goal_cb,
        #                                                   result_cb=goto_result_cb,
        #                                                   input_keys=['sm_current_node','sm_next_node'],
        #                                                   output_keys=['sm_current_node']), 
        #                       {'succeeded':'POINT_CHOOSE', 'aborted':'aborted'})

        # GO TO CHARGING POINT
        def goto_chp_cb(userdata, goal):
            goto_charge_goal = topological_navigation.msg.GotoNodeGoal()
            goto_charge_goal.target = "ChargingPoint"
            goto_charge_goal.origin = userdata.sm_current_node
            return goto_charge_goal
            
        def goto_chp_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.sm_current_node="ChargingPoint"
                return 'succeeded'
            else :
                userdata.sm_current_node="Unknown"
                return 'aborted'
                
        smach.StateMachine.add('GOTO_CHARGING_POINT', 
                               smach_ros.SimpleActionState('topological_navigation', 
                                                           topological_navigation.msg.GotoNodeAction, 
                                                           goal_cb=goto_chp_cb,
                                                           input_keys=['sm_current_node','sm_next_node'],
                                                           result_cb=goto_chp_result_cb,
                                                           output_keys=['sm_current_node']), 
                               {'succeeded':'DOCKING_STATE',
                                'aborted':'aborted'})
                                

    sis = smach_ros.IntrospectionServer('server_name', sm0, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm0.execute()
    sleep(10)
    rospy.signal_shutdown('All done.')
    sis.stop()


if __name__ == '__main__':
    main()
