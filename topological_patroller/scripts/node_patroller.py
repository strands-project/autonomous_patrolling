#!/usr/bin/env python


from time import sleep
import rospy
import smach
import smach_ros
import datetime
import sys

from topological_patroller.checkpoint import *
from topological_patroller.starting_state import *
from topological_patroller.point_choose import *
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
                    inf = info.split(',',2)
                    #action = {'node':inf[0].strip(), 'action':inf[1].strip()}
                    ckp._insert_actions(inf)
                    #actions.append(info)
                    line = fin.readline()
                #actions.pop(0)
            checkpoints.append(ckp)
        else:
            line = fin.readline()
    fin.close()
    return checkpoints


def main():

    file_name=str(sys.argv[1])
    rospy.init_node('gather_smach')

    patrol_points = loadTask(file_name)
    for i in patrol_points:
        print i.name
        for j in i.actions:
            print j.name, j.args
    #['WayPoint1','WayPoint2','WayPoint3','WayPoint4']
    # Create a SMACH state machine

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm0.userdata.sm_patrol_points=patrol_points
    #sm0.userdata.sm_next_node="Unknown"


    # Open the container
    with sm0:
        # Add states to the container
        smach.StateMachine.add('STARTING_STATE', StartingState(), transitions={'undock':'UNDOCKING_STATE', 'go_to_charge':'GOTO_CHARGING_POINT'})




        sm_patrol = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys = ['sm_patrol_points','task_name'])

        with sm_patrol:

            # POINT CHOOSE
            smach.StateMachine.add('POINT_CHOOSE', PointChoose(),
                                   transitions={'next_waypoint':'PATROL_CHECKPOINT', 'back_to_home':'succeeded'},
                                   remapping={'pc_patrol_points':'sm_patrol_points','pc_next_node':'sm_next_node'})
            
            #GO TO CHECKPOINT
            smach.StateMachine.add('PATROL_CHECKPOINT', PatrolCheckpoint(),
                                   transitions={'succeeded':'POINT_CHOOSE', 'aborted':'aborted'},
                                   remapping={'pch_patrol_points':'sm_patrol_points','pch_next_node':'sm_next_node'})

        smach.StateMachine.add('PATROL', sm_patrol,
                               transitions={'succeeded':'GOTO_CHARGING_POINT','aborted':'GOTO_CHARGING_POINT','preempted':'GOTO_CHARGING_POINT'})



        undocking_goal = scitos_apps_msgs.msg.ChargingGoal()
        undocking_goal.Command = 'undock'
        undocking_goal.Timeout = 1000
        smach.StateMachine.add('UNDOCKING_STATE', 
                               smach_ros.SimpleActionState('/chargingServer', scitos_apps_msgs.msg.ChargingAction, goal=undocking_goal),
                               transitions={'succeeded':'PATROL', 'aborted':'aborted'})


        charging_goal = scitos_apps_msgs.msg.ChargingGoal()
        charging_goal.Command = 'charge'
        charging_goal.Timeout = 1000
        smach.StateMachine.add('DOCKING_STATE', 
                               smach_ros.SimpleActionState('/chargingServer', scitos_apps_msgs.msg.ChargingAction, goal=charging_goal), 
                               transitions={'succeeded':'STARTING_STATE','aborted':'aborted'})


        goto_charge_goal = topological_navigation.msg.GotoNodeGoal()
        goto_charge_goal.target = "ChargingPoint"
        
        smach.StateMachine.add('GOTO_CHARGING_POINT', 
                               smach_ros.SimpleActionState('topological_navigation',topological_navigation.msg.GotoNodeAction, goal=goto_charge_goal),
                               transitions= {'succeeded':'DOCKING_STATE','aborted':'aborted'})


    sis = smach_ros.IntrospectionServer('server_name', sm0, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm0.execute()
    sleep(10)
    rospy.signal_shutdown('All done.')
    sis.stop()


if __name__ == '__main__':
    main()
