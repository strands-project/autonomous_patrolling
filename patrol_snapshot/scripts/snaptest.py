#! /usr/bin/env python

import rospy
# Brings in the SimpleActionClient
import actionlib
import patrol_snapshot.msg


def snap_client():
    
    client = actionlib.SimpleActionClient('patrol_snapshot', patrol_snapshot.msg.PatrolSnapshotAction)
    
    client.wait_for_server()
    rospy.loginfo(" ... Init done")

    goal = patrol_snapshot.msg.PatrolSnapshotGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    rospy.init_node('snap_test')
    ps = snap_client()
    print ps