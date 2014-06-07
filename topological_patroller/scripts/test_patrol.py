#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
#import topological_navigation.msg
import topological_patroller.msg

class patrol_client(object):
    
    def __init__(self, task) :
        
        #rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('patrol_server', topological_patroller.msg.DoPatrolAction)
        
        print "Requesting Task %s" %task
        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")
    
        navgoal = topological_patroller.msg.DoPatrolGoal()
    
        print "Requesting Task %s" %task
        
        navgoal.task_name = task

        # Sends the goal to the action server.
        self.client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        print ps

    #def _on_node_shutdown(self):
        #self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('patrol_test')
    ps = patrol_client(sys.argv[1])
