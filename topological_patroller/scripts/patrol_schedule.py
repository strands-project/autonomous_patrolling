#!/usr/bin/env python

import rospy
from time import sleep
from datetime import datetime
import actionlib


from threading import Timer
import topological_navigation.msg
import upload_cloud_data.msg
import topological_patroller.msg


from scitos_msgs.msg import BatteryState
from std_msgs.msg import Float32


#from ros_datacentre_msgs.msg import MoveEntriesAction, MoveEntriesGoal, StringList
#from patrol_snapshot.msg import *


class patrol_schedule():
    _at_home=False
    _killall_timers=False
    _last_minute_added=70
    
    def __init__(self):
        rospy.loginfo("Starting Schedule")
        rospy.on_shutdown(self._on_node_shutdown)
        
        rospy.set_param('/topological_patroller/execute',True)
        #Here goes the schedule
        a=['august3d_sweeps',[0]]
        b=['august_snaps',[10,20,30,40,50]]
        self.schedule=[]
        self.schedule.append(a)
        self.schedule.append(b)
        self.pending=[]
        
        if not self.check_for_home():
            self.go_home()
        
        self.upload_data()

        t = Timer(10.0, self._time_callback)
        t.start()

        self.the_loop()

        print "init done"


    def _time_callback(self):
        is_executing = rospy.get_param('/topological_patroller/execute')
        #runmileage=self._current_mileage-self._starting_mileage

        wait_for_it= datetime.now()
        print "%d:%d Pending tasks:" %(wait_for_it.minute, wait_for_it.second)
        print self.pending
        if is_executing and wait_for_it.minute != self._last_minute_added :
            for i in self.schedule:
                if wait_for_it.minute in i[1]:
                    self.pending.append(i[0])
                    self._last_minute_added = wait_for_it.minute


        if not self._killall_timers :
            t = Timer(10.0, self._time_callback)
            t.start()

    def the_loop(self):
        
        while not self._killall_timers:
            is_executing = rospy.get_param('/topological_patroller/execute')
            if is_executing and len(self.pending)>0 and self._charger_level > 30 :
                task_to_do=self.pending.pop(0)
                self.do_patrol(task_to_do)
                
            else :
                if self._charger_level<=30:
                    self.go_home()
                    self.upload_data()
            rospy.sleep(rospy.Duration.from_sec(1))

    def do_patrol(self, task):
        self.upload_data()

        #rospy.on_shutdown(self._on_node_shutdown)
        client = actionlib.SimpleActionClient('patrol_server', topological_patroller.msg.DoPatrolAction)
        
        print "Requesting Task %s" %task
        
        client.wait_for_server()
        rospy.loginfo(" ... Init done")
    
        navgoal = topological_patroller.msg.DoPatrolGoal()
    
        print "Requesting Task %s" %task
        
        navgoal.task_name = task

        # Sends the goal to the action server.
        client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = client.get_result()
     
        self.go_home()
        self.upload_data()
        
        return ps
        

    def upload_data(self):
        rospy.loginfo("Uploading Patrol Data")
        client = actionlib.SimpleActionClient('upload_cloud_data', upload_cloud_data.msg.UploadAction)     
        client.wait_for_server()
        rospy.loginfo(" ... Init done")
    
        goal = upload_cloud_data.msg.UploadGoal()
        
        # Sends the goal to the action server.
        client.send_goal(goal)
    
        # Waits for the server to finish performing the action.
        client.wait_for_result()
    
        #return client.get_result()  



    def go_home(self):
        rospy.loginfo("Navigating To Charging Station")
        counter = 0
        nav_client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        nav_client.wait_for_server()
        navgoal = topological_navigation.msg.GotoNodeGoal()
        print "Requesting Navigation to Home"
        navgoal.target = 'ChargingPoint'
        
        success = False
        while success == False and counter < 5 :
            # Sends the goal to the action server.
            nav_client.send_goal(navgoal)
        
            # Waits for the server to finish performing the action.
            nav_client.wait_for_result()
            # Prints out the result of executing the action
            result = nav_client.get_result()
            print "navigation result"
            print result
            success = result.success
            counter += 1
            
        if counter < 5 :
            print "navigation Succeeded"
        
        return success

    def check_for_home(self):
        rec=True
        try:
            battery = rospy.wait_for_message('/battery_state', BatteryState, timeout=3.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get battery state")
            rec=False

        if rec:
            self._at_charger=battery.charging
        else : 
            self._at_charger=False

        rec=True
        try:
            mil = rospy.wait_for_message('/odom_mileage', Float32, timeout=3.0)
        except rospy.ROSException :
            rospy.logwarn("Failed to get mileage")
            rec=False

        if rec:
            self._starting_mileage=mil.data
        else : 
            self._starting_mileage=0.0


        rospy.Subscriber("/battery_state", BatteryState, self.charger_callback)
        rospy.Subscriber("/odom_mileage", Float32, self.mileage_callback)
        return self._at_charger
        
    def mileage_callback(self, data):
        #self._starting_mileage=data.data
        self._current_mileage=data.data

        
    def charger_callback(self, data) :
        self._at_charger=data.charging
        self._charger_level = data.lifePercent


    def _on_node_shutdown(self):
        self._killall_timers=True



if __name__ == '__main__':
    rospy.init_node('patrol_schedule')
    ps = patrol_schedule()
    rospy.spin()