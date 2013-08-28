import rospy

import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import *
from ros_mary_tts.msg import *

from scitos_msgs.srv import ResetMotorStop
from scitos_msgs.msg import MotorStatus


from geometry_msgs.msg import Twist





#this file has the recovery states that will be used when some failures are detected.
#there is a recovery behaviour for move_base and another for when the bumper is pressed


MAX_BUMPER_RECOVERY_ATTEMPTS=10
MAX_MOVE_BASE_RECOVERY_ATTEMPTS=5




class RecoverMoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure','preempted'], input_keys=['n_move_base_fails'] #we need the number of move_base fails as incoming data from the move_base action state, because it is not possible for this recovery behaviour to check if it was succeeded
        )
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self._vel_cmd = Twist()
        self._vel_cmd.linear.x=-0.1


    def execute(self,userdata):
        #move slowly backwards a bit. A better option might be to save the latest messages received in cmd_vel and reverse them
        for i in range(0,20): 
            self.vel_pub.publish(self._vel_cmd)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.2)
        
        #since there is no way to check if the recovery behaviour was successful, we always go back to the move_base action state with 'succeeded' until the number of failures treshold is reached        
        if userdata.n_move_base_fails<MAX_MOVE_BASE_RECOVERY_ATTEMPTS:     
            return 'succeeded'
        else:
            return 'failure'
        
        
        

class RecoverBumper(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure']
        )
        self.reset_motorstop = rospy.ServiceProxy('reset_motorstop', ResetMotorStop)
        self.motor_monitor=rospy.Subscriber("/motor_status", MotorStatus, self.bumper_cb)
        self.isRecovered=False
        self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        self.speak_goal= maryttsGoal()
        self.speak_goal.text='My bumper is being pressed. Please release it so I can move on!'
        self.speaker.wait_for_server()
                


    def bumper_cb(self,msg):
        self.isRecovered=not msg.motor_stopped

    #restarts the motors and check to see of they really restarted. Between each failure the waiting time to try and restart the motors again increases. This state can check its own success
    def execute(self,userdata):
        n_tries=1
        while True:
            rospy.sleep(3*n_tries)
            self.reset_motorstop()
            rospy.sleep(0.1)
            if self.isRecovered:
                return 'succeeded'
            if n_tries>1:            
                self.speaker.send_goal(self.speak_goal)
                #check if action was successful
            #if n_tries>MAX_BUMPER_RECOVERY_ATTEMPTS:
                #log error, warn people
            n_tries=n_tries+1

        
