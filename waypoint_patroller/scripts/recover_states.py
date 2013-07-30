import rospy

import smach
import smach_ros


from scitos_msgs.srv import ResetMotorStop
from scitos_msgs.msg import MotorStatus


from geometry_msgs.msg import Twist

MAX_BUMPER_RECOVERY_ATTEMPTS=5
MAX_MOVE_BASE_RECOVERY_ATTEMPTS=5


class RecoverMoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure'], input_keys=['fails']
        )
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self._vel_cmd = Twist()
        self._vel_cmd.linear.x=-0.1


    def execute(self,userdata):
        if userdata.fails<MAX_MOVE_BASE_RECOVERY_ATTEMPTS:
            for i in range(0,20): 
                self.vel_pub.publish(self._vel_cmd)
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                rospy.sleep(0.1)
                return 'succeeded'
        else:
            return 'failure'
        
        
        

class RecoverBumper(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure'],
            input_keys=['n_recover_tries_in'],
            output_keys=['n_recover_tries_out']
        )
        self.reset_motorstop = rospy.ServiceProxy('reset_motorstop', ResetMotorStop)
        self.is_recovered=False
        self.motor_monitor=rospy.Subscriber("/motor_status", MotorStatus, self.bumper_cb)

                


    def bumper_cb(self,msg):
        self.isRecovered=not msg.bumper_pressed

    def execute(self,userdata):
        n_tries=userdata.n_recover_tries_in
        if n_tries<MAX_BUMPER_RECOVERY_ATTEMPTS:
            n_tries=n_tries+1
            rospy.sleep(3*n_tries)
            self.reset_motorstop()
            rospy.sleep(0.1)
            if self.isRecovered:
                userdata.n_recover_tries_out=0
            else:           
                userdata.n_recover_tries_out=n_tries
            return 'succeeded'
        else:
            return 'failure'
        
