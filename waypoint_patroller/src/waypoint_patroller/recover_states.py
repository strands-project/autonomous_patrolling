# TODO: Add speaking back in here
import rospy

import smach
import smach_ros

from scitos_msgs.srv import ResetMotorStop
from scitos_msgs.srv import EnableMotors
from ap_msgs.srv import BumperRecovered
from ap_msgs.srv import RobotMoved

from scitos_msgs.msg import MotorStatus
from geometry_msgs.msg import Twist

import actionlib
#from actionlib_msgs.msg import *
from ros_mary_tts.msg import *

from logger import Loggable

import marathon_touch_gui.client


# this file has the recovery states that will be used when some failures are
# detected. There is a recovery behaviour for move_base and another for when the
# bumper is pressed



class RecoverMoveBase(smach.State, Loggable):
    def __init__(self,max_move_base_recovery_attempts=5):
        smach.State.__init__(self,
                             # we need the number of move_base fails as
                             # incoming data from the move_base action state,
                             # because it is not possible for this recovery
                             # behaviour to check if it was succeeded
                             outcomes=['succeeded', 'failure', 'preempted'],
                             input_keys=['n_move_base_fails'],
                             output_keys=['n_move_base_fails'],
                             )
        #self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        #self._vel_cmd = Twist()
        #self._vel_cmd.linear.x = -0.1
        self.set_patroller_thresholds(max_move_base_recovery_attempts)
        

        self.enable_motors= rospy.ServiceProxy('enable_motors',
                                                  EnableMotors)
                                                  
        self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        self.speak_goal= maryttsGoal()
        self.speaker.wait_for_server()                                                  
                                                  

    def robot_moved_checker(self,req):
        self.speak_goal.text='Thank you! I will be on my way.'
        self.speaker.send_goal(self.speak_goal)
        self.speaker.wait_for_result()
        self.isRecovered=True
                                                  
                                                  
    def execute(self, userdata):
        # move slowly backwards a bit. A better option might be to save the
        # latest messages received in cmd_vel and reverse them
        #for i in range(0, 20):
            #self.vel_pub.publish(self._vel_cmd)
            #if self.preempt_requested():
                #self.service_preempt()
                #return 'preempted'
            #rospy.sleep(0.2)
        
        self.isRecovered=False
            
        self.enable_motors(False)    
        rospy.sleep(0.2)
        # since there is no way to check if the recovery behaviour was
        # successful, we always go back to the move_base action state with
        # 'succeeded' until the number of failures treshold is reached
        if userdata.n_move_base_fails < self.MAX_MOVE_BASE_RECOVERY_ATTEMPTS:
            self.get_logger().log_navigation_recovery_attempt(success=True,
                                                              attempts=userdata.n_move_base_fails)
            displayNo = rospy.get_param("~display", 0)
            on_completion = 'robot_moved'
            service_prefix = '/patroller'
            marathon_touch_gui.client.nav_fail(displayNo, service_prefix, on_completion)
            
            help_button_monitor=rospy.Service('/patroller/robot_moved', RobotMoved, self.robot_moved_checker)
            
            self.speak_goal.text='I am having problems moving. Please push me to a clear area.'
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()
            
            for i in range(0,40):
                if self.isRecovered:
                    break
                rospy.sleep(1)   
            
            marathon_touch_gui.client.display_main_page(displayNo)
            help_button_monitor.shutdown()
            return 'succeeded'
        else:
            userdata.n_move_base_fails=0
            self.get_logger().log_navigation_recovery_attempt(success=False,
                                                              attempts=userdata.n_move_base_fails)
            return 'failure'


            
    def set_patroller_thresholds(self, max_move_base_recovery_attempts):
        if max_move_base_recovery_attempts is not None:
            self.MAX_MOVE_BASE_RECOVERY_ATTEMPTS = max_move_base_recovery_attempts        
            
            
            
class RecoverBumper(smach.State, Loggable):
    def __init__(self,max_bumper_recovery_attempts=5):
        smach.State.__init__(self,
                             outcomes=['succeeded']
                             )
        self.reset_motorstop = rospy.ServiceProxy('reset_motorstop',
                                                  ResetMotorStop)
        self.enable_motors= rospy.ServiceProxy('enable_motors',
                                                  EnableMotors)                                          
        self.is_recovered = False
        self.motor_monitor = rospy.Subscriber("/motor_status",
                                              MotorStatus,
                                              self.bumper_monitor_cb)
                                              
        self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        self.speak_goal= maryttsGoal()
        self.speaker.wait_for_server()
        self.set_patroller_thresholds(max_bumper_recovery_attempts)
        

        
    def bumper_recovered_checker(self,req):
        self.reset_motorstop()    
        rospy.sleep(0.1)
        if self.isRecovered:
            self.speak_goal.text='Thank you! I will be on my way.'
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()
        else:
            self.speak_goal.text='Something is still wrong. Are you sure I am in a clear area?'
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()
        
    def bumper_monitor_cb(self, msg):
        self.isRecovered = not msg.bumper_pressed

    # restarts the motors and check to see of they really restarted.
    # Between each failure the waiting time to try and restart the motors
    # again increases. This state can check its own success
    def execute(self, userdata):
        help_button_monitor=rospy.Service('/patroller/bumper_recovered', BumperRecovered, self.bumper_recovered_checker)
        displayNo = rospy.get_param("~display", 0)
        n_tries=1
        while True:
            self.enable_motors(False)
            self.reset_motorstop()
            rospy.sleep(0.1)
            #for i in range(0,3*n_tries):
                #if self.isRecovered:
                    #help_button_monitor.shutdown()
                    #self.get_logger().log_bump_count(n_tries)
                    #marathon_touch_gui.client.display_main_page(displayNo)
                    #return 'succeeded' 
                #rospy.sleep(1)
            if self.isRecovered:
                help_button_monitor.shutdown()
                self.get_logger().log_bump_count(n_tries)
                marathon_touch_gui.client.display_main_page(displayNo)
                return 'succeeded'
                
            rospy.sleep(4*n_tries)
            
            
            if n_tries==2:
                on_completion = 'bumper_recovered'
                service_prefix = '/patroller'
                marathon_touch_gui.client.bumper_stuck(displayNo, service_prefix, on_completion)            
            
            if n_tries>1:
                self.speak_goal.text='My bumper is being pressed. Please release it so I can move on!'
                self.speaker.send_goal(self.speak_goal)
                self.speaker.wait_for_result()

	
            #if n_tries>self.MAX_BUMPER_RECOVERY_ATTEMPTS:
                #send email
            n_tries += 1
            
   
            
            
    def set_patroller_thresholds(self, max_bumper_recovery_attempts):
        if max_bumper_recovery_attempts is not None:
            self.MAX_BUMPER_RECOVERY_ATTEMPTS = max_bumper_recovery_attempts
                

class RecoverStuckOnCarpet(smach.State, Loggable):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure'])
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self._vel_cmd = Twist()
        
        


    def execute(self,userdata):
        #small forward vel to unstuck robot
        self._vel_cmd.linear.x=0.8
        self._vel_cmd.angular.z=0.4
        for i in range(0,4): 
            self.vel_pub.publish(self._vel_cmd)
            self._vel_cmd.linear.x=self._vel_cmd.linear.x-0.2
            self._vel_cmd.angular.z=self._vel_cmd.angular.z-0.2  
   #         if self.preempt_requested():
   #             self.service_preempt()
   #             return 'preempted'
            rospy.sleep(0.2)
   #     os.system("rosservice call /ros_mary 'Recovering carpet stuck'")
        self._vel_cmd.linear.x=0.0
        self._vel_cmd.angular.z=0.0
        self.vel_pub.publish(self._vel_cmd)
        
#        now = rospy.get_rostime()
 #       f = open('/home/bruno/log.txt','a')
  #      f.write(str(rospy.get_time()))
   #     f.write('\n')
    #    f.close()
        
        #check if behaviour was successful       
        if True:     
            return 'succeeded'
        else:
            return 'failure'
