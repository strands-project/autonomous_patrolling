#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

#if WITH_SCITOS
	#include <scitos_msgs/EnableMotors.h>
	#include <scitos_msgs/ResetMotorStop.h>
	#include <scitos_msgs/EmergencyStop.h>

	ros::ServiceClient enable_client, reset_client, emergency_client;
	scitos_msgs::EnableMotors enable_srv;
	scitos_msgs::ResetMotorStop reset_srv;
	scitos_msgs::EmergencyStop emergency_srv;
#endif

ros::Publisher pub;
double l_scale_, a_scale_;
  

geometry_msgs::Twist t;

bool interrupt_broadcasting;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void controlCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //steer robot while holding dead man switch
  if(msg->buttons[4]) {
    t.linear.x = 0.9*t.linear.x + 0.1*l_scale_ * msg->axes[1];
    t.angular.z = 0.5*t.angular.z + 0.5*a_scale_ * msg->axes[0];
    interrupt_broadcasting = false;
    pub.publish(t);
  } else {
    t.linear.x = 0.0;
    t.angular.z = 0.0;
    if(interrupt_broadcasting == false){
       interrupt_broadcasting = true;
       pub.publish(t);
    }
  }
  #if WITH_SCITOS
	  //enable motors after bump and/or freerun
	  if(msg->buttons[7]) {
	    enable_srv.request.enable = true;
	    if (!enable_client.call(enable_srv))
	    {
	      ROS_ERROR("Failed to call service /enable_motors");
	    }
	    if (!reset_client.call(reset_srv))
	    {
	      ROS_ERROR("Failed to call service /reset_motorstop");
	    }
	  }
	  //disable motors to move robot manually
	  if(msg->buttons[6]) {
	    enable_srv.request.enable = false;
	    if (!enable_client.call(enable_srv))
	    {
	      ROS_ERROR("Failed to call service /enable_motors");
	    }
	  }
	  //emergency stop
	  if(msg->axes[2] == -1.0) {
	    if (!emergency_client.call(emergency_srv))
	    {
	      ROS_ERROR("Failed to call service /emergency_stop");
	    }
	  }
  #endif
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "rumble_control");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n("rumble_control");
  n.param("scale_angular", a_scale_, 1.1);
  n.param("scale_linear", l_scale_, 1.1);
  interrupt_broadcasting = false;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/joy", 1000, controlCallback);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  #if WITH_SCITOS
	  enable_client = n.serviceClient<scitos_msgs::EnableMotors>("/enable_motors");
	  reset_client = n.serviceClient<scitos_msgs::ResetMotorStop>("/reset_motorstop");
	  emergency_client = n.serviceClient<scitos_msgs::EmergencyStop>("/emergency_stop");
  #endif	

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
