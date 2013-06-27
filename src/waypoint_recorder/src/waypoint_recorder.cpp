#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <ros/console.h>

#include <ctime>

ros::Publisher pub;
  
bool save_pose;
FILE * pFile;
std::string csv_name;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void controlCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if(msg->buttons[0]) {
    save_pose = true;
  } else if(msg->buttons[1]) {
    fclose (pFile);
    ros::shutdown();
  }
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if(save_pose) {
    fprintf(pFile, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n", 
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    save_pose = false;
  }
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
  ros::init(argc, argv, "waypoint_recorder");
  save_pose = false;
  
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n("waypoint_recorder");
  n.param("csv_name", csv_name, std::string("~/waypoints.csv"));
  char buff[20];
  time_t now = time(NULL);
  strftime(buff, 20, "%Y_%m_%d_%H_%M_%S", localtime(&now));
  csv_name += std::string(buff);
  csv_name += ".csv";
  pFile = fopen (csv_name.c_str(),"a");


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
  ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 1000, amclCallback);
	

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  

  return 0;
}
