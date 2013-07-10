#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <ros/console.h>

#include "ap_msgs/SaveWaypoint.h"
#include "ap_msgs/SaveWaypointFile.h"

#include <ctime>
#include <vector>
#include <list>
#include <fstream>

#if WITH_TELEOP
	#include "scitos_apps_msgs/action_buttons.h"
#endif

ros::Publisher pub;
  
bool save_pose;
FILE * pFile;
std::string csv_name;
std::list<std::vector<float> > points;

void savePoint() {
	ROS_INFO("Saving next position update as waypoint.");
	save_pose = true;
}

void saveToFile(std::string file_name) {
	ROS_INFO("Saving waypoints to: %s", file_name.c_str());
	std::ofstream waypoint_file;
	waypoint_file.open (file_name.c_str());
	for (std::list<std::vector<float> >::const_iterator it = points.begin(); it != points.end(); ++it) {
	  for(int i = 0; i < it->size(); i++) {
		  waypoint_file << it->at(i);
		  if(i != it->size() -1)
			  waypoint_file << ",";
	  }
	  waypoint_file << "\n";
	}
	waypoint_file.close();
}

#if WITH_TELEOP
	void controlCallback(const scitos_apps_msgs::action_buttons::ConstPtr& msg)
	{
	  if(msg->A && !save_pose) {
		  savePoint();
	  } else if(msg->B) {
		  saveToFile(csv_name);
	  }
	}
#endif

bool saveWaypoint(ap_msgs::SaveWaypoint::Request  &req,
		ap_msgs::SaveWaypoint::Response &res)
{
	savePoint();
	return true;
}

bool saveWaypointFile(ap_msgs::SaveWaypointFile::Request  &req,
		ap_msgs::SaveWaypointFile::Response &res)
{
	saveToFile(req.file_name);
	return true;
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if(save_pose) {
	  std::vector<float> point(7);
      point[0] = msg->pose.pose.position.x;
	  point[1] = msg->pose.pose.position.y;
	  point[2] = msg->pose.pose.position.z;
	  point[3] = msg->pose.pose.orientation.x;
	  point[4] = msg->pose.pose.orientation.y;
	  point[5] = msg->pose.pose.orientation.z;
	  point[6] = msg->pose.pose.orientation.w;
    ROS_INFO("New waypoint added at: x:%f y:%f z:%f ...", msg->pose.pose.position.x,
    	      msg->pose.pose.position.y,
    	      msg->pose.pose.position.z);
    ROS_INFO("... with orientation: x:%f y:%f z:%f w:%f", msg->pose.pose.orientation.x,
    	      msg->pose.pose.orientation.y,
    	      msg->pose.pose.orientation.z,
    	      msg->pose.pose.orientation.w);
    points.push_back(point);
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
#if WITH_TELEOP
  	  ros::Subscriber sub = n.subscribe("/teleop_joystick/action_buttons", 1000, controlCallback);
#endif
  ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 1000, amclCallback);
  ros::ServiceServer waypoint = n.advertiseService("SaveWaypoint", saveWaypoint);
  ros::ServiceServer waypoint_file = n.advertiseService("SaveWaypointFile", saveWaypointFile);
	

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  

  return 0;
}