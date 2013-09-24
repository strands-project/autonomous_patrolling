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
bool save_pt;
FILE * pFile;
std::string csv_name;
std::list<std::vector<float> > points;

int startPanAngle, endPanAngle, panIncrementAngle;
int startTiltAngle, endTiltAngle, tiltIncrementAngle;


void savePoint() {
	ROS_INFO("Saving next position update as waypoint.");
	save_pose = true;
}


void savePointWithPT() {
	ROS_INFO("Saving next position update as waypoint with pan tilt.");
	save_pose = true;
	save_pt = true;
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
	  } else if (msg->X){
		savePointWithPT();
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

void amclCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	if(save_pose) {
		std::vector<float> point(7);
		point[0] = msg->position.x;
		point[1] = msg->position.y;
		point[2] = msg->position.z;
		point[3] = msg->orientation.x;
		point[4] = msg->orientation.y;
		point[5] = msg->orientation.z;
		point[6] = msg->orientation.w;
		ROS_INFO("New waypoint added at: x:%f y:%f z:%f ...", msg->position.x,
				msg->position.y,
				msg->position.z);
		ROS_INFO("... with orientation: x:%f y:%f z:%f w:%f", msg->orientation.x,
				msg->orientation.y,
				msg->orientation.z,
				msg->orientation.w);

		if (save_pt) // add the pan tilt information to this waypoint 
 		{
			point.push_back(startPanAngle);
			point.push_back(panIncrementAngle);
			point.push_back(endPanAngle);
			point.push_back(startTiltAngle);
			point.push_back(tiltIncrementAngle);
			point.push_back(endTiltAngle);
			save_pt = false;
		
			ROS_INFO("... with Pan Tilt parameteers: pan_start:%d pan_inc:%d pan_end:%d tilt_start:%d tilt_inc:%d tilt_end:%d", startPanAngle,
				panIncrementAngle,
				endPanAngle,
				startTiltAngle, tiltIncrementAngle, endTiltAngle);
		}

		points.push_back(point);
		save_pose = false;
	}
}

int main(int argc, char **argv)
{


  //Check if waypoints name was given as argument to the launch file, and create default waypoints name otherwise
  csv_name=std::string(argv[1]);
  if (!csv_name.compare(std::string("default_waypoints_name")))  {
    ROS_WARN("No file name given for waypoints, waypoints will be saved with default name on home directory");
    char buff[20];
    std::string home(getenv("HOME"));
    home+="/";
    time_t now = time(NULL);
    strftime(buff, 20, "%Y_%m_%d_%H_%M_%S", localtime(&now));
    csv_name = home + std::string(buff) + std::string("waypoints.csv");
  }

  // Initialize pan tilt parameters

  startPanAngle = -120;
  endPanAngle = 120;
  panIncrementAngle = 30;

  startTiltAngle = -30; 
  endTiltAngle = 30; 
  tiltIncrementAngle = 15;
  
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
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("pan_start", startPanAngle, int(-120.0)); 
  private_node_handle_.param("pan_end", endPanAngle, int(120.0)); 
  private_node_handle_.param("pan_step", panIncrementAngle, int(30.0)); 
  private_node_handle_.param("tilt_start", startTiltAngle, int(-30.0)); 
  private_node_handle_.param("tilt_end", endTiltAngle, int(30.0)); 
  private_node_handle_.param("tilt_step", tiltIncrementAngle, int(15.0)); 

/*
  std::cout<<"Start pan angle "<<startPanAngle<<std::endl;
  std::cout<<"End pan angle "<<endPanAngle<<std::endl;
  std::cout<<"Step pan angle "<<panIncrementAngle<<std::endl;
  std::cout<<"Start tilt angle "<<startTiltAngle<<std::endl;
  std::cout<<"End tilt angle "<<endTiltAngle<<std::endl;
  std::cout<<"Step tilt angle "<<tiltIncrementAngle<<std::endl;
*/
  
  
//   n.param("csv_name", csv_name, std::string("~/waypoints.csv"));
//   char buff[20];
//   time_t now = time(NULL);
//   strftime(buff, 20, "%Y_%m_%d_%H_%M_%S", localtime(&now));
//   csv_name += std::string(buff);
//   csv_name += ".csv";


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
  ros::Subscriber sub_amcl = n.subscribe("/robot_pose", 1000, amclCallback);
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
