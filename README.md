* roscore
* rosrun strands_sim simulator.sh
* roslaunch strands_morse_2dnav robot.launch
* rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=/odom
* rosrun rviz rviz -> drive around and see map
* When map is done save it: rosrun map_server map_saver

