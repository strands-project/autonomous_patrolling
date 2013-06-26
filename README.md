### Saving a map
Prior to autonomous patrolling, a map has to be created from the space that should be patrolled. To do this, execute the following steps:
* Connect a joypad to the machine. 
* Make sure `roscore` is running.
* [Simulation only] Start the strands simulator: `rosrun strands_sim simulator.sh`
* (Optional) Test if you can move the robot with the joypad. Because the joypad sometimes has a small residual movement, we implemented a Dead man's switch. Keep the left shoulder button pressed and then move the robot with the left joystick. 
* [Simulation only] Configure ROS to use the robot in morse: `roslaunch strands_morse_2dnav robot.launch`
* Start the gmapping package: `rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=/odom`
* (Optional) Start rviz to visualize the map creation: `rosrun rviz rviz`
* Run the mapsaver: `roslaunch map_saver joy_map_saver.launch` 
* After covering the desired area with the robot, press the (A) button on the joypad to save the map to  `<map_saver_dir>/maps/maps.pgm`. The directory can also be changed in the joy_map_saver.launch file. 


### Saving points from the map
After the map has been created, save some points within the map. The robot uses AMCL to localize itself and the result can be stored by button presses.
* The map which is used for localization is specified in the map_saver package in the map directory (maps.pgm and maps.yaml). If another map should be used, this should be set in the waypoint_recorder launch file. 
* Connect a joypad to the machine. 
* Make sure `roscore` is running.
* [Simulation only] Start the strands simulator: `rosrun strands_sim simulator.sh`
* [Simulation only] Configure ROS to use the robot in morse: `roslaunch strands_morse_2dnav robot.launch`
* (Optional) Start rviz to visualize the map creation: `rosrun rviz rviz`
* Run the mapsaver: `roslaunch waypoint_recorder waypoint_recorder.launch` 
* [Robot only] Start up the laser. 
* Press the (A) button to save a point to a file. Press the (B) button to finish mapping. The resulting csv file can be specified in the waypoint_recorder launch file. 

 



### Saving a map using the simple Strands tutorial deprecated
* roscore
* rosrun strands_sim simulator.sh
* roslaunch strands_morse_2dnav robot.launch
* rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=/odom
* rosrun rviz rviz -> drive around and see map
* When map is done save it: rosrun map_server map_saver

