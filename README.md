### Saving a map
Prior to autonomous patrolling, a map has to be created from the space that should be patrolled. To do this, execute the following steps:
* Connect a joypad to the machine. 
* Make sure `roscore` is running.
* Start the strands simulator: `rosrun strands_sim simulator.sh`
* (Optional) Test if you can move the robot with the joypad. Because the joypad sometimes has a small residual movement, we implemented a Dead man's switch. Keep the left shoulder button pressed and then move the robot with the left joystick. 
* Configure ROS to use the robot in morse: `roslaunch strands_morse_2dnav robot.launch`
* Start the gmapping package: `rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=/odom`
* (Optional) Start rviz to visualize the map creation: `rosrun rviz rviz`
* Run the mapsaver: `roslaunch map_saver joy_map_saver.launch` 
* After covering the desired area with the robot, press the (A) button on the joypad to save the map to  `<map_saver_dir>/maps/maps.pgm`. The directory can also be changed in the joy_map_saver.launch file. 








### Saving a map using the simple Strands tutorial deprecated
* roscore
* rosrun strands_sim simulator.sh
* roslaunch strands_morse_2dnav robot.launch
* rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=/odom
* rosrun rviz rviz -> drive around and see map
* When map is done save it: rosrun map_server map_saver

