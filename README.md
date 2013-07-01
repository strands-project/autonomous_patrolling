### Setting up the workspace
* Clone the github repository: `git clone https://github.com/strands-project/autonomous_patrolling.git`
* Setting up the catkin workspace
  * Change to the root directory of the repository: `cd autonomous_patrolling`
  * To enable the new emergency stop and motor reset ability the scitos_mira package has to be installed and sourced before making the autonomous_patrolling package: `source <path_to_workspace>/scitos_mira/devel/setup.bash`. If you are using the simulator and not the real robot, you do not have to source the scitos_mira package. This will prevent the emergency stop and bumper reset buttons from working. All the other functionalities will work as described.
  * Run `catkin_make` (Catkin_make builds all binary files and creates environment variables like the setup.bash)
  * Troubleshooting: It might complain about missing include directories. If this is the case, go to the specified folder and create an empty include directory.
* Setting up the rosbuild workspace  
  * Change to the rosbuild_ws directory, located in the root directory of the repository: `cd rosbuild_ws`
  * Run `rosws init . ../devel` (This sets up the initial rosbuild workspace on top of the catkin workspace.)
  * Run `rosws set patroller` (This adds the patroller project to our rosbuild workspace.)
  * Run `rosws update` (This updates the setup files.) 
* To use packages from either of both workspaces, you have to source the `setup.bash` or `setup.zsh` located in the **rosbuild_ws** in every terminal you open. This will setup both the rosbuild AND the catkin worksapce. 

### Rumblepad control
The rumblepad_control package is designed to work with a Logitech Wireless Gamepad F710.
* Source the corresponding setup.bash: `source autonomous_patrolling/rosbuild_ws/setup.bash`
* Launch the rumblepad control: `roslaunch rumblepad_control rumbelpad_control.launch`
 * If the scitos_mira drivers are running, you should now be able to crontrol the robot using the joypad.
* Controlling the robot (if you do not press any buttons, the rumbelpad control will not interfere with any autonomous behaviour but can be used to emergency stop the robot or reset the bumper after a crash):
 * In order to move the robot you have to press and hold the dead man switch which is the left top shoulder button while moving the robot around.
 * You can move the robot with the left joystick (if the mode LED on the rumbelpad is off) or with the D-Pad (if the mode LED is on). Use "Mode" button to toggle between behaviours.
 * The lower left sholder button is the emergency stop and cuts the power to the motors if press down completely.
 * To put the robot into freerun mode you can use the "Back" button on the pad.
 * To re-enable the motors after an emergency stop or hitting something with the bumper you can press the "Start" button on the pad.

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
* [Robot only] Make sure the laser is setup correctly and broadcasting to the `/scan` topic. 
* [Simulation only] Start the strands simulator: `rosrun strands_sim simulator.sh`
* [Simulation only] Configure ROS to use the robot in morse: `roslaunch strands_morse_2dnav robot.launch`
* (Optional) Start rviz to visualize the map: `rosrun rviz rviz`
* Run the waypoint recorder: `roslaunch waypoint_recorder waypoint_recorder.launch` 
* Press the (A) button to save a point to a file. Press the (B) button to finish mapping. The resulting csv file can be specified in the waypoint_recorder launch file. 

### Starting the patrolling
When a list of points has been stored, these can be patrolled by the robot. 
* Make sure `roscore` is running.
* [Robot only] Make sure the laser is setup correctly and broadcasting to the `/scan` topic. 
* [Simulation only] Start the strands simulator: `rosrun strands_sim simulator.sh`
* [Simulation only] Configure ROS to use the robot in morse: `roslaunch strands_morse_2dnav robot.launch`
* Start rviz to visualize the map and to give the robot an initial localization: `rosrun rviz rviz`
* Run the patroller: `roslaunch patroller nav.launch`
* Give the robot an initial localization in rviz. 

### General notes
* Do not switch the button at the top of the controller! This causes the keys to be mapped in a different way. It should always be in the "X" position. 
* To build the projects in the catkin workspace use `catkin_make`.
* To build the projects in the rosbuild worksapce use `rosmake <package_name>`.

### Saving a map using the simple Strands tutorial (deprecated)
* roscore
* rosrun strands_sim simulator.sh
* roslaunch strands_morse_2dnav robot.launch
* rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=/odom
* rosrun rviz rviz -> drive around and see map
* When map is done save it: rosrun map_server map_saver

