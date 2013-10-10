# The autonomous patrolling package
           

This package allows for the sequential or randomized visit of a set of pre-defined waypoints in the environment, along with a charging behaviour when the battery level drops below a given treshold. It also has two tools for creating and saving a map, and for creating and saving a set of waypoints.
It assumes that either a [morse simulation](https://github.com/strands-project/strands_morse) or the [scitos_bringup](https://github.com/strands-project/scitos_robot) have been launched


## The map_saver

### Without rumblepad

Prior to waypoint autonomous patrolling, a map of the area to be patrolled needs to be created. For that, the `map_saver` package can be used:
  
      $ roslaunch map_saver ap_map_saver.launch 
        
        
The `ap_map_saver` provides a `SaveMap` service, that takes as input a string of the form `"path to folder where to save the map"/map_name` (e.g., `/home/strands_user/map`) and creates the  `map_name.yaml` and `map_name.pgm` files in the selected folder.


### With rumblepad
  
The `ap_map_saver` can also be used in conjunction with the rumblepad, where the user can build a map by driving the robot around and then save it. To do this:
     
* Before launching `ap_map_saver.launch`, launch the teleop_app:
        
           $ roslaunch scitos_teleop teleop_joystick.launch js:=/dev/input/"joystick name"
                
* Launch the `ap_map_saver`, with an optional argument:
        
           $ roslaunch map_saver ap_map_saver.launch map:="path to folder where to save the map"/map_name
                
* (Optional) Start rviz to visualize the map creation: 
     
           $ rosrun rviz rviz
                
* Drive the robot around using the rumblepad
     
* To save the map,  press the (A) button.
       
    * The optional `map` argument sets the path to save the map. This string has the same format as the one used for the `SaveMap` service. It's default value is `~/map`.


    
## The waypoint recorder
The waypoint recorder uses the robot_pose topic instead of the amcl_pose topic. This makes saving accuracy waypoints easier as the updates are comming in more frequently. In order for poses to be published on this topic run the robot_pose_publisher:
      
      $ rosrun robot_pose_publisher robot_pose_publisher  

### Without rumblepad

Prior to waypoints autonomous patrolling, a file with the waypoints to be visited needs to be created. For that, the `waypoint_recorder` package can be used:
  
      $ roslaunch waypoint_recorder waypoint_recorder map:="file path to the map's .yaml file"
        
        
The `waypoint_recorder` provides two services:
  
* A `SaveWaypoint` service, that saves the next pose published by amcl in a list.

* A `SaveWaypointFile` service, that receives a string specifying the file path of the output file (e.g., `/home/strands_user/waypoints.csv`), and writes all the waypoints saved in the list to that file.





### With rumblepad
  
The `waypoint_recorder` can also be used in conjunction with the rumblepad, where the user can  drive the robot around, create waypoints, and then save them to a file. To do this:
     
* Before launching `waypoint_recorder.launch`, launch the teleop_app:
        
           $ roslaunch scitos_teleop teleop_joystick.launch js:=/dev/input/"joystick name"
                
* Launch the `waypoint_recorder`, with an optional argument:
        
           $ rosrun waypoint_recorder waypoint_recorder.launch map:="file path to the map's .yaml file" waypoints:="file path to the file where waypoints shoud be saved"
                
* (Optional) Start rviz to check if the robot is well localized, and give it a pose estimate if needed: 
     
           $ rosrun rviz rviz
                
* Drive the robot around using the rumblepad
     
* To save waypoints to the list,  press the (A) button. NOTE: The FIRST point you save should be the one in front of your charging station, so that the robot knows where to navigate when it needs to charge.

* To save the list of waypoints to a file,  press the (B) button.
       
    * The optional `waypoints` argument sets the file path to save the waypoints. This string has the same format as the one used for the `SaveWaypointFile` service. It's default value is `~/waypoints.csv`.



## The waypoint patroller

Aunonomously  visits a pre-defined list of points randomly or in sequence. Goes to charge when battery drops below a given treshold and after it is recharged, continues the patrolling. Assumes static map and waypoints set are given as input.

* Launch the scitos 2d navigation:

           $ roslaunch scitos_2d_navigation scitos_2d_nav.launch map:="file path to the map's .yaml file"
           
  * The 2d navigation is kept separated from the autonomous patroller so one can kill the patrolling process without killing the navigation related nodes.

           
* (Optional) Start rviz to check if the robot is well localized, and give it a pose estimate if needed: 
     
           $ rosrun rviz rviz
      
* Launch the strands_datacentre:
```
HOSTNAME=bob roslaunch strands_datacentre datacentre.launch
```


* Run the autonomous docking service:

           $ roslaunch scitos_docking charging.launch
           
           or:
           
           $ rosrun scitos_docking visual_charging
           
           


* Calibrate the docking as detailed in the scitos_docking package:
```bash
rosrun actionlib axclient.py /chargingServer
```
Then in the `Goal` textfield complete as follows:
```
Command: calibrate
Timeout: 1000
```
Then press the `SEND GOAL` button.

* If you already have waypoints in your datacentre, proceed to execute. Otherwise, to insert the waypoints from a waypoint log file (as created using the waypoint recorder) into your datacentre:
```bash
rosrun waypoint_recorder insert_in_db.py waypoints.csv waypoint_set_name map_name
```
Currently map_name is unimportant (until the maps are also stored in db), and the first waypoint is assumed to be the pre-charging waypoint. 

* Launch the patroller:
  
           $ roslaunch waypoint_patroller long_term_patroller.launch waypoints:=waypoint_set_name <randomized:="value"> <n_it:="number of iterations">

           
   * The optional argument randomized can be true or false. Default is true. If false is given, then the points are visited sequentially
   * The optional argument n_it specifies how many complete iterations of all the points should be done before the patroller outputs succeeded. Default is -1, which means infinite iterations
   * NOTE: For the robot to speak and ask for help, the user that launches this file needs to be logged in the robot's computer. This is related to [this](https://github.com/strands-project/strands_hri/issues/7)



