# The autonomous patrolling package

This package allows for the sequential visit of a set of pre-defined waypoints in the environment. It also has two tools for creating and saving a map, and for creating and saving a set of waypoints.
It assumes that either a [morse simulation](https://github.com/strands-project/strands_morse) or the [scitos_bringup](https://github.com/strands-project/scitos_robot) have been launched


## The map_saver

### Without rumblepad

Prior to waypoint autonomous patrolling, a map of the area to be patrolled needs to be created. For that, the `map_saver` package can be used:
  
      $ roslaunch map_saver ap_map_saver.launch 
        
        
The `ap_map_saver` provides a `SaveMap` service, that takes as input a string of the form `"path to folder where to save the map"/map_name` (e.g., `/home/strands_user/map`) and creates the  `map_name.yaml` and `map_name.pgm` files in the selected folder.


### With rumblepad
  
The `ap_map_saver` can also be used in conjunction with the rumblepad, where the user can build a map by driving the robot around and then save it. To do this:
     
* Before launching `ap_map_saver.launch`, launch the teleop_app:
        
           $ roslaunch scitos_teleop teleop_joystick.launch <js:=/dev/input/"joystick name">
                
* Launch the `ap_map_saver`, with an optional argument:
        
           $ roslaunch map_saver ap_map_saver.launch <map:="path to folder where to save the map"/map_name>
                
* (Optional) Start rviz to visualize the map creation: 
     
           $ rosrun rviz rviz
                
* Drive the robot around using the rumblepad
     
* To save the map,  press the (A) button.
       
    * The optional `map` argument sets the path to save the map. This string has the same format as the one used for the `SaveMap` service. It's default value is `~/map`.


    
## The waypoint recorder
  

### Without rumblepad

Prior to waypoints autonomous patrolling, a file with the waypoints to be visited needs to be created. For that, the `waypoint_recorder` package can be used:
  
      $ roslaunch waypoint_recorder waypoint_recorder map:="file path to the map's .yaml file"
        
        
The `waypoint_recorder` provides two services:
  
* A `SaveWaypoint` service, that saves the next pose published by amcl in a list.

* A `SaveWaypointFile` service, that receives a string specifying the file path of the output file (e.g., `/home/strands_user/waypoints.csv`), and writes all the waypoints saved in the list to that file.





### With rumblepad
  
The `waypoint_recorder` can also be used in conjunction with the rumblepad, where the user can  drive the robot around, create waypoints, and then save them to a file. To do this:
     
* Before launching `waypoint_recorder.launch`, launch the teleop_app:
        
           $ roslaunch scitos_teleop teleop_joystick.launch <js:=/dev/input/"joystick name">
                
* Launch the `waypoint_recorder`, with an optional argument:
        
           $ roslaunch map_saver ap_map_saver.launch map:="file path to the map's .yaml file" <waypoints:="file path to the file where waypoints shoud be saved">
                
* (Optional) Start rviz to check if the robot is well localized, and give it a pose estimate if needed: 
     
           $ rosrun rviz rviz
                
* Drive the robot around using the rumblepad
     
* To save waypoints to the list,  press the (A) button.

* To save the list of waypoints to a file,  press the (B) button.
       
    * The optional `waypoints` argument sets the file path to save the waypoints. This string has the same format as the one used for the `SaveWaypointFile` service. It's default value is `~/waypoints.csv`.



## The waypoint patroller

Aunonomously  visits a pre-defined list of points in sequence. Assumes static map and waypoints files are given as input. To run:

* Launch the scitos 2d navigation:

           $ roslaunch scitos_2d_navigation scitos_2d_nav.launch map:="file path to the map's .yaml file"
           
  * The 2d navigation is kept separated from the autonomous patroller so one can kill the patrolling process without killing the navigation related nodes.

           
* (Optional) Start rviz to check if the robot is well localized, and give it a pose estimate if needed: 
     
           $ rosrun rviz rviz
           
  
* Launch the patroller:
  
           $ roslaunch waypoint_patroller patroller.launch <waypoints:="file path to the waypoints file">





