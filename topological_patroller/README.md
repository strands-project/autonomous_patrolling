#TOPOLOGICAL PATROLLER

To run the topological topological patroller you will need to have the topological navigation and localisation running 
and to create a file like this:

```
CheckPoint:
	WayPoint1
	Action:
		sleep, 1
CheckPoint:
	WayPoint2
	Action:
		sleep, 1
CheckPoint:
	WayPoint3
	Action:
		sleep, 1
CheckPoint:
	WayPoint4
	Action:
		sleep, 1
```

On which every checkpoint is a waypoint to visit on every round, the first line is the name of the node, 
then on the Action fields add the Actions you want to be launched at each node (on this version try only sleep), 
with its arguments.

Once this is done you can launch the topological patroller running this command:

`rosrun topological_patroller node_patroller.py name_of_the_file.task`

This will make the robot to visit every waypoint in the list in a Random way and then dock wait until the minute is even 
and undock again and run the same process again.
